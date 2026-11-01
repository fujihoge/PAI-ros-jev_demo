// M5 試作: ROS 2 トピックを受けて Gazebo Classic の visual（色・表示/非表示）を変える world プラグイン。
// 状態をプラグインが持ち、gzclient が接続して connect_delay 秒（既定 8 秒）経つと全状態を再送し、以後は
// period 秒（既定 1 秒）ごとに再送する。~/visual は状態を保持しないため、あとから gzclient をつないでも
// 正しい見た目になる。物理（衝突・質量）には触れない。
//
// ROS 側（std_msgs/String）:
//   /m5/lamp_cmd   "M1:green" | "M3:orange" | "M2:red" | "M4:0.2,0.4,1"   機械ランプの色
//   /m5/cargo_cmd  "show" | "hide" | "show:green" | "hide:orange"          積載箱の表示/非表示（と色）
//   /m5/wip_cmd    "M1_M2:brown" | "M2_M3:0.7,0.7,0.72"                    仕掛かり置き場の平板の色（conv_M1_M2::wip::wip_M1_M2）
//   /m5/work_cmd   "M1:show" | "M1:hide"                                   機械の中のワーク箱（M1::work::work）の表示/非表示
//   /m5/banner_cmd "show" | "hide"                                         完成表示板（completed_banner の board と支柱）の表示/非表示
//   /m5/flow_cmd   "2|x0,y0;x1,y1;..."                                    ワークの移動表示（見た目のみ）。ベルト中心線（world 座標 m）を秒数で等速に動く黄色い箱を出す。
//                  world の <flow_pieces>N</flow_pieces>（flow_piece0..N-1::piece::piece、同時 N 個まで）と <flow_z>（高さ）が要る。
//   /m5/stock_cmd  "M1:3" | "M2:0"                                         機械の部材在庫（オレンジの小立方体 M1::stock::s0..s{cap-1}）の先頭 3 個だけ表示、残りは非表示
//   world の <stock_caps>M1:3 M2:9 M3:6 M4:12</stock_caps> で各機械の立方体の個数（容量）を与える。起動時は全部非表示。
//   world の <work_ids>M1 M2 M3 M4</work_ids> を与えると、起動時にワーク箱と表示板を「非表示」状態で持つ（指令が来るまで隠す）。
// 色名: green orange red blue yellow gray brown。または "r,g,b"。
// 本番統合（2026-10-12）: factory_sim パッケージの colcon ビルドに組み込み、world ファイルに <plugin> が含まれる。
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <ignition/math/Color.hh>
#include <map>
#include <vector>
#include <mutex>
#include <sstream>

namespace m5 {

static bool ParseColor(const std::string & s, double c[3]) {
  static const std::map<std::string, std::array<double, 3>> named = {
    {"green", {0.1, 0.9, 0.2}}, {"orange", {1.0, 0.5, 0.0}}, {"red", {1.0, 0.1, 0.1}},
    {"blue", {0.2, 0.4, 1.0}}, {"yellow", {0.95, 0.85, 0.1}},
    {"gray", {0.35, 0.35, 0.38}}, {"brown", {0.55, 0.33, 0.12}}};
  auto it = named.find(s);
  if (it != named.end()) { for (int i = 0; i < 3; ++i) c[i] = it->second[i]; return true; }
  char ch;
  std::istringstream is(s);
  return static_cast<bool>(is >> c[0] >> ch >> c[1] >> ch >> c[2]);
}

class VisualState : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    period_ = sdf->HasElement("period") ? sdf->Get<double>("period") : 1.0;
    connect_delay_ = sdf->HasElement("connect_delay") ? sdf->Get<double>("connect_delay") : 8.0;
    cargo_name_ = sdf->HasElement("cargo_visual") ? sdf->Get<std::string>("cargo_visual") :
      "factory_robot::base_footprint::base_footprint_fixed_joint_lump__cargo_box_link_visual_2";
    cargo_parent_ = cargo_name_.substr(0, cargo_name_.find("::", cargo_name_.find("::") + 2));
    if (sdf->HasElement("work_ids")) {
      std::istringstream is(sdf->Get<std::string>("work_ids"));
      std::string id;
      while (is >> id) SetWork(id, false);
      SetBanner(false);
    }
    if (sdf->HasElement("stock_caps")) {
      std::istringstream is(sdf->Get<std::string>("stock_caps"));
      std::string tok;
      while (is >> tok) {
        auto p = tok.find(':');
        if (p == std::string::npos) continue;
        stock_caps_[tok.substr(0, p)] = std::atoi(tok.c_str() + p + 1);
      }
      for (auto & kv : stock_caps_) SetStock(kv.first, 0);
    }
    flow_n_ = sdf->HasElement("flow_pieces") ? sdf->Get<int>("flow_pieces") : 0;
    flow_z_ = sdf->HasElement("flow_z") ? sdf->Get<double>("flow_z") : 0.4;
    pieces_.resize(flow_n_);
    for (int k = 0; k < flow_n_; ++k) SetPiece(k, false);
    gz_node_ = boost::make_shared<gazebo::transport::Node>();
    gz_node_->Init(world->Name());
    pub_ = gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");
    ros_ = gazebo_ros::Node::Get(sdf);
    lamp_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/lamp_cmd", 10, [this](std_msgs::msg::String::SharedPtr m) { OnLamp(m->data); });
    wip_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/wip_cmd", 10, [this](std_msgs::msg::String::SharedPtr m) { OnWip(m->data); });
    cargo_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/cargo_cmd", 10, [this](std_msgs::msg::String::SharedPtr m) { OnCargo(m->data); });
    work_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/work_cmd", 10, [this](std_msgs::msg::String::SharedPtr m) { OnWork(m->data); });
    stock_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/stock_cmd", 10, [this](std_msgs::msg::String::SharedPtr m) { OnStock(m->data); });
    flow_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/flow_cmd", 50, [this](std_msgs::msg::String::SharedPtr m) { OnFlow(m->data); });
    banner_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/m5/banner_cmd", 10, [this](std_msgs::msg::String::SharedPtr m) { SetBanner(m->data != "hide"); });
    upd_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo & i) { OnUpdate(i); });
  }

 private:
  static void SetColor(gazebo::msgs::Visual & v, const double c[3], bool emissive) {
    auto * mat = v.mutable_material();
    // ランプは emissive 主体（明るさで色が飽和して橙が黄色に見えるのを避ける）
    const double k = emissive ? 0.3 : 1.0;
    gazebo::msgs::Set(mat->mutable_ambient(), ignition::math::Color(c[0] * k, c[1] * k, c[2] * k, 1));
    gazebo::msgs::Set(mat->mutable_diffuse(), ignition::math::Color(c[0] * k, c[1] * k, c[2] * k, 1));
    if (emissive) gazebo::msgs::Set(mat->mutable_emissive(), ignition::math::Color(c[0], c[1], c[2], 1));
  }
  void Store(const std::string & key, const gazebo::msgs::Visual & v) {
    std::lock_guard<std::mutex> l(mu_);
    states_[key] = v;
    pending_.push_back(v);
  }
  void OnLamp(const std::string & s) {
    RCLCPP_DEBUG(ros_->get_logger(), "lamp cmd: %s", s.c_str());
    auto p = s.find(':');
    if (p == std::string::npos) return;
    std::string id = s.substr(0, p);
    double c[3];
    if (!ParseColor(s.substr(p + 1), c)) return;
    gazebo::msgs::Visual v;
    v.set_name(id + "::lamp::lamp");
    v.set_parent_name(id + "::lamp");
    SetColor(v, c, true);
    Store(v.name(), v);
  }
  // "M1_M2:brown" -> visual conv_M1_M2::wip::wip_M1_M2 (parent conv_M1_M2::wip). Plain colour, no emission.
  void OnWip(const std::string & s) {
    RCLCPP_DEBUG(ros_->get_logger(), "wip cmd: %s", s.c_str());
    auto p = s.find(':');
    if (p == std::string::npos) return;
    std::string pair = s.substr(0, p);
    double c[3];
    if (!ParseColor(s.substr(p + 1), c)) return;
    gazebo::msgs::Visual v;
    v.set_name("conv_" + pair + "::wip::wip_" + pair);
    v.set_parent_name("conv_" + pair + "::wip");
    SetColor(v, c, false);
    Store(v.name(), v);
  }
  // "M1:show" -> visual M1::work::work (parent M1::work)
  void OnWork(const std::string & s) {
    RCLCPP_DEBUG(ros_->get_logger(), "work cmd: %s", s.c_str());
    auto p = s.find(':');
    if (p == std::string::npos) return;
    SetWork(s.substr(0, p), s.substr(p + 1) != "hide");
  }
  void SetWork(const std::string & id, bool visible) {
    gazebo::msgs::Visual v;
    v.set_name(id + "::work::work");
    v.set_parent_name(id + "::work");
    v.set_visible(visible);
    Store(v.name(), v);
  }
  // ---- ワークの移動表示（見た目のみ）。flow_piece{k}::piece::piece の pose を period_flow_ 秒ごとに ~/visual で送る ----
  gazebo::msgs::Visual PieceMsg(int k, bool visible) const {
    gazebo::msgs::Visual v;
    v.set_name("flow_piece" + std::to_string(k) + "::piece::piece");
    v.set_parent_name("flow_piece" + std::to_string(k) + "::piece");
    v.set_visible(visible);
    return v;
  }
  void SetPiece(int k, bool visible) { Store(PieceMsg(k, visible).name(), PieceMsg(k, visible)); }
  // "2|x0,y0;x1,y1;..." : 空いている箱に割り当てて、受信時刻（sim 時間）から等速で動かす。空きが無ければ無視。
  void OnFlow(const std::string & s) {
    auto bar = s.find('|');
    if (bar == std::string::npos) return;
    Piece np;
    np.dur = std::atof(s.substr(0, bar).c_str());
    std::istringstream is(s.substr(bar + 1));
    std::string tok;
    while (std::getline(is, tok, ';')) {
      double x, y;
      char ch;
      std::istringstream ts(tok);
      if (ts >> x >> ch >> y) np.pts.push_back({x, y});
    }
    if (np.dur <= 0 || np.pts.size() < 2) return;
    np.cum.push_back(0.0);
    for (size_t i = 1; i < np.pts.size(); ++i)
      np.cum.push_back(np.cum.back() + std::hypot(np.pts[i][0] - np.pts[i - 1][0], np.pts[i][1] - np.pts[i - 1][1]));
    if (np.cum.back() <= 0) return;
    std::lock_guard<std::mutex> l(mu_);
    if (!ready_) return;                       // 画面が無い（headless）か接続直後は何もしない
    np.t0 = world_->SimTime().Double();
    np.active = true;
    for (int k = 0; k < flow_n_; ++k) {
      if (!pieces_[k].active) {
        pieces_[k] = np;
        states_.erase("flow_piece" + std::to_string(k) + "::piece::piece");   // 全再送で隠れ状態を送り返さない
        return;
      }
    }
    RCLCPP_WARN(ros_->get_logger(), "flow: no free piece (%d busy), dropped", flow_n_);
  }
  // mu_ を持って呼ぶ
  void UpdatePieces(double st) {
    if (flow_n_ == 0 || st - last_flow_ < 0.04) return;   // 25 Hz (sim 時間)
    last_flow_ = st;
    for (int k = 0; k < flow_n_; ++k) {
      Piece & p = pieces_[k];
      if (!p.active) continue;
      double f = (st - p.t0) / p.dur;
      auto v = PieceMsg(k, f < 1.0);
      if (f >= 1.0) {
        p.active = false;
        states_[v.name()] = v;
      } else {
        double want = std::max(0.0, f) * p.cum.back();
        size_t i = 1;
        while (i + 1 < p.pts.size() && p.cum[i] < want) ++i;
        double seg = p.cum[i] - p.cum[i - 1], u = seg > 0 ? (want - p.cum[i - 1]) / seg : 1.0;
        double x = p.pts[i - 1][0] + (p.pts[i][0] - p.pts[i - 1][0]) * u;
        double y = p.pts[i - 1][1] + (p.pts[i][1] - p.pts[i - 1][1]) * u;
        gazebo::msgs::Set(v.mutable_pose(), ignition::math::Pose3d(x, y, flow_z_, 0, 0, 0));
      }
      pub_->Publish(v);
    }
  }
  // "M1:3" -> visuals M1::stock::s0..s{cap-1} (parent M1::stock): the first n visible, the rest hidden
  void OnStock(const std::string & s) {
    RCLCPP_DEBUG(ros_->get_logger(), "stock cmd: %s", s.c_str());
    auto p = s.find(':');
    if (p == std::string::npos) return;
    SetStock(s.substr(0, p), std::atoi(s.c_str() + p + 1));
  }
  void SetStock(const std::string & id, int n) {
    auto it = stock_caps_.find(id);
    if (it == stock_caps_.end()) return;
    for (int k = 0; k < it->second; ++k) {
      gazebo::msgs::Visual v;
      v.set_name(id + "::stock::s" + std::to_string(k));
      v.set_parent_name(id + "::stock");
      v.set_visible(k < n);
      Store(v.name(), v);
    }
  }
  // completed_banner::banner::{board,post_l,post_r}
  void SetBanner(bool visible) {
    for (const char * n : {"board", "post_l", "post_r"}) {
      gazebo::msgs::Visual v;
      v.set_name(std::string("completed_banner::banner::") + n);
      v.set_parent_name("completed_banner::banner");
      v.set_visible(visible);
      Store(v.name(), v);
    }
  }
  void OnCargo(const std::string & s) {
    RCLCPP_DEBUG(ros_->get_logger(), "cargo cmd: %s", s.c_str());
    auto p = s.find(':');
    std::string mode = s.substr(0, p);
    gazebo::msgs::Visual v;
    {  // 既存の状態を引き継ぐ（色だけ変える/表示だけ変えるを可能に）
      std::lock_guard<std::mutex> l(mu_);
      auto it = states_.find(cargo_name_);
      if (it != states_.end()) v = it->second;
    }
    v.set_name(cargo_name_);
    v.set_parent_name(cargo_parent_);
    v.set_visible(mode != "hide");
    if (p != std::string::npos) {
      double c[3];
      if (ParseColor(s.substr(p + 1), c)) SetColor(v, c, false);
    }
    Store(cargo_name_, v);
  }
  // gzclient が接続した直後は Scene の visual がまだ無く、その時点で届いた ~/visual は「新規 visual の作成」
  // として扱われて以後の更新が効かなくなる（実測）。接続から connect_delay_ 秒（実時間）経ってから送る。
  void OnUpdate(const gazebo::common::UpdateInfo & info) {
    std::lock_guard<std::mutex> l(mu_);
    const auto now = std::chrono::steady_clock::now();
    if (!pub_->HasConnections()) {
      connected_ = false;
      ready_ = false;
      pending_.clear();
      for (int k = 0; k < flow_n_; ++k) {
        if (pieces_[k].active) { pieces_[k].active = false; states_["flow_piece" + std::to_string(k) + "::piece::piece"] = PieceMsg(k, false); }
      }
      return;
    }
    if (!connected_) { connected_ = true; conn_since_ = now; }
    if (!ready_) {
      if (std::chrono::duration<double>(now - conn_since_).count() < connect_delay_) { pending_.clear(); return; }
      ready_ = true;
      RCLCPP_INFO(ros_->get_logger(), "client ready: full republish of %d states", static_cast<int>(states_.size()));
      last_ = -1e9;  // 直ちに全再送
    }
    for (auto & v : pending_) pub_->Publish(v);
    if (!pending_.empty()) last_ = info.simTime.Double();
    pending_.clear();
    UpdatePieces(info.simTime.Double());
    if (info.simTime.Double() - last_ >= period_) {
      for (auto & kv : states_) pub_->Publish(kv.second);
      last_ = info.simTime.Double();
    }
  }
  gazebo::physics::WorldPtr world_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::PublisherPtr pub_;
  gazebo_ros::Node::SharedPtr ros_;
  struct Piece {
    bool active = false;
    double t0 = 0, dur = 1;
    std::vector<std::array<double, 2>> pts;
    std::vector<double> cum;
  };
  std::vector<Piece> pieces_;
  int flow_n_ = 0;
  double flow_z_ = 0.4, last_flow_ = -1e9;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flow_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lamp_sub_, cargo_sub_, wip_sub_, work_sub_, stock_sub_, banner_sub_;
  gazebo::event::ConnectionPtr upd_;
  std::mutex mu_;
  std::map<std::string, gazebo::msgs::Visual> states_;
  std::vector<gazebo::msgs::Visual> pending_;
  double period_ = 1.0, last_ = 0.0, connect_delay_ = 8.0;
  bool connected_ = false, ready_ = false;
  std::chrono::steady_clock::time_point conn_since_;
  std::string cargo_name_, cargo_parent_;
  std::map<std::string, int> stock_caps_;
};
GZ_REGISTER_WORLD_PLUGIN(VisualState)
}  // namespace m5
