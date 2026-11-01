# 部材補給 AMR の次の仕事を Jev が選ぶ

## 概要

1 台の AMR（自律搬送ロボット）が、直列ライン M1→M2→M3→M4 に部材を供給し、不良品（NG）を回収し、完成品を出荷する。次にどの仕事をするか（M1〜M4 への供給、NG 回収、出荷）を、TypeSafe の判断 API（Jev、`typesafe-sdk`）に選ばせる。

使った版は `policies/jev_b1.py` の `b1x` である。出荷のタイミングはコードが決め、それ以外の選択を Jev に聞く。聞くのは 1 文の問いと、候補ごとの事実だけである。

計算モデル（`factory_b/engine.py`）で 1800 秒の 1 回の走行を行い、Jev の判断を記録した。その結果は完成数 20 個だった。Gazebo Classic 11 では、この記録済みの判断列を再生して、ロボットの動きとラインの様子を表示する。Gazebo の側では API を呼ばない。

この版は課題 B1 のデモ用であり、他の方式との比較に使うコードは含めていない（「注意」を参照）。

## 構成

ROS 2 側は次の 4 つの要素からなる（パッケージ `factory_sim`）。

| 要素 | 役割 |
|---|---|
| `b1_node` | 基本機能。計算モデルのライン状態を進め、ロボットを走らせ、判断が要るとき `/factory/state` に判断要求を載せる。Gazebo の見た目（ランプ、在庫、積荷）の指令も出す |
| `decision_node` | 判断ノード。`/factory/state` の判断要求に `/dispatch/decision` で答える。`policy:=replay`（記録の再生、既定）か `policy:=jev_b1x`（Jev を呼ぶ）。`b1_node` が 8 秒（実時間）待っても答えが来ないときは、`b1_node` 側が時間切れの代替（`policies/jev_b1.py` の `fallback_choice`、余裕時間が最も小さい候補）を使う |
| `safety_stop` | 安全層。LiDAR の `/scan` を見て、`/cmd_vel_raw` を `/cmd_vel` に通す。前方に障害物があれば停止する |
| Gazebo（`gzserver`） | 世界、差動 2 輪ロボット、LiDAR、機械の表示用プラグイン（`m5_visual_state`） |

主なトピック:

| トピック | 向き | 内容 |
|---|---|---|
| `/factory/state` | `b1_node` → `decision_node` | ラインの状態（1 Hz と変化時）。判断が要るあいだは `decision_request`（候補と事実）を含む |
| `/dispatch/decision` | `decision_node` → `b1_node` | 判断の答え（選んだ仕事、方式、規則、Jev の確率など） |
| `/cmd_vel_raw` | `b1_node` → `safety_stop` | 航法の速度指令（安全層の前） |
| `/cmd_vel` | `safety_stop` → Gazebo | 安全層を通した速度指令 |
| `/scan` | Gazebo → `safety_stop` | LiDAR |
| `/model_states` | Gazebo → `b1_node` | ロボットの位置（真値） |
| `/odom` | Gazebo → `b1_node` | オドメトリ（速度） |

このほか、見た目だけの指令 `/m5/lamp_cmd` などを `b1_node` が Gazebo のプラグインに送る。メッセージは `std_msgs/String` に JSON を入れた形式である（`ros_ws/src/factory_sim/factory_sim/decision_core.py` の冒頭に仕様を書いた）。

## Jev に渡すもの

- 問いは次の 1 文だけである。優先順位の決まり（どの機械を先にする、など）は渡していない。

  `Which job should the robot do next to keep every machine running for the rest of the shift?`

- 候補ごとの事実は、すべてコードで計算した数値か短い文字列で、null はない。主な項目:
  - 供給: 到着までの秒数、投入ロックの残り時間と到着時の待ち、部材が届く時刻、ロボットが空く時刻、在庫（個数、1 サイクルの必要数、何サイクル分か）、供給しないと対象が止まるまでの秒数、余裕時間、選んだ場合の対象の遊び時間
  - NG 回収: 到着までの秒数、回収完了の時刻、対象が再開できない状態か、次の 1 個も不良だった場合に止まるまでの秒数、余裕時間
- 状態として、時刻と残り時間、各機械の状態、完成品置き場の個数と空き、M4 が詰まるまでの秒数も渡す。
- 出荷のタイミングは Jev に聞かず、コードが決める。完成品置き場が満杯、または M4 が詰まるまでの時間が、他の有用な仕事 1 つ分の所要時間より短いとき、コードが出荷を選ぶ。それ以外のときは出荷を候補から外して、残りを Jev に聞く。候補が 1 つだけのときは聞かずに決める（`policies/jev_b1.py` の冒頭に規則を書いた）。

## ディレクトリ構成

```
factory_b/            計算モデル（離散事象）。engine.py, variants.py（B1 の設定）, defects.py（不良の系列）
policies/jev_b1.py    b1x 方式：状態と事実の作成、出荷の規則、Jev への問い合わせ、時間切れの代替
adapters/jev.py       Jev の接続部（typesafe-sdk の呼び出し、モック）。API キーは環境変数から SDK が読む
scripts/run_b1x_episode.py   1 回の走行の入口（replay / mock / real）
recorded/b1x_seed5_empty.json  記録済みの判断列（133 回）と出荷イベント
tests/                計算モデルと b1x のテスト（API 不要）
ros_ws/src/factory_sim/  ROS 2 パッケージ
  factory_sim/        ノードの本体（b1_node, decision_node）と部品（decision_core, b1_runner, nav_driver など）
  scripts/            実行ファイル（b1_node.py, decision_node.py, safety_stop.py）
  launch/             sim.launch.py, b1_episode.launch.py
  worlds/ urdf/ plugin/ media/   世界、ロボット、Gazebo プラグイン（C++）、表示板のテクスチャ
  test/               部品のテスト（rclpy 不要）
```

## 必要環境

- ROS 2 Humble、Gazebo Classic 11（Gazebo での再生に必要。Ubuntu 22.04 を想定）
- Python 3.10 以上
- `typesafe-sdk` 0.7.2 以上（PyPI。Jev を実際に呼ぶときだけ必要。記録の再生とテストには要らない）
- テストには `pytest`

`typesafe-sdk` は同梱していない。PyPI から入れる。

## 再現手順

### 1. 計算モデルで記録を再生する（API 不要）

リポジトリの直下で実行する。

```
python scripts/run_b1x_episode.py replay
```

記録済みの判断列（`recorded/b1x_seed5_empty.json`）を順に適用し、`finished = 20` と判断回数 133 を表示する。条件は seed 5、在庫なしから開始、1800 秒、サイクル時間 0.6 倍、不良率 10%、NG 停止 60 秒、投入ロックあり、荷役 2 秒、オーバーヘッド 2 秒である。

### 2. 自分の API キーで Jev を呼ぶ（手順の記載のみ。公開用に整理した後のコードでは未実行）

```
pip install "typesafe-sdk>=0.7.2"
export TYPESAFE_API_KEY=...        # 自分のキー。ファイルやコードに書かない
python scripts/run_b1x_episode.py real --cap 117 --out out/my_run
```

`--cap` は API 試行回数（再試行を含む）の上限である。キーの有無だけを確認し、値は表示しない。Jev の答えは実行ごとに変わりうるので、完成数が 20 になる保証はない。API を使わずに配線だけ確かめるには `mock` を使う（Jev の代わりの擬似回答であり、結果は Jev のものではない）。

```
python scripts/run_b1x_episode.py mock --cap 150
```

### 3. Gazebo で再生する

ROS 2 Humble と Gazebo Classic 11 が入った環境で、リポジトリの直下から:

```
cd ros_ws
colcon build --packages-select factory_sim
source install/setup.bash
ros2 launch factory_sim b1_episode.launch.py headless:=false out:=out/b1_replay
```

- `decision_node` が記録済みの判断列（`policy:=replay`、既定）を `/dispatch/decision` で返し、`b1_node` がそれに従ってロボットを走らせる。1800 秒（シミュレーション時間）で終了し、`out/b1_replay/` に `decisions.jsonl`、`legs.jsonl`、`tasks.jsonl`、`metrics.json` を書く。
- 移動と車輪の既定値（`nav_vmax:=1.2 nav_wmax:=2.5 nav_drive_dec:=0.8 nav_turn_dec:=4.0 max_wheel_accel:=6.0 max_wheel_torque:=15.0`）は、記録した走行と同じ値である。遅い値にすると移動が計画の時刻に間に合わず、途中で打ち切られることがある。
- GUI を出さないときは `headless:=true`。
- 別の記録を使うときは `trace:=<json のパス>`。
- Jev をつなぐ場合（自分のキーが必要）: `policy:=jev_b1x jev_transport:=sdk`。`jev_transport:=mock`（既定）は擬似回答で、ネットワークを使わない。
- ノードはリポジトリの直下（`factory_b/` のある場所）を自動で探す。見つからないときは環境変数 `B1_PROJECT_ROOT` にそのパスを入れる。

作成者の環境（2026-10-29）では、Gazebo での再生でも判断 133 回、完成数 20 個になった。ただし、ロボットの実際の走行時間は計算モデルの想定とずれることがあり、環境によっては完成数が 20 と一致しない場合がある。ずれは `metrics.json` の `leg_over_plan_*` などに出る。

### テスト

```
python -m pytest
```

API もネットワークも使わない。`ros_ws/src/factory_sim/test/` のテストは `rclpy` を読み込まないので、ROS 2 がなくても動く（ノード本体の `b1_node.py` と `decision_node.py` は `rclpy` が要り、テストの対象外）。

## 結果

条件は seed 5、在庫なしから開始、1800 秒、1 回の走行である。出典は `recorded/b1x_seed5_empty.json`（記録は計算モデルの走行で、Jev に実際に問い合わせた結果）。

| 項目 | 値 |
|---|---|
| 完成数（出荷数） | 20 個 |
| 判断回数 | 133 回 |
| うち Jev に聞いた回数 | 116 回（供給 110、NG 回収 6）。API 試行 116 回、失敗 0 回 |
| うちコードが出荷を決めた回数 | 10 回（完成品置き場が満杯） |
| うち候補が 1 つで自動で決まった回数 | 7 回（出荷を外した結果 1 候補が 3 回、エンジン側で 1 候補が 4 回） |
| 不良の個数 | 7 個 |
| 使われたモデル | jev-1.14.0 |

出荷はすべてコードの規則が決めており、Jev は出荷のタイミングを選んでいない。

## 注意

- デモ用である。seed 5 の 1 回の走行の結果であり、性能評価ではない。提出した動画とスライドでは、同じ条件の固定巡回（M1→M4 の順に供給を繰り返す方式。完成数 17 個）と並べて示したが、これも 1 回の比較であり、統計的な比較ではない。
- 問いと事実の項目、出荷の規則、各しきい値は、事前に別の場面集合で調整したものである。この走行の結果は、独立な検証にはならない。
- 他の方式との比較に使ったコード（比較用の方針、評価・掃引のスクリプト）は、このリポジトリに含めていない。`policies/jev_b1.py` の `fallback_choice` は、Jev の呼び出しが失敗または時間切れのときの代替であり、比較対象ではない。
- 世界ファイル `factory_b1.world` は生成物で、生成スクリプトは含めていない。
- Jev の答えは同じ入力でも変わりうるので、API を使った再実行で同じ判断列になるとは限らない。

## 参考文献・リンク

- TypeSafe のドキュメント: https://docs.typesafe.ai/
- typesafe-sdk（PyPI）: https://pypi.org/project/typesafe-sdk/
- ROS 2 Humble のドキュメント: https://docs.ros.org/en/humble/
- Gazebo Classic: https://classic.gazebosim.org/

## ライセンス

MIT。`LICENSE` を参照。
