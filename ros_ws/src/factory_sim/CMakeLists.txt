cmake_minimum_required(VERSION 3.8)
project(factory_sim)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Python package (b1_node, decision_node, b1_runner, nav_driver, ...) -> install/.../site-packages/factory_sim
ament_python_install_package(${PROJECT_NAME})

# Gazebo world plugin for machine lamps / cargo box / WIP plates . Installed to lib/factory_sim;
# sim.launch.py adds that folder to GAZEBO_PLUGIN_PATH.
add_library(m5_visual_state SHARED plugin/m5_visual_state.cc)
target_include_directories(m5_visual_state PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(m5_visual_state ${GAZEBO_LIBRARIES})
ament_target_dependencies(m5_visual_state gazebo_ros rclcpp std_msgs)
install(TARGETS m5_visual_state LIBRARY DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf worlds launch media
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/safety_stop.py scripts/b1_node.py scripts/decision_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
