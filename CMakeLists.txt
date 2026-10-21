cmake_minimum_required(VERSION 3.8)
project(aif_debug_visualizer)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(socketcan_interface_msg REQUIRED)
find_package(steered_drive_msg REQUIRED)
find_package(vectornav_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/decode_config.cpp
  src/vehicle_state.cpp
  src/manual_dashboard_overlay.cpp
  src/manual_dashboard_panel.cpp
  include/aif_debug_visualizer/manual_dashboard_overlay.hpp
  include/aif_debug_visualizer/manual_dashboard_panel.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp rviz_common rviz_rendering ament_index_cpp
  std_msgs geometry_msgs sensor_msgs socketcan_interface_msg steered_drive_msg vectornav_msgs)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets yaml-cpp)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY config launch assets DESTINATION share/${PROJECT_NAME})


ament_package()
