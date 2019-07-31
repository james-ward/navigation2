cmake_minimum_required(VERSION 3.5)
project(nav2_world_model)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util)
find_package(nav2_msgs)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name world_model_core)

add_library(${library_name} SHARED
  src/world_model.cpp
)

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

set(dependencies
  rclcpp
  nav2_util
  nav2_msgs
  nav2_costmap_2d
  tf2_ros
)

target_link_libraries(${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(world_model_core)
ament_export_dependencies(${dependencies})

ament_package()
