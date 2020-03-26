cmake_minimum_required(VERSION 3.5)
project(robotx_bt_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robotx_behavior_tree REQUIRED)
find_package(ZeroMQ)

include_directories(${ZMQ_INCLUDE_DIR})
include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

##################################################
######             COMPONENT               #######
##################################################
add_library(bt_planner_component SHARED
        src/bt_planner_component.cpp)

target_compile_definitions(bt_planner_component
        PRIVATE "ROBOTX_BT_PLANNER_BT_PLANNER_COMPONENT_BUILDING_DLL")

ament_target_dependencies(bt_planner_component
        ament_index_cpp
        rclcpp
        rclcpp_components
        behaviortree_cpp_v3
        robotx_behavior_tree
)
target_link_libraries(bt_planner_component ${behaviortree_cpp_v3_LIBRARIES} ${ZMQ_LIBRARIES})

rclcpp_components_register_nodes(bt_planner_component
        "robotx_bt_planner::BTPlannerComponent")

##################################################
######                NODE                 #######
##################################################
add_executable(bt_planner_node
  src/bt_planner_node.cpp
)

ament_target_dependencies(
  bt_planner_node
  rclcpp
  behaviortree_cpp_v3
  robotx_behavior_tree
)

target_link_libraries(bt_planner_node bt_planner_component)

install(TARGETS bt_planner_node
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bt_planner_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)
ament_export_libraries(bt_planner_component)

install(DIRECTORY "include/"
        DESTINATION include
)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
