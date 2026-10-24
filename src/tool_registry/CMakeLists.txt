cmake_minimum_required(VERSION 3.8)
project(tool_registry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(spacedrone_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(tool_registry_node src/tool_registry_node.cpp)
ament_target_dependencies(tool_registry_node
  rclcpp
  geometry_msgs
  std_srvs
  tf2
  spacedrone_interfaces
)
target_link_libraries(tool_registry_node yaml-cpp)
# target_link_libraries(tool_registry_node ${YAML_CPP_LIBRARIES})  # empty on 24.04 for some reason, linker errors

install(TARGETS tool_registry_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
