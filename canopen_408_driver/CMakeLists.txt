cmake_minimum_required(VERSION 3.8)
project(canopen_408_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(hydraulic_axis_bridge
  src/hydraulic_axis.cpp
)
target_compile_features(hydraulic_axis_bridge PUBLIC c_std_99 cxx_std_17)
target_compile_options(hydraulic_axis_bridge PUBLIC -Wl,--no-undefined)
target_include_directories(hydraulic_axis_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hydraulic_axis_bridge PUBLIC
  canopen_base_driver::node_canopen_base_driver
  rclcpp::rclcpp
)

add_library(node_canopen_cia408_driver
  src/node_interfaces/node_canopen_408_driver.cpp
)
target_compile_features(node_canopen_cia408_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(node_canopen_cia408_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_cia408_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(node_canopen_cia408_driver PUBLIC
  ${canopen_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  canopen_proxy_driver::node_canopen_proxy_driver
  hydraulic_axis_bridge
)

add_library(lifecycle_cia408_driver
  src/lifecycle_cia408_driver.cpp
)
target_compile_features(lifecycle_cia408_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(lifecycle_cia408_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_cia408_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_cia408_driver PUBLIC
  canopen_core::node_canopen_driver
  node_canopen_cia408_driver
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_compile_definitions(lifecycle_cia408_driver PRIVATE "CANOPEN_408_DRIVER_BUILDING_LIBRARY")
rclcpp_components_register_nodes(lifecycle_cia408_driver "ros2_canopen::LifecycleCia408Driver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleCia408Driver;$<TARGET_FILE:lifecycle_cia408_driver>\n")

add_library(cia408_driver
  src/cia408_driver.cpp
)
target_compile_features(cia408_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(cia408_driver PUBLIC -Wl,--no-undefined)
target_include_directories(cia408_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cia408_driver PUBLIC
  canopen_core::node_canopen_driver
  node_canopen_cia408_driver
  rclcpp_components::component
)
target_compile_definitions(cia408_driver PRIVATE "CANOPEN_408_DRIVER_BUILDING_LIBRARY")
rclcpp_components_register_nodes(cia408_driver "ros2_canopen::Cia408Driver")
set(node_plugins "${node_plugins}ros2_canopen::Cia408Driver;$<TARGET_FILE:cia408_driver>\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS lifecycle_cia408_driver cia408_driver node_canopen_cia408_driver hydraulic_axis_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(
  lifecycle_cia408_driver
  cia408_driver
  node_canopen_cia408_driver
  hydraulic_axis_bridge
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  canopen_base_driver
  canopen_core
  canopen_interfaces
  canopen_proxy_driver
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  std_srvs
)

ament_package()
