cmake_minimum_required(VERSION 3.8)
project(canopen_410_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_library(node_canopen_cia410_driver
  src/node_interfaces/node_canopen_410_driver.cpp
)
target_compile_features(node_canopen_cia410_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(node_canopen_cia410_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_cia410_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(node_canopen_cia410_driver PUBLIC
  ${canopen_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  canopen_proxy_driver::node_canopen_proxy_driver
  tf2::tf2
)

add_library(lifecycle_cia410_driver
  src/lifecycle_cia410_driver.cpp
)
target_compile_features(lifecycle_cia410_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(lifecycle_cia410_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_cia410_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_cia410_driver PUBLIC
  canopen_core::node_canopen_driver
  node_canopen_cia410_driver
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_compile_definitions(lifecycle_cia410_driver PRIVATE "CANOPEN_410_DRIVER_BUILDING_LIBRARY")
rclcpp_components_register_nodes(lifecycle_cia410_driver "ros2_canopen::LifecycleCia410Driver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleCia410Driver;$<TARGET_FILE:lifecycle_cia410_driver>\n")

add_library(cia410_driver
  src/cia410_driver.cpp
)
target_compile_features(cia410_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(cia410_driver PUBLIC -Wl,--no-undefined)
target_include_directories(cia410_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cia410_driver PUBLIC
  canopen_core::node_canopen_driver
  node_canopen_cia410_driver
  rclcpp_components::component
)
target_compile_definitions(cia410_driver PRIVATE "CANOPEN_410_DRIVER_BUILDING_LIBRARY")
rclcpp_components_register_nodes(cia410_driver "ros2_canopen::Cia410Driver")
set(node_plugins "${node_plugins}ros2_canopen::Cia410Driver;$<TARGET_FILE:cia410_driver>\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS lifecycle_cia410_driver cia410_driver node_canopen_cia410_driver
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
  lifecycle_cia410_driver
  cia410_driver
  node_canopen_cia410_driver
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  canopen_base_driver
  canopen_core
  canopen_interfaces
  canopen_proxy_driver
  geometry_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  tf2
)

ament_package()
