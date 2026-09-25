cmake_minimum_required(VERSION 3.8)
project(canopen_401_driver)

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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(node_canopen_cia401_driver
  src/node_interfaces/node_canopen_401_driver.cpp
)
target_compile_features(node_canopen_cia401_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(node_canopen_cia401_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_cia401_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(node_canopen_cia401_driver PUBLIC
  ${canopen_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  canopen_proxy_driver::node_canopen_proxy_driver
)

add_library(lifecycle_cia401_driver
  src/lifecycle_cia401_driver.cpp
)
target_compile_features(lifecycle_cia401_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(lifecycle_cia401_driver PUBLIC -Wl,--no-undefined)
target_include_directories(lifecycle_cia401_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_cia401_driver PUBLIC
  canopen_core::node_canopen_driver
  node_canopen_cia401_driver
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_compile_definitions(lifecycle_cia401_driver PRIVATE "CANOPEN_401_DRIVER_BUILDING_LIBRARY")
rclcpp_components_register_nodes(lifecycle_cia401_driver "ros2_canopen::LifecycleCia401Driver")
set(node_plugins "${node_plugins}ros2_canopen::LifecycleCia401Driver;$<TARGET_FILE:lifecycle_cia401_driver>\n")

add_library(cia401_driver
  src/cia401_driver.cpp
)
target_compile_features(cia401_driver PUBLIC c_std_99 cxx_std_17)
target_compile_options(cia401_driver PUBLIC -Wl,--no-undefined)
target_include_directories(cia401_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cia401_driver PUBLIC
  canopen_core::node_canopen_driver
  node_canopen_cia401_driver
  rclcpp_components::component
)
target_compile_definitions(cia401_driver PRIVATE "CANOPEN_401_DRIVER_BUILDING_LIBRARY")
rclcpp_components_register_nodes(cia401_driver "ros2_canopen::Cia401Driver")
set(node_plugins "${node_plugins}ros2_canopen::Cia401Driver;$<TARGET_FILE:cia401_driver>\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS lifecycle_cia401_driver cia401_driver node_canopen_cia401_driver
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
  lifecycle_cia401_driver
  cia401_driver
  node_canopen_cia401_driver
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
  sensor_msgs
  std_msgs
  std_srvs
)

ament_package()
