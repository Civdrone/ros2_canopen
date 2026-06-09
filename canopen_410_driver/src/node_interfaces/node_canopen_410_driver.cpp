#include "canopen_410_driver/node_interfaces/node_canopen_410_driver.hpp"
#include "canopen_410_driver/node_interfaces/node_canopen_410_driver_impl.hpp"

template class ros2_canopen::node_interfaces::NodeCanopen410Driver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopen410Driver<rclcpp_lifecycle::LifecycleNode>;
