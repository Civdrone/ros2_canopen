#include "canopen_408_driver/node_interfaces/node_canopen_408_driver.hpp"
#include "canopen_408_driver/node_interfaces/node_canopen_408_driver_impl.hpp"

template class ros2_canopen::node_interfaces::NodeCanopen408Driver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopen408Driver<rclcpp_lifecycle::LifecycleNode>;
