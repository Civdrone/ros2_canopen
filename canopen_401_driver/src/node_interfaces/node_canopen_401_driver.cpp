#include "canopen_401_driver/node_interfaces/node_canopen_401_driver.hpp"
#include "canopen_401_driver/node_interfaces/node_canopen_401_driver_impl.hpp"

template class ros2_canopen::node_interfaces::NodeCanopen401Driver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopen401Driver<rclcpp_lifecycle::LifecycleNode>;
