#include "canopen_410_driver/cia410_driver.hpp"

using namespace ros2_canopen;

Cia410Driver::Cia410Driver(rclcpp::NodeOptions node_options) : CanopenDriver(node_options)
{
  impl_ = std::make_shared<node_interfaces::NodeCanopen410Driver<rclcpp::Node>>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(impl_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::Cia410Driver)
