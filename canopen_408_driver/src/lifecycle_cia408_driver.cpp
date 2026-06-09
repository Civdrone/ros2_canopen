#include "canopen_408_driver/lifecycle_cia408_driver.hpp"

using namespace ros2_canopen;

LifecycleCia408Driver::LifecycleCia408Driver(rclcpp::NodeOptions node_options)
: LifecycleCanopenDriver(node_options)
{
  impl_ =
    std::make_shared<node_interfaces::NodeCanopen408Driver<rclcpp_lifecycle::LifecycleNode>>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(impl_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleCia408Driver)
