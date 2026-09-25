#include "canopen_401_driver/lifecycle_cia401_driver.hpp"

using namespace ros2_canopen;

LifecycleCia401Driver::LifecycleCia401Driver(rclcpp::NodeOptions node_options)
: LifecycleCanopenDriver(node_options)
{
  impl_ =
    std::make_shared<node_interfaces::NodeCanopen401Driver<rclcpp_lifecycle::LifecycleNode>>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(impl_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleCia401Driver)
