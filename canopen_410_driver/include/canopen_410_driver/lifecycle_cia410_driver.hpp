#ifndef CANOPEN_410_DRIVER__LIFECYCLE_CIA410_DRIVER_HPP_
#define CANOPEN_410_DRIVER__LIFECYCLE_CIA410_DRIVER_HPP_

#include "canopen_410_driver/node_interfaces/node_canopen_410_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{

class LifecycleCia410Driver : public ros2_canopen::LifecycleCanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopen410Driver<rclcpp_lifecycle::LifecycleNode>> impl_;

public:
  explicit LifecycleCia410Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  double get_long_rad() const { return impl_->get_long_rad(); }
  double get_lateral_rad() const { return impl_->get_lateral_rad(); }
  bool zero_long() { return impl_->zero_long(); }
  bool zero_lateral() { return impl_->zero_lateral(); }
};

}  // namespace ros2_canopen

#endif  // CANOPEN_410_DRIVER__LIFECYCLE_CIA410_DRIVER_HPP_
