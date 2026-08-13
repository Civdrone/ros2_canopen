#ifndef CANOPEN_408_DRIVER__CIA408_DRIVER_HPP_
#define CANOPEN_408_DRIVER__CIA408_DRIVER_HPP_

#include "canopen_408_driver/node_interfaces/node_canopen_408_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{

class Cia408Driver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopen408Driver<rclcpp::Node>> impl_;

public:
  explicit Cia408Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  bool enable_axis() { return impl_->enable_axis(); }
  bool disable_axis() { return impl_->disable_axis(); }
  bool hold_axis() { return impl_->hold_axis(); }
  bool recover_axis() { return impl_->recover_axis(); }
  bool float_axis() { return impl_->float_axis(); }
  bool save_axis() { return impl_->save_axis(); }
  bool set_position(double percent) { return impl_->set_position(percent); }
  double get_spool_position() const { return impl_->get_spool_position(); }
};

}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__CIA408_DRIVER_HPP_
