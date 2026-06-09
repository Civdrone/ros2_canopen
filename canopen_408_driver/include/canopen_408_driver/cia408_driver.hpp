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

  bool init_axis() { return impl_->init_axis(); }
  bool halt_axis() { return impl_->halt_axis(); }
  bool recover_axis() { return impl_->recover_axis(); }
  bool shutdown_axis() { return impl_->shutdown_axis(); }
  bool set_target(double t) { return impl_->set_target(t); }
  double get_actual() const { return impl_->get_actual(); }
};

}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__CIA408_DRIVER_HPP_
