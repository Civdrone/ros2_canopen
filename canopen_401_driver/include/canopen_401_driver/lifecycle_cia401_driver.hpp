#ifndef CANOPEN_401_DRIVER__LIFECYCLE_CIA401_DRIVER_HPP_
#define CANOPEN_401_DRIVER__LIFECYCLE_CIA401_DRIVER_HPP_

#include <vector>

#include "canopen_401_driver/node_interfaces/node_canopen_401_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{

class LifecycleCia401Driver : public ros2_canopen::LifecycleCanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopen401Driver<rclcpp_lifecycle::LifecycleNode>> impl_;

public:
  explicit LifecycleCia401Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  std::vector<float> get_axes() const { return impl_->get_axes(); }
  std::vector<int32_t> get_buttons() const { return impl_->get_buttons(); }
  bool get_estop() const { return impl_->get_estop(); }
  bool set_digital_output_byte(uint8_t sub, uint8_t value)
  {
    return impl_->set_digital_output_byte(sub, value);
  }
};

}  // namespace ros2_canopen

#endif  // CANOPEN_401_DRIVER__LIFECYCLE_CIA401_DRIVER_HPP_
