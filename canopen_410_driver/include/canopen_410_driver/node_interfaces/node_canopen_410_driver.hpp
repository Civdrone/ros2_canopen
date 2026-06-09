#ifndef CANOPEN_410_DRIVER__NODE_INTERFACES__NODE_CANOPEN_410_DRIVER_HPP_
#define CANOPEN_410_DRIVER__NODE_INTERFACES__NODE_CANOPEN_410_DRIVER_HPP_

#include <memory>
#include <string>
#include <type_traits>

#include "canopen_410_driver/inclinometer.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
class NodeCanopen410Driver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  std::shared_ptr<Inclinometer410> inclinometer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_long_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_lateral_service_;

  std::string frame_id_;
  double deg_per_lsb_fallback_;
  bool has_lateral_axis_;
  // Configurable orientation covariance (3x3 row-major). If left as the default
  // -1.0 sentinel on element [0], the published Imu marks orientation as unknown.
  double orientation_stddev_;

  void configure_common();
  void publish();
  void poll_timer_callback() override;

public:
  explicit NodeCanopen410Driver(NODETYPE * node);

  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void add_to_master() override;

  double get_long_rad() const { return inclinometer_ ? inclinometer_->get_long_rad() : 0.0; }
  double get_lateral_rad() const { return inclinometer_ ? inclinometer_->get_lateral_rad() : 0.0; }

  bool zero_long() { return inclinometer_ && inclinometer_->zero_long(); }
  bool zero_lateral() { return inclinometer_ && inclinometer_->zero_lateral(); }
};

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // CANOPEN_410_DRIVER__NODE_INTERFACES__NODE_CANOPEN_410_DRIVER_HPP_
