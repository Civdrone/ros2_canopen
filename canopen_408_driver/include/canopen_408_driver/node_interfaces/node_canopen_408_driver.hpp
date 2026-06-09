#ifndef CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_HPP_
#define CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_HPP_

#include <memory>
#include <string>
#include <type_traits>

#include "canopen_408_driver/hydraulic_axis.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
class NodeCanopen408Driver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  std::shared_ptr<HydraulicAxis408> axis_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actual_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr halt_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_service_;
  rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr target_service_;

  // Scale: target_raw = (engineering_target - offset) * scale_to_dev
  //        engineering_actual = actual_raw * scale_from_dev + offset
  double scale_to_dev_;
  double scale_from_dev_;
  double offset_;
  uint8_t default_mode_;
  int init_timeout_ms_;

  void configure_common();
  void publish();
  void poll_timer_callback() override;

public:
  explicit NodeCanopen408Driver(NODETYPE * node);

  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void add_to_master() override;

  bool init_axis();
  bool halt_axis();
  bool recover_axis();
  bool shutdown_axis();
  bool set_target(double engineering_target);
  double get_actual() const;
};

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_HPP_
