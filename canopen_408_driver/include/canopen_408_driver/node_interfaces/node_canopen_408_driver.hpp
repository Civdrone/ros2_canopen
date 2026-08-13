#ifndef CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_HPP_
#define CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_HPP_

#include <memory>
#include <string>
#include <type_traits>

#include "canopen_408_driver/hydraulic_axis.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
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

  // Telemetry publishers.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr spool_position_publisher_;  // percent
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr demand_publisher_;          // percent
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr statusword_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr temperature_publisher_;      // raw
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_publisher_;            // decoded EMCY

  // Command services.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr float_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr set_position_service_;

  // Streaming setpoint input: publish percent here to command the spool at high
  // rate (lighter than the service). Applied immediately via the RPDO path.
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_subscription_;

  // Scale: setpoint_raw = (position_percent - offset) * scale_to_dev
  //        position_percent = actual_raw * scale_from_dev + offset
  double scale_to_dev_;
  double scale_from_dev_;
  double offset_;
  int16_t float_setpoint_raw_;

  // Output topic names (overridable per node in bus.yml).
  std::string spool_position_topic_;
  std::string demand_topic_;
  std::string status_word_topic_;
  std::string pcb_temperature_topic_;
  std::string fault_topic_;
  std::string target_topic_;   // streaming setpoint input (percent)

  void configure_common();
  void publish();
  void poll_timer_callback() override;
  void on_emcy(ros2_canopen::COEmcy emcy) override;  // decode + publish faults

public:
  explicit NodeCanopen408Driver(NODETYPE * node);

  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void add_to_master() override;

  bool enable_axis();
  bool disable_axis();
  bool hold_axis();
  bool recover_axis();
  bool float_axis();
  bool save_axis();
  bool set_position(double position_percent);
  double get_spool_position() const;
};

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_HPP_
