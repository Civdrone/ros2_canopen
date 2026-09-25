#ifndef CANOPEN_401_DRIVER__NODE_INTERFACES__NODE_CANOPEN_401_DRIVER_HPP_
#define CANOPEN_401_DRIVER__NODE_INTERFACES__NODE_CANOPEN_401_DRIVER_HPP_

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "canopen_401_driver/m880_remote.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
class NodeCanopen401Driver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  std::shared_ptr<M880Remote> remote_;

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr digital_output_subscription_;

  std::string frame_id_;
  std::string joy_topic_;
  std::string estop_topic_;
  std::string digital_output_topic_;

  // OD index map (defaults to the IMET M880 layout; overridable in YAML).
  Cia401RegisterMap register_map_;

  std::vector<AxisSpec> axes_;
  std::vector<BitSpec> buttons_;

  // Digital-input sub-indices safe to poll each cycle. Defaults to the stock
  // M880 TPDO layout: TPDO2 carries 0x6000:01..07 plus 0x6000:11, TPDO4
  // carries 0x6000:08..0F. Anything outside this set falls back to a blocking
  // SDO read on every poll, so it must be opted into deliberately.
  std::vector<uint8_t> polled_digital_subs_;

  // Emergency stop bit. Not safety-rated over this path -- see the header
  // comment in the impl and the topic name.
  BitSpec estop_spec_;
  bool estop_configured_;

  void configure_common();
  void publish();
  void poll_timer_callback() override;

  void on_digital_output(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

  // Parses an `{index, sub, bit, active_low}` YAML node into a BitSpec.
  // Returns false and leaves `out` untouched if the node is absent/malformed.
  bool parse_bit_spec(const YAML::Node & node, BitSpec & out) const;

public:
  explicit NodeCanopen401Driver(NODETYPE * node);

  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void add_to_master() override;

  std::vector<float> get_axes() const
  {
    return remote_ ? remote_->read_axes() : std::vector<float>();
  }
  std::vector<int32_t> get_buttons() const
  {
    return remote_ ? remote_->read_buttons() : std::vector<int32_t>();
  }
  bool get_estop() const
  {
    return (remote_ && estop_configured_) ? remote_->read_bit(estop_spec_) : false;
  }
  bool set_digital_output_byte(uint8_t sub, uint8_t value)
  {
    return remote_ && remote_->write_output_byte(sub, value);
  }
};

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // CANOPEN_401_DRIVER__NODE_INTERFACES__NODE_CANOPEN_401_DRIVER_HPP_
