#ifndef CANOPEN_401_DRIVER__NODE_INTERFACES__NODE_CANOPEN_401_DRIVER_IMPL_HPP_
#define CANOPEN_401_DRIVER__NODE_INTERFACES__NODE_CANOPEN_401_DRIVER_IMPL_HPP_

#include <string>
#include <vector>

#include "canopen_401_driver/node_interfaces/node_canopen_401_driver.hpp"

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

namespace
{
// Digital-input sub-indices carried by the stock IMET M880 TPDO configuration:
// TPDO2 maps 0x6000:01..07 plus 0x6000:11, TPDO4 maps 0x6000:08..0F.
// 0x6000:10 is deliberately absent -- the factory mapping skips it.
const std::vector<uint8_t> kDefaultPolledDigitalSubs = {
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x11};

// Analog channels carried by the stock TPDO configuration: TPDO1 maps
// 0x2021:01..08 (CH1..CH8) and TPDO3 maps 0x2021:09..10 (CH9..CH16).
constexpr uint8_t kDefaultAxisCount = 16;
}  // namespace

template <class NODETYPE>
NodeCanopen401Driver<NODETYPE>::NodeCanopen401Driver(NODETYPE * node)
: NodeCanopenProxyDriver<NODETYPE>(node),
  frame_id_("m880_remote"),
  joy_topic_("~/joy"),
  // The name carries the warning on purpose. The M880 delivers its
  // safety-relevant bits (STOP, Safety EN) over SRDO with an inverted mirror
  // in 0x2120/0x2121 and a frame counter. Neither ros2_canopen nor the Lely
  // build used here implements SRDO, so this topic is derived from ordinary
  // PDO/SDO traffic with none of that cross-checking. Do not use it as the
  // authoritative emergency stop.
  estop_topic_("~/estop_not_safety_rated"),
  digital_output_topic_("~/digital_outputs"),
  estop_configured_(false)
{
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::init(bool /*called_from_base*/)
{
  RCLCPP_ERROR(this->node_->get_logger(), "NodeCanopen401Driver::init() base template called");
}

template <>
inline void NodeCanopen401Driver<rclcpp::Node>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  // Publishers are created in configure_common() once YAML topic-name
  // overrides are known.
}

template <>
inline void NodeCanopen401Driver<rclcpp_lifecycle::LifecycleNode>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  // Publishers are created in configure_common() once YAML topic-name
  // overrides are known.
}

template <class NODETYPE>
bool NodeCanopen401Driver<NODETYPE>::parse_bit_spec(const YAML::Node & node, BitSpec & out) const
{
  if (!node.IsDefined() || !node.IsMap()) return false;

  BitSpec spec = out;
  try
  {
    spec.index = static_cast<uint16_t>(node["index"].as<int>());
  }
  catch (...)
  {
  }
  try
  {
    spec.sub = static_cast<uint8_t>(node["sub"].as<int>());
  }
  catch (...)
  {
  }
  try
  {
    spec.bit = static_cast<uint8_t>(node["bit"].as<int>());
  }
  catch (...)
  {
  }
  try
  {
    spec.active_low = node["active_low"].as<bool>();
  }
  catch (...)
  {
  }
  out = spec;
  return true;
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::configure_common()
{
  NodeCanopenProxyDriver<NODETYPE>::configure(false);

  try
  {
    frame_id_ = this->config_["frame_id"].template as<std::string>();
  }
  catch (...)
  {
  }
  try
  {
    joy_topic_ = this->config_["joy_topic"].template as<std::string>();
  }
  catch (...)
  {
  }
  try
  {
    estop_topic_ = this->config_["estop_topic"].template as<std::string>();
  }
  catch (...)
  {
  }
  try
  {
    digital_output_topic_ = this->config_["digital_output_topic"].template as<std::string>();
  }
  catch (...)
  {
  }

  // register_map: block -- override any subset of the OD indices. Defaults
  // match the IMET M880 (digital I/O at the CiA 401 standard indices, analog
  // and named commands in manufacturer space).
  try
  {
    auto rm = this->config_["register_map"];
    if (rm.IsDefined() && rm.IsMap())
    {
      auto pick_u16 = [&rm](const char * key, uint16_t & out)
      {
        try
        {
          out = static_cast<uint16_t>(rm[key].template as<int>());
        }
        catch (...)
        {
        }
      };
      pick_u16("digital_input_index", register_map_.digital_input_index);
      pick_u16("digital_output_index", register_map_.digital_output_index);
      pick_u16("analog_index", register_map_.analog_index);
      pick_u16("command_index", register_map_.command_index);
    }
  }
  catch (...)
  {
  }

  // polled_digital_subs: which 0x6000 sub-indices to read every cycle.
  // Anything not TPDO-mapped triggers a blocking SDO read per poll, so the
  // default is exactly the stock M880 TPDO set.
  polled_digital_subs_.clear();
  try
  {
    auto subs = this->config_["polled_digital_subs"];
    if (subs.IsDefined() && subs.IsSequence())
    {
      for (const auto & entry : subs)
      {
        polled_digital_subs_.push_back(static_cast<uint8_t>(entry.template as<int>()));
      }
    }
  }
  catch (...)
  {
    polled_digital_subs_.clear();
  }
  if (polled_digital_subs_.empty())
  {
    polled_digital_subs_ = kDefaultPolledDigitalSubs;
  }

  // axes: list of proportional channels. Defaults to CH1..CH16, which is what
  // the stock TPDO1/TPDO3 mapping delivers.
  axes_.clear();
  try
  {
    auto axes = this->config_["axes"];
    if (axes.IsDefined() && axes.IsSequence())
    {
      for (const auto & entry : axes)
      {
        AxisSpec axis;
        try
        {
          axis.sub = static_cast<uint8_t>(entry["channel"].template as<int>());
        }
        catch (...)
        {
          continue;  // a channel number is the one field with no sane default
        }
        try
        {
          axis.center = static_cast<uint8_t>(entry["center"].template as<int>());
        }
        catch (...)
        {
        }
        try
        {
          axis.deadband = static_cast<uint8_t>(entry["deadband"].template as<int>());
        }
        catch (...)
        {
        }
        try
        {
          axis.invert = entry["invert"].template as<bool>();
        }
        catch (...)
        {
        }
        axes_.push_back(axis);
      }
    }
  }
  catch (...)
  {
    axes_.clear();
  }
  if (axes_.empty())
  {
    for (uint8_t channel = 1; channel <= kDefaultAxisCount; ++channel)
    {
      AxisSpec axis;
      axis.sub = channel;
      axes_.push_back(axis);
    }
  }

  // buttons: optional named bit list. Without it the driver expands every
  // polled digital-input byte into 8 buttons, so button index N corresponds to
  // input N+1 of the concatenated polled bytes.
  buttons_.clear();
  try
  {
    auto buttons = this->config_["buttons"];
    if (buttons.IsDefined() && buttons.IsSequence())
    {
      for (const auto & entry : buttons)
      {
        BitSpec spec;
        spec.index = register_map_.digital_input_index;
        if (parse_bit_spec(entry, spec))
        {
          buttons_.push_back(spec);
        }
      }
    }
  }
  catch (...)
  {
    buttons_.clear();
  }

  // estop: which bit carries the STOP command. Defaults to unset -- without an
  // explicit mapping the driver does not guess, and the estop topic stays
  // silent rather than publishing a fabricated "not stopped".
  estop_spec_.index = register_map_.command_index;
  estop_spec_.sub = 0x13;         // 0x2020:13 "STOP Commands safe"
  estop_spec_.active_low = true;  // STOP chains are active-low: 1 == not stopped
  try
  {
    estop_configured_ = parse_bit_spec(this->config_["estop"], estop_spec_);
  }
  catch (...)
  {
    estop_configured_ = false;
  }

  joy_publisher_ = this->node_->template create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);
  if (estop_configured_)
  {
    estop_publisher_ =
      this->node_->template create_publisher<std_msgs::msg::Bool>(estop_topic_, 10);
  }
  digital_output_subscription_ =
    this->node_->template create_subscription<std_msgs::msg::UInt8MultiArray>(
      digital_output_topic_, 10,
      std::bind(&NodeCanopen401Driver<NODETYPE>::on_digital_output, this, _1));

  RCLCPP_INFO(
    this->node_->get_logger(),
    "CiA 401 (IMET M880) driver configured. frame_id=%s, joy_topic=%s, axes=%zu, "
    "polled_digital_subs=%zu, buttons=%s, digital_in=0x%04X, digital_out=0x%04X, "
    "analog=0x%04X, commands=0x%04X",
    frame_id_.c_str(), joy_topic_.c_str(), axes_.size(), polled_digital_subs_.size(),
    buttons_.empty() ? "auto-expanded" : "explicit", register_map_.digital_input_index,
    register_map_.digital_output_index, register_map_.analog_index, register_map_.command_index);

  if (estop_configured_)
  {
    RCLCPP_WARN(
      this->node_->get_logger(),
      "E-stop mapped to 0x%04X:%02X bit %u (active_%s) and published on '%s'. This path is NOT "
      "safety-rated: the M880 delivers STOP over SRDO, which this stack does not implement, so "
      "the inverted mirror and frame counter are not validated. Keep the hardware safety chain "
      "authoritative.",
      estop_spec_.index, estop_spec_.sub, estop_spec_.bit, estop_spec_.active_low ? "low" : "high",
      estop_topic_.c_str());
  }
}

template <>
inline void NodeCanopen401Driver<rclcpp::Node>::configure(bool /*called_from_base*/)
{
  this->configure_common();
}

template <>
inline void NodeCanopen401Driver<rclcpp_lifecycle::LifecycleNode>::configure(
  bool /*called_from_base*/)
{
  this->configure_common();
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::activate(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);

  // Warn once about anything we are about to poll that the device does not
  // actually expose -- those reads would otherwise fail silently every cycle.
  if (this->lely_driver_)
  {
    for (const uint8_t sub : polled_digital_subs_)
    {
      if (!this->lely_driver_->has_object(register_map_.digital_input_index, sub))
      {
        RCLCPP_WARN(
          this->node_->get_logger(), "Polled digital input 0x%04X:%02X is not in the device OD",
          register_map_.digital_input_index, sub);
      }
    }
    for (const auto & axis : axes_)
    {
      if (!this->lely_driver_->has_object(register_map_.analog_index, axis.sub))
      {
        RCLCPP_WARN(
          this->node_->get_logger(), "Axis channel 0x%04X:%02X is not in the device OD",
          register_map_.analog_index, axis.sub);
      }
    }
  }
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::deactivate(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  this->exec_->post([this]() { remote_.reset(); });
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  remote_ = std::make_shared<M880Remote>(
    this->lely_driver_, register_map_, axes_, buttons_, polled_digital_subs_);
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  publish();
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::publish()
{
  if (!remote_) return;

  const auto stamp = this->node_->now();

  sensor_msgs::msg::Joy joy_msg;
  joy_msg.header.stamp = stamp;
  joy_msg.header.frame_id = frame_id_;
  joy_msg.axes = remote_->read_axes();
  joy_msg.buttons = remote_->read_buttons();
  joy_publisher_->publish(joy_msg);

  if (estop_configured_ && estop_publisher_)
  {
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = remote_->read_bit(estop_spec_);
    estop_publisher_->publish(estop_msg);
  }
}

template <class NODETYPE>
void NodeCanopen401Driver<NODETYPE>::on_digital_output(
  const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  if (!remote_ || !msg) return;

  // Element i maps to 0x6200:(i+1), i.e. output channels Dig_(i*8)+1 ..
  // Dig_(i*8)+8. The stock EDS maps 0x6200 into RPDO11/RPDO12, so these are
  // PDO writes. That only holds while the generated config keeps that mapping
  // -- declaring an explicit `rpdo:` block in bus.yml without 0x6200 in it
  // silently demotes every write here to a blocking SDO.
  for (size_t i = 0; i < msg->data.size(); ++i)
  {
    remote_->write_output_byte(static_cast<uint8_t>(i + 1), msg->data[i]);
  }
}

#endif  // CANOPEN_401_DRIVER__NODE_INTERFACES__NODE_CANOPEN_401_DRIVER_IMPL_HPP_
