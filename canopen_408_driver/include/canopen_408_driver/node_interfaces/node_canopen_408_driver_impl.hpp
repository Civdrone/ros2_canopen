#ifndef CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_IMPL_HPP_
#define CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_IMPL_HPP_

#include <chrono>
#include <cmath>
#include <cstdio>

#include "canopen_408_driver/node_interfaces/node_canopen_408_driver.hpp"
#include "canopen_408_driver/pved_faults.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template <class NODETYPE>
NodeCanopen408Driver<NODETYPE>::NodeCanopen408Driver(NODETYPE * node)
: NodeCanopenProxyDriver<NODETYPE>(node),
  scale_to_dev_(163.84),               // percent -> raw (16384 / 100)
  scale_from_dev_(1.0 / 163.84),       // raw -> percent
  offset_(0.0),
  float_setpoint_raw_(ros2_canopen::SETPOINT_FLOAT),
  spool_position_topic_("~/spool_position"),
  demand_topic_("~/demand"),
  status_word_topic_("~/status_word"),
  pcb_temperature_topic_("~/pcb_temperature"),
  fault_topic_("~/fault"),
  target_topic_("~/target")
{
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::init(bool /*called_from_base*/)
{
  RCLCPP_ERROR(this->node_->get_logger(), "NodeCanopen408Driver::init() base template called");
}

template <>
inline void NodeCanopen408Driver<rclcpp::Node>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  // Publishers are created in configure_common() once YAML topic-name overrides
  // are known.
}

template <>
inline void NodeCanopen408Driver<rclcpp_lifecycle::LifecycleNode>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  // Publishers are created in configure_common() once YAML topic-name overrides
  // are known.
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::configure_common()
{
  NodeCanopenProxyDriver<NODETYPE>::configure(false);

  try
  {
    scale_to_dev_ = this->config_["scale_target_to_dev"].template as<double>();
  }
  catch (...)
  {
  }
  try
  {
    scale_from_dev_ = this->config_["scale_actual_from_dev"].template as<double>();
  }
  catch (...)
  {
  }
  try
  {
    offset_ = this->config_["offset"].template as<double>();
  }
  catch (...)
  {
  }
  try
  {
    float_setpoint_raw_ = static_cast<int16_t>(this->config_["float_setpoint_raw"].template as<int>());
  }
  catch (...)
  {
  }

  // Topic-name overrides from bus.yml (default to ~/...).
  try { spool_position_topic_ = this->config_["spool_position_topic"].template as<std::string>(); }
  catch (...) {}
  try { demand_topic_ = this->config_["demand_topic"].template as<std::string>(); }
  catch (...) {}
  try { status_word_topic_ = this->config_["status_word_topic"].template as<std::string>(); }
  catch (...) {}
  try { pcb_temperature_topic_ = this->config_["pcb_temperature_topic"].template as<std::string>(); }
  catch (...) {}
  try { fault_topic_ = this->config_["fault_topic"].template as<std::string>(); }
  catch (...) {}
  try { target_topic_ = this->config_["target_topic"].template as<std::string>(); }
  catch (...) {}

  spool_position_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::Float64>(spool_position_topic_, 10);
  demand_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::Float64>(demand_topic_, 10);
  statusword_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::UInt16>(status_word_topic_, 10);
  temperature_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::UInt16>(pcb_temperature_topic_, 10);
  fault_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::String>(fault_topic_, 10);

  const std::string prefix = std::string(this->node_->get_name()) + "/";

  enable_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "enable",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    { response->success = this->enable_axis(); });

  disable_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "disable",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    { response->success = this->disable_axis(); });

  hold_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "hold",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) { response->success = this->hold_axis(); });

  recover_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "recover",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    { response->success = this->recover_axis(); });

  float_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "float",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    { response->success = this->float_axis(); });

  save_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "save",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) { response->success = this->save_axis(); });

  set_position_service_ =
    this->node_->template create_service<canopen_interfaces::srv::COTargetDouble>(
      prefix + "set_position",
      [this](
        const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr req,
        canopen_interfaces::srv::COTargetDouble::Response::SharedPtr resp)
      { resp->success = this->set_position(req->target); });

  // Streaming setpoint: percent on ~/target -> spool, applied immediately.
  // KeepLast(1) so a backlog can't queue stale setpoints if the executor stalls;
  // only the newest command is ever applied.
  target_subscription_ = this->node_->template create_subscription<std_msgs::msg::Float64>(
    target_topic_, rclcpp::QoS(1),
    [this](const std_msgs::msg::Float64::SharedPtr msg) { this->set_position(msg->data); });

  RCLCPP_INFO(
    this->node_->get_logger(),
    "PVED-CC (CiA 408) driver configured. scale_to=%f, scale_from=%f, offset=%f, "
    "spool_position_topic=%s, fault_topic=%s",
    scale_to_dev_, scale_from_dev_, offset_, spool_position_topic_.c_str(), fault_topic_.c_str());
}

template <>
inline void NodeCanopen408Driver<rclcpp::Node>::configure(bool /*called_from_base*/)
{
  this->configure_common();
}

template <>
inline void NodeCanopen408Driver<rclcpp_lifecycle::LifecycleNode>::configure(bool /*called_from_base*/)
{
  this->configure_common();
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::activate(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  // Deliberately NOT auto-enabling: this is a hydraulic valve. Enable explicitly
  // via the ~/enable service once it is safe to move the spool.
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::deactivate(bool /*called_from_base*/)
{
  if (axis_) axis_->disable();
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  this->exec_->post([this]() { axis_.reset(); });
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  axis_ = std::make_shared<HydraulicAxis408>(this->lely_driver_);
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  // Advance the (non-blocking) DSM climb and stream control word + set point.
  if (axis_) axis_->spin_once();
  publish();
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::publish()
{
  if (!axis_) return;

  // spool position + status word come from TPDO1 (cached reads, no bus traffic).
  std_msgs::msg::Float64 spool;
  spool.data = get_spool_position();
  spool_position_publisher_->publish(spool);

  std_msgs::msg::UInt16 sw;
  sw.data = axis_->get_statusword();
  statusword_publisher_->publish(sw);

  // NOTE: demand (0x6310) and PCB temperature (0x3468) lived on TPDO2, which this
  // PVED firmware rejects (SDO abort 0x06040047). They are intentionally not
  // published here -- reading them would force a blocking SDO every cycle. If you
  // need them, add a decimated SDO poll (e.g. every ~1s) rather than per-cycle.
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::on_emcy(ros2_canopen::COEmcy emcy)
{
  // Preserve base-class behavior (diagnostics aggregation).
  NodeCanopenProxyDriver<NODETYPE>::on_emcy(emcy);

  // Danfoss EMCY layout: eec = fault code, er = error register,
  // msef[0] = occurrence counter, msef[1] = fault ID, msef[4] = severity.
  const char * name = ros2_canopen::pved_fault_name(emcy.eec);
  char buf[256];
  std::snprintf(
    buf, sizeof(buf), "EMCY 0x%04X err_reg=0x%02X severity=%s fault_id=%u occ=%u : %s", emcy.eec,
    emcy.er, ros2_canopen::pved_severity_name(emcy.msef[4]), static_cast<unsigned>(emcy.msef[1]),
    static_cast<unsigned>(emcy.msef[0]), name ? name : "undocumented fault");

  std_msgs::msg::String msg;
  msg.data = buf;
  if (fault_publisher_) fault_publisher_->publish(msg);

  if (emcy.eec == 0x0000)
  {
    RCLCPP_INFO(this->node_->get_logger(), "fault cleared (%s)", buf);
  }
  else
  {
    RCLCPP_ERROR(this->node_->get_logger(), "%s", buf);
  }
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::enable_axis()
{
  return axis_ ? axis_->enable() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::disable_axis()
{
  return axis_ ? axis_->disable() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::hold_axis()
{
  return axis_ ? axis_->hold() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::recover_axis()
{
  return axis_ ? axis_->recover() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::float_axis()
{
  return axis_ ? axis_->set_setpoint(float_setpoint_raw_) : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::save_axis()
{
  return axis_ ? axis_->save_to_eeprom() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::set_position(double position_percent)
{
  if (!axis_) return false;
  const int32_t raw = std::lround((position_percent - offset_) * scale_to_dev_);
  return axis_->set_setpoint(static_cast<int16_t>(raw));
}

template <class NODETYPE>
double NodeCanopen408Driver<NODETYPE>::get_spool_position() const
{
  if (!axis_) return 0.0;
  return static_cast<double>(axis_->get_spool_position()) * scale_from_dev_ + offset_;
}

#endif  // CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_IMPL_HPP_
