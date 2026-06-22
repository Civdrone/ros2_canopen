#ifndef CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_IMPL_HPP_
#define CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_IMPL_HPP_

#include <chrono>
#include <cmath>
#include <optional>

#include "canopen_408_driver/node_interfaces/node_canopen_408_driver.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template <class NODETYPE>
NodeCanopen408Driver<NODETYPE>::NodeCanopen408Driver(NODETYPE * node)
: NodeCanopenProxyDriver<NODETYPE>(node),
  scale_to_dev_(1000.0),
  scale_from_dev_(0.001),
  offset_(0.0),
  default_mode_(static_cast<uint8_t>(Cia408Mode::Position)),
  init_timeout_ms_(5000)
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
  actual_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::Float64>("~/actual", 10);
}

template <>
inline void NodeCanopen408Driver<rclcpp_lifecycle::LifecycleNode>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  actual_publisher_ =
    this->node_->template create_publisher<std_msgs::msg::Float64>("~/actual", 10);
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
    default_mode_ = this->config_["default_mode"].template as<uint8_t>();
  }
  catch (...)
  {
  }
  try
  {
    init_timeout_ms_ = this->config_["init_timeout_ms"].template as<int>();
  }
  catch (...)
  {
  }

  const std::string prefix = std::string(this->node_->get_name()) + "/";

  init_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "init",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) { response->success = this->init_axis(); });

  halt_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "halt",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) { response->success = this->halt_axis(); });

  recover_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "recover",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    { response->success = this->recover_axis(); });

  shutdown_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    prefix + "shutdown",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    { response->success = this->shutdown_axis(); });

  target_service_ = this->node_->template create_service<canopen_interfaces::srv::COTargetDouble>(
    prefix + "target",
    [this](
      const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr req,
      canopen_interfaces::srv::COTargetDouble::Response::SharedPtr resp)
    { resp->success = this->set_target(req->target); });

  RCLCPP_INFO(
    this->node_->get_logger(),
    "CiA 408 driver configured. mode=%u, scale_to=%f, scale_from=%f, offset=%f",
    default_mode_, scale_to_dev_, scale_from_dev_, offset_);
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
  if (axis_)
  {
    axis_->set_mode(static_cast<Cia408Mode>(default_mode_));
  }
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::deactivate(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  this->exec_->post([this]() { axis_.reset(); });
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  axis_ = std::make_shared<HydraulicAxis408>(
    this->lely_driver_, static_cast<Cia408Mode>(default_mode_));
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  publish();
}

template <class NODETYPE>
void NodeCanopen408Driver<NODETYPE>::publish()
{
  if (!axis_) return;
  std_msgs::msg::Float64 msg;
  msg.data = get_actual();
  actual_publisher_->publish(msg);
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::init_axis()
{
  if (!axis_) return false;
  return axis_->init(std::chrono::milliseconds(init_timeout_ms_));
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::halt_axis()
{
  return axis_ ? axis_->halt() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::recover_axis()
{
  return axis_ ? axis_->recover() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::shutdown_axis()
{
  return axis_ ? axis_->shutdown() : false;
}

template <class NODETYPE>
bool NodeCanopen408Driver<NODETYPE>::set_target(double engineering_target)
{
  if (!axis_) return false;
  const int32_t raw =
    static_cast<int32_t>(std::lround((engineering_target - offset_) * scale_to_dev_));
  return axis_->set_target(raw);
}

template <class NODETYPE>
double NodeCanopen408Driver<NODETYPE>::get_actual() const
{
  if (!axis_) return 0.0;
  return static_cast<double>(axis_->get_actual()) * scale_from_dev_ + offset_;
}

#endif  // CANOPEN_408_DRIVER__NODE_INTERFACES__NODE_CANOPEN_408_DRIVER_IMPL_HPP_
