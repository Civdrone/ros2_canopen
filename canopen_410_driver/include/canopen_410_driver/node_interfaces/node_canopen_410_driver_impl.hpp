#ifndef CANOPEN_410_DRIVER__NODE_INTERFACES__NODE_CANOPEN_410_DRIVER_IMPL_HPP_
#define CANOPEN_410_DRIVER__NODE_INTERFACES__NODE_CANOPEN_410_DRIVER_IMPL_HPP_

#include <optional>

#include "canopen_410_driver/node_interfaces/node_canopen_410_driver.hpp"
#include "canopen_core/driver_error.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

template <class NODETYPE>
NodeCanopen410Driver<NODETYPE>::NodeCanopen410Driver(NODETYPE * node)
: NodeCanopenProxyDriver<NODETYPE>(node),
  frame_id_("inclinometer"),
  deg_per_lsb_fallback_(0.01),
  has_lateral_axis_(true),
  orientation_stddev_(-1.0)
{
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::init(bool /*called_from_base*/)
{
  RCLCPP_ERROR(this->node_->get_logger(), "NodeCanopen410Driver::init() base template called");
}

template <>
inline void NodeCanopen410Driver<rclcpp::Node>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  imu_publisher_ = this->node_->template create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
}

template <>
inline void NodeCanopen410Driver<rclcpp_lifecycle::LifecycleNode>::init(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  imu_publisher_ =
    this->node_->template create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::configure_common()
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
    deg_per_lsb_fallback_ = this->config_["deg_per_lsb"].template as<double>();
  }
  catch (...)
  {
  }
  try
  {
    has_lateral_axis_ = this->config_["has_lateral_axis"].template as<bool>();
  }
  catch (...)
  {
  }
  try
  {
    orientation_stddev_ = this->config_["orientation_stddev"].template as<double>();
  }
  catch (...)
  {
  }

  // register_map: block — override any subset of OD indices / widths.
  // Defaults match the CiA 410 standard. Devices like Posital DST X720 use
  // a different layout (lateral axis at 0x6020, 16-bit slopes, resolution
  // at 0x6000) and override here.
  try
  {
    auto rm = this->config_["register_map"];
    if (rm.IsDefined() && rm.IsMap())
    {
      auto pick_u16 = [&rm](const char * key, uint16_t & out) {
        try
        {
          out = static_cast<uint16_t>(rm[key].template as<int>());
        }
        catch (...)
        {
        }
      };
      auto pick_u8 = [&rm](const char * key, uint8_t & out) {
        try
        {
          out = static_cast<uint8_t>(rm[key].template as<int>());
        }
        catch (...)
        {
        }
      };
      pick_u16("slope_long_index", register_map_.slope_long_index);
      pick_u8("slope_long_bits", register_map_.slope_long_bits);
      pick_u16("slope_lateral_index", register_map_.slope_lateral_index);
      pick_u8("slope_lateral_bits", register_map_.slope_lateral_bits);
      pick_u16("resolution_index", register_map_.resolution_index);
      pick_u16("slope_long_preset_index", register_map_.slope_long_preset_index);
      pick_u16("slope_lateral_preset_index", register_map_.slope_lateral_preset_index);
      pick_u16("slope_long_offset_index", register_map_.slope_long_offset_index);
      pick_u16("slope_lateral_offset_index", register_map_.slope_lateral_offset_index);
      pick_u8("preset_bits", register_map_.preset_bits);
      pick_u8("offset_bits", register_map_.offset_bits);
    }
  }
  catch (...)
  {
  }

  zero_long_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()) + "/zero_long",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) { response->success = this->zero_long(); });

  if (has_lateral_axis_)
  {
    zero_lateral_service_ = this->node_->template create_service<std_srvs::srv::Trigger>(
      std::string(this->node_->get_name()) + "/zero_lateral",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      { response->success = this->zero_lateral(); });
  }

  RCLCPP_INFO(
    this->node_->get_logger(),
    "CiA 410 driver configured. frame_id=%s, deg_per_lsb_fallback=%f, has_lateral=%s, "
    "slope_long=0x%04X/%u-bit, slope_lateral=0x%04X/%u-bit, resolution=0x%04X",
    frame_id_.c_str(), deg_per_lsb_fallback_, has_lateral_axis_ ? "yes" : "no",
    register_map_.slope_long_index, register_map_.slope_long_bits,
    register_map_.slope_lateral_index, register_map_.slope_lateral_bits,
    register_map_.resolution_index);
}

template <>
inline void NodeCanopen410Driver<rclcpp::Node>::configure(bool /*called_from_base*/)
{
  this->configure_common();
}

template <>
inline void NodeCanopen410Driver<rclcpp_lifecycle::LifecycleNode>::configure(bool /*called_from_base*/)
{
  this->configure_common();
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::activate(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  if (inclinometer_)
  {
    inclinometer_->refresh_resolution();
    RCLCPP_INFO(
      this->node_->get_logger(),
      "Inclinometer resolution: %f deg/LSB", inclinometer_->deg_per_lsb());
  }
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::deactivate(bool /*called_from_base*/)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  this->exec_->post([this]() { inclinometer_.reset(); });
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  inclinometer_ =
    std::make_shared<Inclinometer410>(this->lely_driver_, register_map_, deg_per_lsb_fallback_);
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  publish();
}

template <class NODETYPE>
void NodeCanopen410Driver<NODETYPE>::publish()
{
  if (!inclinometer_) return;

  // CiA 410 convention: "long" slope is around the longitudinal axis (typically pitch),
  // "lateral" is around the lateral axis (typically roll). Yaw is not measured.
  const double pitch = inclinometer_->get_long_rad();
  const double roll = has_lateral_axis_ ? inclinometer_->get_lateral_rad() : 0.0;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, 0.0);

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = this->node_->now();
  msg.header.frame_id = frame_id_;
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();

  if (orientation_stddev_ > 0.0)
  {
    const double var = orientation_stddev_ * orientation_stddev_;
    msg.orientation_covariance[0] = var;
    msg.orientation_covariance[4] = var;
    msg.orientation_covariance[8] = var;
  }
  else
  {
    // -1 on element 0 signals "covariance unknown" per sensor_msgs/Imu convention.
    msg.orientation_covariance[0] = -1.0;
  }
  msg.angular_velocity_covariance[0] = -1.0;
  msg.linear_acceleration_covariance[0] = -1.0;

  imu_publisher_->publish(msg);
}

#endif  // CANOPEN_410_DRIVER__NODE_INTERFACES__NODE_CANOPEN_410_DRIVER_IMPL_HPP_
