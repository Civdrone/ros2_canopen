#ifndef CANOPEN_410_DRIVER__INCLINOMETER_HPP_
#define CANOPEN_410_DRIVER__INCLINOMETER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_canopen
{

// CiA 410 object dictionary indices.
// Spec reference: CiA DSP-410 "CANopen device profile for inclinometer".
// Some devices implement only the 16-bit variants (0x6000 / 0x6100); others
// add the 32-bit variants (0x6010 / 0x6110). Resolution is at 0x6200 and gives
// LSB-per-degree (e.g. 100 = 0.01° per LSB). If a device omits 0x6200, the
// driver falls back to a YAML-configured default (typically 100).
struct Cia410Register
{
  static constexpr uint16_t SLOPE_LONG_16 = 0x6000;
  static constexpr uint16_t SLOPE_LONG_32 = 0x6010;
  static constexpr uint16_t SLOPE_LONG_OPERATING_PARAMETER = 0x6020;
  static constexpr uint16_t SLOPE_LONG_PRESET = 0x6030;
  static constexpr uint16_t SLOPE_LONG_OFFSET = 0x6040;  // NOTE: not a controlword — see [[cia-profiles-reference]]
  static constexpr uint16_t SLOPE_LATERAL_16 = 0x6100;
  static constexpr uint16_t SLOPE_LATERAL_32 = 0x6110;
  static constexpr uint16_t SLOPE_LATERAL_OPERATING_PARAMETER = 0x6120;
  static constexpr uint16_t SLOPE_LATERAL_PRESET = 0x6130;
  static constexpr uint16_t SLOPE_LATERAL_OFFSET = 0x6140;
  static constexpr uint16_t RESOLUTION = 0x6200;
};

/**
 * Device-logic wrapper for a CiA 410 inclinometer.
 *
 * Reads tilt angles via the lely_driver bridge's local OD cache (populated by
 * incoming TPDOs from the slave). Writes presets/offsets via SDO. Conversion to
 * radians is done here so the node interface can publish strongly-typed messages.
 *
 * Resolution handling: when the slave publishes 0x6200, we read it during init and
 * compute degrees-per-LSB = 1 / resolution. Otherwise the YAML-configured fallback
 * is used. Final value is `raw * deg_per_lsb * π/180`.
 *
 * Single-channel device for now. Some inclinometers chain multiple sensors on one
 * node; if that becomes a need, follow the CiA 402-style 0x800 channel offset.
 */
class Inclinometer410
{
public:
  Inclinometer410(std::shared_ptr<LelyDriverBridge> driver, double deg_per_lsb_fallback = 0.01)
  : driver_(std::move(driver)), deg_per_lsb_(deg_per_lsb_fallback)
  {
  }

  /// Try to read 0x6200 to derive degrees-per-LSB. Called once after device boot.
  void refresh_resolution()
  {
    if (!driver_) return;
    try
    {
      if (driver_->has_object(Cia410Register::RESOLUTION, 0))
      {
        const auto resolution = driver_->universal_get_value<uint16_t>(Cia410Register::RESOLUTION, 0);
        if (resolution > 0)
        {
          deg_per_lsb_ = 1.0 / static_cast<double>(resolution);
        }
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("inclinometer_410"),
        "Could not read 0x6200 resolution (%s); using fallback %f deg/LSB", e.what(), deg_per_lsb_);
    }
  }

  /// Slope around long axis (X / pitch). Radians.
  double get_long_rad() const { return read_slope_rad(true); }

  /// Slope around lateral axis (Y / roll). Radians.
  double get_lateral_rad() const { return read_slope_rad(false); }

  bool has_lateral_axis() const
  {
    if (!driver_) return false;
    return driver_->has_object(Cia410Register::SLOPE_LATERAL_32, 0) ||
           driver_->has_object(Cia410Register::SLOPE_LATERAL_16, 0);
  }

  /// Write the preset value to capture-and-zero the long axis at the current angle.
  bool zero_long()
  {
    if (!driver_) return false;
    // CiA 410 preset is in the same units as the slope value (LSBs). 0 = zero point.
    return driver_->async_sdo_write_typed<int32_t>(Cia410Register::SLOPE_LONG_PRESET, 0, 0).get();
  }

  bool zero_lateral()
  {
    if (!driver_) return false;
    return driver_->async_sdo_write_typed<int32_t>(Cia410Register::SLOPE_LATERAL_PRESET, 0, 0).get();
  }

  /// Write an explicit offset (in radians) to either axis.
  bool set_long_offset_rad(double radians)
  {
    if (!driver_) return false;
    const int32_t raw = radians_to_raw(radians);
    return driver_->async_sdo_write_typed<int32_t>(Cia410Register::SLOPE_LONG_OFFSET, 0, raw).get();
  }

  bool set_lateral_offset_rad(double radians)
  {
    if (!driver_) return false;
    const int32_t raw = radians_to_raw(radians);
    return driver_->async_sdo_write_typed<int32_t>(Cia410Register::SLOPE_LATERAL_OFFSET, 0, raw)
      .get();
  }

  double deg_per_lsb() const { return deg_per_lsb_; }

private:
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  double read_slope_rad(bool is_long) const
  {
    if (!driver_) return 0.0;
    const uint16_t idx32 = is_long ? Cia410Register::SLOPE_LONG_32 : Cia410Register::SLOPE_LATERAL_32;
    const uint16_t idx16 = is_long ? Cia410Register::SLOPE_LONG_16 : Cia410Register::SLOPE_LATERAL_16;

    int32_t raw = 0;
    if (driver_->has_object(idx32, 0))
    {
      raw = driver_->universal_get_value<int32_t>(idx32, 0);
    }
    else if (driver_->has_object(idx16, 0))
    {
      raw = static_cast<int32_t>(driver_->universal_get_value<int16_t>(idx16, 0));
    }
    return static_cast<double>(raw) * deg_per_lsb_ * DEG_TO_RAD;
  }

  int32_t radians_to_raw(double radians) const
  {
    if (deg_per_lsb_ == 0.0) return 0;
    return static_cast<int32_t>(std::lround(radians * RAD_TO_DEG / deg_per_lsb_));
  }

  std::shared_ptr<LelyDriverBridge> driver_;
  double deg_per_lsb_;
};

}  // namespace ros2_canopen

#endif  // CANOPEN_410_DRIVER__INCLINOMETER_HPP_
