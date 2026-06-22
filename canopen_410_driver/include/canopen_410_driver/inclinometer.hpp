#ifndef CANOPEN_410_DRIVER__INCLINOMETER_HPP_
#define CANOPEN_410_DRIVER__INCLINOMETER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_canopen
{

/**
 * Object dictionary indices used by the inclinometer driver.
 *
 * Defaults follow the CiA 410 standard (long at 0x6010/0x6000, lateral at
 * 0x6110/0x6100, resolution at 0x6200, preset/offset at 0x6030/0x6040 etc.).
 *
 * Some vendors deviate from the standard — notably Posital's DST series, which
 * puts the lateral axis at 0x6020, the resolution at 0x6000, and uses 16-bit
 * slope values. All indices and widths are overridable from bus.yml via a
 * `register_map:` block.
 */
struct Cia410RegisterMap
{
  // Slope long ("longitudinal" axis — typically pitch).
  uint16_t slope_long_index = 0x6010;
  uint8_t slope_long_bits = 32;            // 16 or 32

  // Slope lateral (lateral axis — typically roll).
  uint16_t slope_lateral_index = 0x6110;
  uint8_t slope_lateral_bits = 32;         // 16 or 32

  // Preset (zero-set) objects. 0 = service is disabled.
  uint16_t slope_long_preset_index = 0x6030;
  uint16_t slope_lateral_preset_index = 0x6130;

  // Offset objects (additive). 0 = service is disabled.
  uint16_t slope_long_offset_index = 0x6040;
  uint16_t slope_lateral_offset_index = 0x6140;

  // Bit-width for preset/offset writes (16 or 32). Defaults match the indices
  // above; on Posital devices these are 16-bit.
  uint8_t preset_bits = 32;
  uint8_t offset_bits = 32;
};

/**
 * Device-logic wrapper for a CiA 410 inclinometer.
 *
 * Reads tilt angles via the lely_driver bridge's local OD cache (populated by
 * incoming TPDOs from the slave). Writes presets/offsets via SDO. Conversion to
 * radians is done here so the node interface can publish strongly-typed messages.
 *
 * Resolution: degrees-per-LSB is provided via YAML (`deg_per_lsb`). Vendors
 * disagree on the formula for any auto-detect register (CiA standard 0x6200
 * uses 1/value while Posital's 0x6000 uses value/1000), so the driver does not
 * try to derive it at runtime — the YAML value is authoritative.
 */
class Inclinometer410
{
public:
  Inclinometer410(
    std::shared_ptr<LelyDriverBridge> driver, Cia410RegisterMap registers,
    double deg_per_lsb)
  : driver_(std::move(driver)),
    registers_(registers),
    deg_per_lsb_(deg_per_lsb)
  {
  }

  double get_long_rad() const
  {
    return read_slope_rad(registers_.slope_long_index, registers_.slope_long_bits);
  }

  double get_lateral_rad() const
  {
    return read_slope_rad(registers_.slope_lateral_index, registers_.slope_lateral_bits);
  }

  bool has_lateral_axis() const
  {
    if (!driver_) return false;
    return driver_->has_object(registers_.slope_lateral_index, 0);
  }

  bool zero_long()
  {
    return write_preset_or_offset(registers_.slope_long_preset_index, 0, registers_.preset_bits);
  }

  bool zero_lateral()
  {
    return write_preset_or_offset(registers_.slope_lateral_preset_index, 0, registers_.preset_bits);
  }

  bool set_long_offset_rad(double radians)
  {
    return write_preset_or_offset(
      registers_.slope_long_offset_index, radians_to_raw(radians), registers_.offset_bits);
  }

  bool set_lateral_offset_rad(double radians)
  {
    return write_preset_or_offset(
      registers_.slope_lateral_offset_index, radians_to_raw(radians), registers_.offset_bits);
  }

  double deg_per_lsb() const { return deg_per_lsb_; }
  const Cia410RegisterMap & registers() const { return registers_; }

private:
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  double read_slope_rad(uint16_t index, uint8_t bits) const
  {
    if (!driver_ || index == 0) return 0.0;
    if (!driver_->has_object(index, 0)) return 0.0;

    int32_t raw = 0;
    if (bits == 16)
    {
      raw = static_cast<int32_t>(driver_->universal_get_value<int16_t>(index, 0));
    }
    else
    {
      raw = driver_->universal_get_value<int32_t>(index, 0);
    }
    return static_cast<double>(raw) * deg_per_lsb_ * DEG_TO_RAD;
  }

  int32_t radians_to_raw(double radians) const
  {
    if (deg_per_lsb_ == 0.0) return 0;
    return static_cast<int32_t>(std::lround(radians * RAD_TO_DEG / deg_per_lsb_));
  }

  bool write_preset_or_offset(uint16_t index, int32_t raw, uint8_t bits)
  {
    if (!driver_ || index == 0) return false;
    try
    {
      if (bits == 16)
      {
        return driver_
          ->async_sdo_write_typed<int16_t>(index, 0, static_cast<int16_t>(raw))
          .get();
      }
      return driver_->async_sdo_write_typed<int32_t>(index, 0, raw).get();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("inclinometer_410"), "Write to 0x%04X failed: %s", index, e.what());
      return false;
    }
  }

  std::shared_ptr<LelyDriverBridge> driver_;
  Cia410RegisterMap registers_;
  double deg_per_lsb_;
};

}  // namespace ros2_canopen

#endif  // CANOPEN_410_DRIVER__INCLINOMETER_HPP_
