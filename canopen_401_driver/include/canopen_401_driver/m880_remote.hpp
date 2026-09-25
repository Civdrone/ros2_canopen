#ifndef CANOPEN_401_DRIVER__M880_REMOTE_HPP_
#define CANOPEN_401_DRIVER__M880_REMOTE_HPP_

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_canopen
{

/**
 * Object dictionary layout of a CiA 401 generic I/O device.
 *
 * Defaults target the IMET M880 CANopen radio remote receiver
 * (vendor 0x000004FA, product 0x4D383830, device type 0x00830191).
 *
 * The M880 implements only the *digital* half of CiA 401 -- its EDS defines
 * neither 0x6401 (read analog input) nor 0x6411 (write analog output). The
 * proportional joystick channels and the named command bits live in
 * manufacturer space instead, so those indices are part of this map rather
 * than being hardcoded to the standard.
 */
struct Cia401RegisterMap
{
  // Standard CiA 401 digital I/O, 8 channels per sub-index.
  // M880: 0x6000:01..1D (232 inputs), 0x6200:01..10 (128 outputs).
  uint16_t digital_input_index = 0x6000;
  uint16_t digital_output_index = 0x6200;

  // Manufacturer-specific. 0x2021 "Analog Channel safe" holds CH1..CH32 as
  // UNSIGNED8; 0x2020 "Commands safe" holds the named button/command bytes
  // (A1-A8 .. P1-P8, CTRL, Safety EN, STOP, START, HORN, ...).
  uint16_t analog_index = 0x2021;
  uint16_t command_index = 0x2020;
};

/**
 * One proportional channel, read as an UNSIGNED8 from `analog_index:sub`.
 *
 * Raw range is 0..255 with `center` at rest. Normalization is piecewise so an
 * off-center rest position still maps to exactly -1 / 0 / +1 at the extremes.
 */
struct AxisSpec
{
  uint8_t sub = 1;        // 1-based sub-index of analog_index (CH1 == sub 1)
  uint8_t center = 0x80;  // raw value at rest
  uint8_t deadband = 0;   // raw counts around center forced to 0.0
  bool invert = false;
};

/** A single bit inside an 8-bit object dictionary sub-index. */
struct BitSpec
{
  uint16_t index = 0x6000;
  uint8_t sub = 1;  // 1-based
  uint8_t bit = 0;  // 0..7, bit 0 == lowest-numbered channel in the byte
  bool active_low = false;
};

/**
 * Device-logic wrapper for the IMET M880 radio remote receiver.
 *
 * Reads button and axis state out of the lely_driver bridge's local OD cache,
 * which is kept current by inbound TPDOs. Digital outputs (0x6200) are written
 * back to the receiver.
 *
 * IMPORTANT -- only poll objects that are actually PDO-mapped.
 * `LelyDriverBridge::universal_get_value` silently falls back to a *blocking
 * SDO read* for any entry that is not TPDO-mapped. Polling an unmapped
 * sub-index at the driver's poll rate would stall the executor and flood the
 * bus with SDO traffic. The set of polled sub-indices is therefore explicit
 * (see `polled_digital_subs`), and defaults to exactly what the stock M880
 * TPDO configuration delivers.
 */
class M880Remote
{
public:
  M880Remote(
    std::shared_ptr<LelyDriverBridge> driver, Cia401RegisterMap registers,
    std::vector<AxisSpec> axes, std::vector<BitSpec> buttons,
    std::vector<uint8_t> polled_digital_subs)
  : driver_(std::move(driver)),
    registers_(registers),
    axes_(std::move(axes)),
    buttons_(std::move(buttons)),
    polled_digital_subs_(std::move(polled_digital_subs))
  {
  }

  /** Normalized axis values in [-1, 1], one per configured AxisSpec. */
  std::vector<float> read_axes() const
  {
    std::vector<float> values;
    values.reserve(axes_.size());
    for (const auto & axis : axes_)
    {
      values.push_back(normalize(read_u8(registers_.analog_index, axis.sub), axis));
    }
    return values;
  }

  /**
   * Button states.
   *
   * With an explicit `buttons:` list, returns one entry per BitSpec in order.
   * Without one, falls back to expanding every polled digital-input byte into
   * its 8 bits, so `buttons[(n * 8) + b]` is input `(n * 8) + b + 1`.
   */
  std::vector<int32_t> read_buttons() const
  {
    std::vector<int32_t> values;
    if (!buttons_.empty())
    {
      values.reserve(buttons_.size());
      for (const auto & button : buttons_)
      {
        values.push_back(read_bit(button) ? 1 : 0);
      }
      return values;
    }

    values.reserve(polled_digital_subs_.size() * 8);
    for (const uint8_t sub : polled_digital_subs_)
    {
      const uint8_t byte = read_u8(registers_.digital_input_index, sub);
      for (uint8_t bit = 0; bit < 8; ++bit)
      {
        values.push_back((byte >> bit) & 0x01);
      }
    }
    return values;
  }

  /** Reads one bit, applying its active_low polarity. */
  bool read_bit(const BitSpec & spec) const
  {
    if (!driver_ || spec.index == 0) return false;
    const uint8_t byte = read_u8(spec.index, spec.sub);
    const bool raw = ((byte >> (spec.bit & 0x07)) & 0x01) != 0;
    return spec.active_low ? !raw : raw;
  }

  /** Writes one byte of digital outputs (channels (sub-1)*8+1 .. (sub-1)*8+8). */
  bool write_output_byte(uint8_t sub, uint8_t value)
  {
    if (!driver_ || registers_.digital_output_index == 0) return false;
    try
    {
      driver_->universal_set_value<uint8_t>(registers_.digital_output_index, sub, value);
      return true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("m880_remote"), "Write to 0x%04X:%02X failed: %s",
        registers_.digital_output_index, sub, e.what());
      return false;
    }
  }

  size_t axis_count() const { return axes_.size(); }

  size_t button_count() const
  {
    return buttons_.empty() ? polled_digital_subs_.size() * 8 : buttons_.size();
  }

  const Cia401RegisterMap & registers() const { return registers_; }
  const std::vector<AxisSpec> & axes() const { return axes_; }
  const std::vector<uint8_t> & polled_digital_subs() const { return polled_digital_subs_; }

private:
  uint8_t read_u8(uint16_t index, uint8_t sub) const
  {
    if (!driver_ || index == 0) return 0;
    if (!driver_->has_object(index, sub)) return 0;
    return driver_->universal_get_value<uint8_t>(index, sub);
  }

  static float normalize(uint8_t raw, const AxisSpec & axis)
  {
    const int value = static_cast<int>(raw);
    const int center = static_cast<int>(axis.center);
    const int delta = value - center;

    if (std::abs(delta) <= static_cast<int>(axis.deadband)) return 0.0f;

    // Scale each side independently so an off-center rest point still reaches
    // exactly +/-1.0 at the raw extremes.
    const int span = delta > 0 ? (255 - center) : center;
    if (span <= 0) return 0.0f;

    float normalized = static_cast<float>(delta) / static_cast<float>(span);
    normalized = std::clamp(normalized, -1.0f, 1.0f);
    return axis.invert ? -normalized : normalized;
  }

  std::shared_ptr<LelyDriverBridge> driver_;
  Cia401RegisterMap registers_;
  std::vector<AxisSpec> axes_;
  std::vector<BitSpec> buttons_;
  std::vector<uint8_t> polled_digital_subs_;
};

}  // namespace ros2_canopen

#endif  // CANOPEN_401_DRIVER__M880_REMOTE_HPP_
