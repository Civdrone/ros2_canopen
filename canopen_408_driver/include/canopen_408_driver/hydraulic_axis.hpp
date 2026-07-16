#ifndef CANOPEN_408_DRIVER__HYDRAULIC_AXIS_HPP_
#define CANOPEN_408_DRIVER__HYDRAULIC_AXIS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_canopen
{

// Object dictionary of the Danfoss PVED-CC Series 5 CANopen valve.
// Source: technical information manual BC180386484705en-000802, cross-checked
// against a live unit over SDO. See canopen_tests/config/cia408/PVE_CC.eds.
//
// NOTE: this is a CiA 408 device (0x1000 = 0x0198) but it does NOT use the
// CiA 402 controlword/statusword bit semantics. 0x6040 drives a VDMAPROP device
// state machine via discrete command values (see ControlWord below), and the
// set point / actual value are INTEGER16 living at sub-index 0x01 of a record.
struct Cia408Register
{
  static constexpr uint16_t CONTROLWORD = 0x6040;       // Device control word (DSM commands)
  static constexpr uint16_t STATUSWORD = 0x6041;        // Device status word
  static constexpr uint16_t DEVICE_MODE = 0x6042;       // 0x01 CAN controlled, 0x02 hand operated

  static constexpr uint16_t SETPOINT = 0x6300;          // Vpoc set point (record)
  static constexpr uint8_t SETPOINT_SUB = 0x01;         // int16, -16384..+16384 = -100%..+100%
  static constexpr uint16_t ACTUAL = 0x6301;            // Vpoc actual value = spool position (record)
  static constexpr uint8_t ACTUAL_SUB = 0x01;           // int16, -16384..+16384 = -100%..+100%

  static constexpr uint16_t DEMAND = 0x6310;            // Demand value (int16)
  static constexpr uint16_t PCB_TEMPERATURE = 0x3468;   // Current PCB temperature (uint16)
  static constexpr uint16_t BATTERY_VOLTAGE = 0x3469;   // Battery voltage, 0.1 V/LSB (uint16)

  static constexpr uint16_t STORE_PARAMS = 0x1010;      // Save-to-EEPROM
  static constexpr uint8_t STORE_SUB = 0x01;
};

// Device state machine command values written to 0x6040 (VDMAPROP DSM).
// Sequence to reach the running "Device_Mode_Active" state from boot:
//   Init(0x08) -> Disabled(0x09) -> Hold(0x0B) -> Active(0x0F)
enum ControlWord : uint16_t
{
  CW_INIT = 0x08,         // D7: -> Init
  CW_DISABLED = 0x09,     // D2/D6: -> Disabled
  CW_HOLD = 0x0B,         // D3/D5/D11: -> Hold (spool held at neutral, not controlled)
  CW_ACTIVE = 0x0F,       // D4: -> Device_Mode_Active (spool follows set point)
  CW_FAULT_RESET = 0x03,  // D11: Fault_Hold -> Hold
};

enum DeviceMode : uint8_t
{
  MODE_CAN_CONTROLLED = 0x01,
  MODE_HAND_OPERATED = 0x02,
};

// Set point extremes.
static constexpr int16_t SETPOINT_FULL_SCALE = 16384;   // = 100% spool travel
static constexpr int16_t SETPOINT_FLOAT = 32767;        // request float state
// Save-to-EEPROM signature: ASCII "save" (0x65766173), per manual.
static constexpr uint32_t STORE_SIGNATURE = 0x65766173;

/**
 * Device-logic wrapper for a Danfoss PVED-CC Series 5 fluid-power valve.
 *
 * Drives the control word (0x6040) through the VDMAPROP device state machine,
 * keeps the device in CAN-controlled mode (0x6042), streams the spool set point
 * (0x6300:1) and reads the spool position (0x6301:1) + status word (0x6041).
 *
 * Set point / feedback are handled as raw int16 here; engineering-unit scaling
 * (percent) is applied one layer up in NodeCanopen408Driver.
 */
class HydraulicAxis408
{
public:
  explicit HydraulicAxis408(std::shared_ptr<LelyDriverBridge> driver)
  : driver_(std::move(driver))
  {
  }

  /// Put the device in CAN-controlled mode and step the DSM
  /// Init -> Disabled -> Hold -> Device_Mode_Active. After this the valve
  /// follows the set point. Returns true if all commands were accepted.
  bool enable();

  /// Bring the DSM to Hold (spool commanded to neutral, no longer controlled).
  bool hold();

  /// Bring the DSM to Disabled.
  bool disable();

  /// Fault reset (Fault_Hold -> Hold) followed by re-enable.
  bool recover();

  /// Write the spool set point (0x6300:1) via the mapped RPDO. `raw` is clamped
  /// to +/-SETPOINT_FULL_SCALE unless it is the sentinel float value.
  bool set_setpoint(int16_t raw);

  /// Command the float state (set point = SETPOINT_FLOAT).
  bool set_float();

  /// Re-transmit the current control word + set point. Call periodically so the
  /// device's RPDO time-guard does not trip (fault: "RPDO not received within
  /// timeout"). No-op until enable() has run.
  void refresh_outputs();

  /// Persist the current parameters to EEPROM (write "save" to 0x1010:1).
  bool save_to_eeprom();

  int16_t get_spool_position() const;   ///< 0x6301:1
  int16_t get_demand() const;           ///< 0x6310
  uint16_t get_statusword() const;      ///< 0x6041
  uint16_t get_pcb_temperature() const; ///< 0x3468 (raw)

  bool is_enabled() const { return enabled_.load(); }

private:
  bool write_controlword(uint16_t value);
  bool set_device_mode(uint8_t mode);
  bool send_setpoint(int16_t raw);

  std::shared_ptr<LelyDriverBridge> driver_;
  std::atomic<bool> enabled_{false};
  std::atomic<uint16_t> current_cw_{CW_DISABLED};
  std::atomic<int16_t> last_setpoint_{0};
  mutable std::mutex io_mutex_;
};

}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__HYDRAULIC_AXIS_HPP_
