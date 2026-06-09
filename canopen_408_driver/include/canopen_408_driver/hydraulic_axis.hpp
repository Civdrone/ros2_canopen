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

// CiA 408 device profile object dictionary indices.
// References:
//   * CiA DSP-408 "Device profile for fluid power technology – proportional valves and
//     hydrostatic transmissions"
//   * Danfoss-specific OD entries (typically 0x2000–0x5FFF) are NOT baked in here;
//     wire them through bus.yml `sdo:` blocks until vendor EDS is in hand.
//
// The state machine objects (0x6040 / 0x6041) match CiA 402, by design: CiA 408
// reuses the 402-style controlword/statusword. The mode and setpoint/feedback
// pairs live at different indices than 402, however.
struct Cia408Register
{
  static constexpr uint16_t CONTROLWORD = 0x6040;
  static constexpr uint16_t STATUSWORD = 0x6041;
  static constexpr uint16_t DEVICE_MODE = 0x6042;
  static constexpr uint16_t DEVICE_MODE_DISPLAY = 0x6043;
  static constexpr uint16_t DEVICE_ERROR = 0x6044;

  // Generic / position-mode profile entries (default mode for this driver).
  static constexpr uint16_t TARGET_VALUE = 0x6300;       // Setpoint (interpreted by active mode)
  static constexpr uint16_t ACTUAL_VALUE = 0x6301;       // Feedback (interpreted by active mode)
  static constexpr uint16_t MIN_LIMIT = 0x6302;
  static constexpr uint16_t MAX_LIMIT = 0x6303;
  static constexpr uint16_t RAMP_UP = 0x6304;
  static constexpr uint16_t RAMP_DOWN = 0x6305;
  static constexpr uint16_t MANUAL_COMMAND = 0x6306;

  // Pressure-mode setpoint/feedback (unused by default driver; available for extension).
  static constexpr uint16_t TARGET_PRESSURE = 0x6310;
  static constexpr uint16_t ACTUAL_PRESSURE = 0x6311;

  // Flow-mode setpoint/feedback (unused by default driver; available for extension).
  static constexpr uint16_t TARGET_FLOW = 0x6320;
  static constexpr uint16_t ACTUAL_FLOW = 0x6321;
};

// CiA 408 device mode enum (subset). Vendor-specific values may extend this.
enum class Cia408Mode : uint8_t
{
  NoMode = 0,
  Manual = 1,
  Position = 2,
  Pressure = 3,
  Flow = 4,
  Speed = 5,
};

// CiA 408 reuses the CiA 402 state machine.
enum class HydraulicState : uint8_t
{
  NotReadyToSwitchOn = 0,
  SwitchOnDisabled = 1,
  ReadyToSwitchOn = 2,
  SwitchedOn = 3,
  OperationEnabled = 4,
  QuickStopActive = 5,
  FaultReactionActive = 6,
  Fault = 7,
  Unknown = 0xFF,
};

/**
 * Device-logic wrapper for a CiA 408 hydraulic / fluid-power node.
 *
 * Drives the controlword (0x6040) through the CiA 402-style state machine, sets the
 * device mode (0x6042, default Position) and the setpoint (0x6300). Reads feedback
 * from 0x6301 and decodes the statusword (0x6041).
 *
 * Vendor extensions (Danfoss): use the bus.yml `sdo:` block for one-time configuration
 * writes into the manufacturer range (0x2000–0x5FFF). Run-time vendor-specific behavior
 * should be added here as separate methods.
 */
class HydraulicAxis408
{
public:
  HydraulicAxis408(std::shared_ptr<LelyDriverBridge> driver, Cia408Mode default_mode = Cia408Mode::Position)
  : driver_(std::move(driver)), target_mode_(default_mode)
  {
  }

  /// Run the state-machine bootstrap: clear faults if any, then transition
  /// SwitchOnDisabled → ReadyToSwitchOn → SwitchedOn → OperationEnabled.
  /// Returns true if the device reaches OperationEnabled within timeout.
  bool init(std::chrono::milliseconds timeout = std::chrono::seconds(5));

  /// Send a quick-stop command (controlword bit 2 cleared).
  bool halt();

  /// Fault reset + re-enable.
  bool recover();

  /// Bring the device to SwitchOnDisabled. Counterpart of init().
  bool shutdown();

  /// Write target value (0x6300) — interpretation depends on the active mode.
  bool set_target(int32_t raw_target);

  /// Read feedback (0x6301).
  int32_t get_actual() const;

  /// Read statusword (0x6041).
  uint16_t get_statusword() const;

  /// Read device-mode display (0x6043).
  uint8_t get_mode_display() const;

  /// Change device mode (writes 0x6042). The driver remembers the requested mode
  /// and re-asserts it during init/recover.
  bool set_mode(Cia408Mode mode);

  Cia408Mode requested_mode() const { return target_mode_; }

  HydraulicState parse_state(uint16_t statusword) const;

  HydraulicState get_state() const { return parse_state(get_statusword()); }

private:
  // Controlword bit definitions — identical to CiA 402.
  enum ControlwordBit : uint16_t
  {
    CW_SWITCH_ON = 1 << 0,
    CW_ENABLE_VOLTAGE = 1 << 1,
    CW_QUICK_STOP = 1 << 2,        // active LOW per CiA 402 — bit must be SET to clear quick-stop
    CW_ENABLE_OPERATION = 1 << 3,
    CW_FAULT_RESET = 1 << 7,
    CW_HALT = 1 << 8,
  };

  // Composite controlword values for the standard CiA 402-style transitions.
  // Quick-stop bit (CW_QUICK_STOP) is set in all "active" values per the spec
  // (it's a "stop is NOT requested" indicator).
  static constexpr uint16_t CW_SHUTDOWN = CW_ENABLE_VOLTAGE | CW_QUICK_STOP;
  static constexpr uint16_t CW_SWITCH_ON_CMD = CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP;
  static constexpr uint16_t CW_ENABLE_OP_CMD =
    CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP | CW_ENABLE_OPERATION;
  static constexpr uint16_t CW_QUICK_STOP_CMD = CW_ENABLE_VOLTAGE;  // bit 2 cleared
  static constexpr uint16_t CW_DISABLE_VOLTAGE_CMD = 0;
  static constexpr uint16_t CW_FAULT_RESET_CMD = CW_FAULT_RESET;

  bool write_controlword(uint16_t value);
  bool wait_for_state(HydraulicState desired, std::chrono::milliseconds timeout);

  std::shared_ptr<LelyDriverBridge> driver_;
  Cia408Mode target_mode_;
  mutable std::mutex io_mutex_;
};

}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__HYDRAULIC_AXIS_HPP_
