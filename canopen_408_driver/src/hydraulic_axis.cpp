#include "canopen_408_driver/hydraulic_axis.hpp"

#include <algorithm>
#include <thread>

namespace ros2_canopen
{

namespace
{
constexpr const char * LOGGER = "hydraulic_axis_408";
constexpr auto DSM_STEP_DELAY = std::chrono::milliseconds(50);
// The PVED only holds a DSM state while it keeps receiving the matching control
// word, so each state is streamed at this interval for this dwell before advancing.
constexpr auto DSM_STREAM_INTERVAL = std::chrono::milliseconds(10);
constexpr auto DSM_STATE_DWELL = std::chrono::milliseconds(200);
}  // namespace

// universal_set_value routes automatically: for objects the master transmits
// (control word / device mode / set point, mapped into RPDOs) it updates the
// cache and fires the PDO WriteEvent; for unmapped objects it falls back to SDO.
bool HydraulicAxis408::write_controlword(uint16_t value)
{
  if (!driver_) return false;
  try
  {
    driver_->universal_set_value<uint16_t>(Cia408Register::CONTROLWORD, 0, value);
    current_cw_.store(value);
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "controlword write failed: %s", e.what());
    return false;
  }
}

bool HydraulicAxis408::set_device_mode(uint8_t mode)
{
  if (!driver_) return false;
  try
  {
    // Seed the cached value so RPDO2 (control word + device mode) always carries
    // the intended mode on subsequent control-word transmissions.
    driver_->universal_set_value<uint8_t>(Cia408Register::DEVICE_MODE, 0, mode);
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(rclcpp::get_logger(LOGGER), "set_device_mode failed: %s", e.what());
    return false;
  }
}

bool HydraulicAxis408::send_setpoint(int16_t raw)
{
  if (!driver_) return false;
  try
  {
    driver_->universal_set_value<int16_t>(
      Cia408Register::SETPOINT, Cia408Register::SETPOINT_SUB, raw);
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "set point write failed: %s", e.what());
    return false;
  }
}

bool HydraulicAxis408::enable()
{
  std::scoped_lock lock(io_mutex_);
  set_device_mode(MODE_CAN_CONTROLLED);
  last_setpoint_.store(0);

  // Climb the DSM Disabled -> Hold -> Active. Each control word must be streamed
  // (not sent once) because the device falls back to 0x03 within a few ms if it
  // stops receiving it; Hold->Active is only accepted while actually in Hold.
  // (TPDO status updates on the Lely CAN thread, so get_statusword() stays fresh
  // even while this loop blocks the ROS executor.)
  const auto stream_state = [this](uint16_t cw)
  {
    const auto deadline = std::chrono::steady_clock::now() + DSM_STATE_DWELL;
    do
    {
      write_controlword(cw);
      send_setpoint(0);
      std::this_thread::sleep_for(DSM_STREAM_INTERVAL);
    } while (std::chrono::steady_clock::now() < deadline);
    RCLCPP_INFO(
      rclcpp::get_logger(LOGGER), "DSM: streamed control word 0x%02X -> status word 0x%04X",
      cw, get_statusword());
  };

  // Start from Init so a re-enable after disable() re-normalizes the DSM the same
  // way a power-cycle/NMT-reset does (which is the only thing that recovered it
  // before). From a fresh boot the device is already in Init, so 0x08 is a no-op.
  stream_state(CW_INIT);
  stream_state(CW_DISABLED);
  stream_state(CW_HOLD);
  stream_state(CW_ACTIVE);

  current_cw_.store(CW_ACTIVE);
  enabled_.store(true);
  const uint16_t sw = get_statusword();
  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER), "enable() complete, final status word 0x%04X", sw);
  return true;
}

bool HydraulicAxis408::hold()
{
  std::scoped_lock lock(io_mutex_);
  enabled_.store(false);
  return write_controlword(CW_HOLD);
}

bool HydraulicAxis408::disable()
{
  std::scoped_lock lock(io_mutex_);
  enabled_.store(false);
  return write_controlword(CW_DISABLED);
}

bool HydraulicAxis408::recover()
{
  {
    std::scoped_lock lock(io_mutex_);
    enabled_.store(false);
    if (!write_controlword(CW_FAULT_RESET)) return false;
    std::this_thread::sleep_for(DSM_STEP_DELAY);
    if (!write_controlword(CW_HOLD)) return false;
  }
  std::this_thread::sleep_for(DSM_STEP_DELAY);
  return enable();
}

bool HydraulicAxis408::set_setpoint(int16_t raw)
{
  if (raw != SETPOINT_FLOAT && raw != -SETPOINT_FLOAT)
  {
    raw = std::clamp<int16_t>(raw, -SETPOINT_FULL_SCALE, SETPOINT_FULL_SCALE);
  }
  last_setpoint_.store(raw);
  std::scoped_lock lock(io_mutex_);
  return send_setpoint(raw);
}

bool HydraulicAxis408::set_float()
{
  return set_setpoint(SETPOINT_FLOAT);
}

void HydraulicAxis408::refresh_outputs()
{
  std::scoped_lock lock(io_mutex_);
  // Stream control word + set point EVERY cycle, even before enable(), so the
  // PVED's flow-command time-guard (EMCY 0x8003, "flow command not received
  // within timeout") never trips. Before enable() the control word holds a safe
  // Disabled state (CW_DISABLED) and the set point is neutral (0); enable()
  // switches it to CW_ACTIVE and set_setpoint() supplies the command.
  write_controlword(current_cw_.load());
  send_setpoint(last_setpoint_.load());
}

bool HydraulicAxis408::save_to_eeprom()
{
  if (!driver_) return false;
  try
  {
    return driver_
      ->async_sdo_write_typed<uint32_t>(
        Cia408Register::STORE_PARAMS, Cia408Register::STORE_SUB, STORE_SIGNATURE)
      .get();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "save to EEPROM failed: %s", e.what());
    return false;
  }
}

int16_t HydraulicAxis408::get_spool_position() const
{
  if (driver_ && driver_->has_object(Cia408Register::ACTUAL, Cia408Register::ACTUAL_SUB))
  {
    return driver_->universal_get_value<int16_t>(
      Cia408Register::ACTUAL, Cia408Register::ACTUAL_SUB);
  }
  return 0;
}

int16_t HydraulicAxis408::get_demand() const
{
  if (driver_ && driver_->has_object(Cia408Register::DEMAND, 0))
  {
    return driver_->universal_get_value<int16_t>(Cia408Register::DEMAND, 0);
  }
  return 0;
}

uint16_t HydraulicAxis408::get_statusword() const
{
  if (driver_ && driver_->has_object(Cia408Register::STATUSWORD, 0))
  {
    return driver_->universal_get_value<uint16_t>(Cia408Register::STATUSWORD, 0);
  }
  return 0;
}

uint16_t HydraulicAxis408::get_pcb_temperature() const
{
  if (driver_ && driver_->has_object(Cia408Register::PCB_TEMPERATURE, 0))
  {
    return driver_->universal_get_value<uint16_t>(Cia408Register::PCB_TEMPERATURE, 0);
  }
  return 0;
}

}  // namespace ros2_canopen
