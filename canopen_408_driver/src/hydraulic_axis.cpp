#include "canopen_408_driver/hydraulic_axis.hpp"

#include <algorithm>

namespace ros2_canopen
{

namespace
{
constexpr const char * LOGGER = "hydraulic_axis_408";
// spin_once() runs on the ~10 ms poll timer; each DSM phase is streamed for this
// many cycles before advancing (the device only holds a state while it keeps
// receiving the matching control word).
constexpr int PHASE_DWELL_TICKS = 15;  // ~150 ms per phase
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
  // Non-blocking: kick off the DSM climb; spin_once() advances it over time.
  // Start from Init so a re-enable after disable() re-normalizes the DSM the same
  // way a power-cycle/NMT-reset does. From a fresh boot Init is a no-op.
  std::scoped_lock lock(io_mutex_);
  last_setpoint_.store(0);
  enabled_.store(false);
  phase_ticks_ = 0;
  phase_.store(Phase::Init);
  return true;
}

bool HydraulicAxis408::recover()
{
  // Non-blocking: fault-reset first, then the full enable climb.
  std::scoped_lock lock(io_mutex_);
  last_setpoint_.store(0);
  enabled_.store(false);
  phase_ticks_ = 0;
  phase_.store(Phase::FaultReset);
  return true;
}

bool HydraulicAxis408::hold()
{
  std::scoped_lock lock(io_mutex_);
  phase_.store(Phase::Idle);
  enabled_.store(false);
  current_cw_.store(CW_HOLD);
  return true;
}

bool HydraulicAxis408::disable()
{
  std::scoped_lock lock(io_mutex_);
  phase_.store(Phase::Idle);
  enabled_.store(false);
  current_cw_.store(CW_DISABLED);
  return true;
}

void HydraulicAxis408::advance_phase_locked()
{
  const Phase p = phase_.load();
  if (p == Phase::Idle) return;

  uint16_t cw;
  switch (p)
  {
    case Phase::FaultReset: cw = CW_FAULT_RESET; break;
    case Phase::Init:       cw = CW_INIT; break;
    case Phase::Disabled:   cw = CW_DISABLED; break;
    case Phase::Hold:       cw = CW_HOLD; break;
    case Phase::Active:     cw = CW_ACTIVE; break;
    default:                return;
  }
  current_cw_.store(cw);  // streamed by spin_once() after this returns

  if (++phase_ticks_ < PHASE_DWELL_TICKS) return;  // keep holding this phase
  phase_ticks_ = 0;

  Phase next;
  switch (p)
  {
    case Phase::FaultReset: next = Phase::Init; break;
    case Phase::Init:       next = Phase::Disabled; break;
    case Phase::Disabled:   next = Phase::Hold; break;
    case Phase::Hold:       next = Phase::Active; break;
    case Phase::Active:     next = Phase::Idle; enabled_.store(true); break;
    default:                next = Phase::Idle; break;
  }
  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER), "DSM: phase control word 0x%02X held -> status word 0x%04X",
    cw, get_statusword());
  phase_.store(next);
}

void HydraulicAxis408::spin_once()
{
  std::scoped_lock lock(io_mutex_);
  advance_phase_locked();
  // Stream control word + set point every cycle so the PVED's flow-command
  // time-guard (EMCY 0x8003) never trips and the DSM phase is held.
  write_controlword(current_cw_.load());
  send_setpoint(last_setpoint_.load());
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
