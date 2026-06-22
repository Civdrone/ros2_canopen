#include "canopen_408_driver/hydraulic_axis.hpp"

#include <thread>

namespace ros2_canopen
{

namespace
{
constexpr const char * LOGGER = "hydraulic_axis_408";
}

bool HydraulicAxis408::write_controlword(uint16_t value)
{
  if (!driver_) return false;
  try
  {
    return driver_->async_sdo_write_typed<uint16_t>(Cia408Register::CONTROLWORD, 0, value).get();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "controlword write failed: %s", e.what());
    return false;
  }
}

uint16_t HydraulicAxis408::get_statusword() const
{
  if (!driver_) return 0;
  if (driver_->has_object(Cia408Register::STATUSWORD, 0))
  {
    return driver_->universal_get_value<uint16_t>(Cia408Register::STATUSWORD, 0);
  }
  return 0;
}

uint8_t HydraulicAxis408::get_mode_display() const
{
  if (!driver_) return 0;
  if (driver_->has_object(Cia408Register::DEVICE_MODE_DISPLAY, 0))
  {
    return driver_->universal_get_value<uint8_t>(Cia408Register::DEVICE_MODE_DISPLAY, 0);
  }
  return 0;
}

int32_t HydraulicAxis408::get_actual() const
{
  if (!driver_) return 0;
  if (driver_->has_object(Cia408Register::ACTUAL_VALUE, 0))
  {
    return driver_->universal_get_value<int32_t>(Cia408Register::ACTUAL_VALUE, 0);
  }
  return 0;
}

HydraulicState HydraulicAxis408::parse_state(uint16_t sw) const
{
  // CiA 402-style decoding (also used by CiA 408). Bits:
  //  0 RTSO, 1 SwitchedOn, 2 OpEnabled, 3 Fault, 5 QuickStop(active-low), 6 SwitchOnDisabled
  const uint16_t mask = 0x6F;  // bits 0..3, 5, 6
  const uint16_t v = sw & mask;

  if ((sw >> 3) & 1) return HydraulicState::Fault;
  if ((sw >> 6) & 1) return HydraulicState::SwitchOnDisabled;
  if ((v & 0x27) == 0x27) return HydraulicState::OperationEnabled;     // 0b0010_0111 = RTSO|SwOn|OpEn|QS
  if ((v & 0x23) == 0x23) return HydraulicState::SwitchedOn;           // 0b0010_0011
  if ((v & 0x21) == 0x21) return HydraulicState::ReadyToSwitchOn;      // 0b0010_0001
  if (((sw >> 5) & 1) == 0 && ((sw >> 2) & 1)) return HydraulicState::QuickStopActive;
  return HydraulicState::Unknown;
}

bool HydraulicAxis408::wait_for_state(HydraulicState desired, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (get_state() == desired) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  RCLCPP_WARN(
    rclcpp::get_logger(LOGGER), "Timeout waiting for state %u, currently %u",
    static_cast<unsigned>(desired), static_cast<unsigned>(get_state()));
  return false;
}

bool HydraulicAxis408::init(std::chrono::milliseconds timeout)
{
  std::scoped_lock lock(io_mutex_);

  // Set requested mode first so the device boots into the right behavior.
  try
  {
    driver_->async_sdo_write_typed<uint8_t>(
        Cia408Register::DEVICE_MODE, 0, static_cast<uint8_t>(target_mode_))
      .get();
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(rclcpp::get_logger(LOGGER), "set_mode at init failed: %s", e.what());
  }

  const auto current = get_state();
  if (current == HydraulicState::Fault)
  {
    if (!write_controlword(CW_FAULT_RESET_CMD)) return false;
    if (!wait_for_state(HydraulicState::SwitchOnDisabled, timeout)) return false;
  }

  if (!write_controlword(CW_SHUTDOWN)) return false;
  if (!wait_for_state(HydraulicState::ReadyToSwitchOn, timeout)) return false;

  if (!write_controlword(CW_SWITCH_ON_CMD)) return false;
  if (!wait_for_state(HydraulicState::SwitchedOn, timeout)) return false;

  if (!write_controlword(CW_ENABLE_OP_CMD)) return false;
  if (!wait_for_state(HydraulicState::OperationEnabled, timeout)) return false;

  return true;
}

bool HydraulicAxis408::halt()
{
  std::scoped_lock lock(io_mutex_);
  return write_controlword(CW_QUICK_STOP_CMD);
}

bool HydraulicAxis408::recover()
{
  std::scoped_lock lock(io_mutex_);
  if (!write_controlword(CW_FAULT_RESET_CMD)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!write_controlword(CW_SHUTDOWN)) return false;
  if (!write_controlword(CW_ENABLE_OP_CMD)) return false;
  return wait_for_state(HydraulicState::OperationEnabled, std::chrono::seconds(2));
}

bool HydraulicAxis408::shutdown()
{
  std::scoped_lock lock(io_mutex_);
  return write_controlword(CW_DISABLE_VOLTAGE_CMD);
}

bool HydraulicAxis408::set_target(int32_t raw_target)
{
  if (!driver_) return false;
  // Setpoint is usually mapped into an RPDO for low-latency updates; if so, an SDO
  // write still works and reaches the same OD entry. The PDO path is preferred when
  // mapped in bus.yml.
  if (driver_->has_object(Cia408Register::TARGET_VALUE, 0))
  {
    try
    {
      driver_->universal_set_value<int32_t>(Cia408Register::TARGET_VALUE, 0, raw_target);
      driver_->tpdo_mapped[Cia408Register::TARGET_VALUE][0].WriteEvent();
      return true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "set_target failed: %s", e.what());
      return false;
    }
  }
  return false;
}

bool HydraulicAxis408::set_mode(Cia408Mode mode)
{
  if (!driver_) return false;
  target_mode_ = mode;
  try
  {
    return driver_
      ->async_sdo_write_typed<uint8_t>(
        Cia408Register::DEVICE_MODE, 0, static_cast<uint8_t>(mode))
      .get();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "set_mode failed: %s", e.what());
    return false;
  }
}

}  // namespace ros2_canopen
