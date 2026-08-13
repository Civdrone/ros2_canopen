#ifndef CANOPEN_408_DRIVER__PVED_FAULTS_HPP_
#define CANOPEN_408_DRIVER__PVED_FAULTS_HPP_

#include <cstdint>
#include <string>

namespace ros2_canopen
{

// Danfoss PVED-CC Series 5 fault decoding, from the diagnostics log in manual
// BC180386484705en-000802. Faults are broadcast as CANopen EMCY messages
// (COB-ID 0x080 + node id): bytes 0-1 = emergency error code (eec), byte 2 =
// error register, bytes 3-7 = manufacturer specific field (msef):
//   msef[0] = occurrence counter, msef[1] = fault ID, msef[4] = severity level.

/// Human-readable name for a PVED-CC emergency error code. Returns nullptr if the
/// code is not in the documented table (caller should fall back to the raw hex).
inline const char * pved_fault_name(uint16_t eec)
{
  switch (eec)
  {
    case 0x0000: return "No error / fault cleared";
    case 0x6200: return "Software initialization fault";
    case 0x6201: return "Internal calculation fault";
    case 0x6203: return "Parameter truncation change";
    case 0x6204: return "Interpolation fault";
    case 0x3411: return "Supply voltage above upper limit";
    case 0x3412: return "Supply voltage below lower limit";
    case 0x3414: return "5V PSU out of range";
    case 0x620B: return "Spool position calculation fault";
    case 0x3413: return "V reference signal out of range";
    case 0x3415: return "GND signal unstable";
    case 0x5235: return "Demodulator A: signal out of range";
    case 0x5236: return "Demodulator B: signal out of range";
    case 0x610D: return "Handshake not received by safeUC";
    case 0x610E: return "Transducer signal frequency out of range";
    case 0x6108: return "Safety demodulator A: signal out of range";
    case 0x6109: return "Safety demodulator B: signal out of range";
    case 0x610F: return "Safety-controller PSU out of range";
    case 0x6110: return "Safety-controller voltage reference out of range";
    case 0x6101: return "Safety-controller fuse bit fault";
    case 0x610A: return "Safety-controller spool position cross validation fault";
    case 0x6111: return "Safety switch state fault (cannot perform safe operation)";
    case 0x6211: return "Safety-controller initialization fault";
    case 0x6112: return "Safety switch status fault (cannot perform safe operation)";
    case 0x6205: return "Handshake not received by mainUC";
    case 0x6113: return "Handshake bootup fault";
    case 0x620C: return "POST fault (power-on self-test failed)";
    case 0x6114: return "Safety controller task scheduling fault";
    case 0x8006: return "Spool position cross validation fault";
    case 0x5511: return "Memory (RAM) corrupted";
    case 0x6322: return "Memory (EEPROM) invalid parameter";
    case 0x5521: return "Memory (Flash) corrupted";
    case 0x5002: return "SPI communication fault";
    case 0xFF06: return "Fault overload (>3 faults simultaneously)";
    case 0x6323: return "PWM calibration fault";
    case 0x5532: return "Memory (EEPROM) communication fault";
    case 0x6209: return "PSM operation fault";
    case 0x5533: return "Config sector CRC fault";
    case 0x5536: return "Diagnostic sector CRC fault";
    case 0x620A: return "PSM buffer overload";
    case 0x4227: return "Average operating temperature above limit";
    case 0x4224: return "Current temperature above upper limit";
    case 0x4225: return "Current temperature below lower limit";
    case 0x8310: return "Main spool cannot return to neutral";
    case 0x8311: return "Float not reached";
    case 0x8307: return "Main spool not in neutral at bootup";
    case 0x8312: return "Actual main spool position exceeds set point";
    case 0x5237: return "Transducer signal frequency out of range";
    case 0x620F: return "SPI buffer overload";
    case 0x6210: return "SPI communication fault";
    case 0x8140: return "Loss and recovery of CAN bus connection";
    case 0x8003: return "Flow command not received within timeout period";
    case 0x6212: return "Safety switch status fault";
    case 0x8313: return "Float threshold setpoint not given";
    case 0x6213: return "Solenoid driver validation fault";
    case 0x620D: return "Stack usage > 90%";
    case 0x5537: return "CRC fault (data change not approved)";
    case 0x5001: return "Invalid hardware version";
    case 0x8009: return "COMM: running number validation fault";
    case 0x8008: return "Corrupted data received by Inlet actuator";
    case 0x8001: return "TPDO from Work Function actuator not received in time";
    case 0x8211: return "RPDO received invalid";
    case 0x81FF: return "RPDO not received within timeout period";
    case 0x9001: return "EMCY consumer received EMCY message from master";
    default: return nullptr;
  }
}

/// Severity level from the EMCY manufacturer field (msef[4], = frame byte 7).
inline const char * pved_severity_name(uint8_t severity)
{
  switch (severity)
  {
    case 0x00: return "Info";
    case 0x10: return "Warning";
    case 0x20: return "Critical";
    case 0x30: return "Severe";
    default: return "Unknown";
  }
}

}  // namespace ros2_canopen

#endif  // CANOPEN_408_DRIVER__PVED_FAULTS_HPP_
