---
name: cia-profiles-reference
description: CANopen device-profile reference covering CiA 301 (communication), CiA 402 (drives & motion — already implemented), CiA 410 (inclinometers), and CiA 408 (fluid power technology / hydraulics, used by Danfoss). Object-dictionary indices, state machines, default PDO mappings, scaling conventions. Invoke when writing/debugging code that touches profile-specific OD entries or PDO mappings.
---

# CANopen device-profile reference

Every CANopen device uses **CiA 301** as the communication-layer base. On top of that, a *device profile* (CiA 4xx) standardizes the application-layer object dictionary indices for a specific device class. The ROS 2 driver layer cares about both the comm profile and the device profile.

This reference focuses on the four profiles relevant to this repo:

- **CiA 301**: Application layer & comms profile (always present)
- **CiA 402**: Drives & motion (existing `canopen_402_driver`)
- **CiA 410**: Inclinometers (new — `canopen_410_driver`)
- **CiA 408**: Fluid power technology — proportional valves, hydrostatic transmissions (new — `canopen_408_driver`, Danfoss-targeted)

Indices below use standard CANopen hex notation `0xXXXX:sub`.

---

## CiA 301 — communication profile area (0x1000–0x1FFF)

Universal across all CANopen devices. Worth knowing when scaffolding YAML:

| Index | Object | Notes |
|---|---|---|
| 0x1000 | Device type | High word = CiA profile number (0x0192 = 402, 0x019D = 410, 0x0198 = 408). Drivers can check this in `init()` to verify they're talking to the right device. |
| 0x1001 | Error register | uint8 — bit set on any active error |
| 0x1003:n | Predefined error field | History of EMCY codes |
| 0x1005 | COB-ID SYNC | |
| 0x1006 | Communication cycle period | µs |
| 0x1017 | Producer heartbeat time | ms |
| 0x1018 | Identity object (vendor/product/rev/serial) | sub1=vendor id, sub2=product code, sub3=rev, sub4=serial |
| 0x1280–0x12FF | SDO client parameters | |
| 0x1400–0x15FF | RPDO comm parameters (sub1=COB-ID, sub2=transmission type) |
| 0x1600–0x17FF | RPDO mapping (sub1..n = packed (idx<<16)|(sub<<8)|bitlen) |
| 0x1800–0x19FF | TPDO comm parameters |
| 0x1A00–0x1BFF | TPDO mapping |

Transmission types (in 0x14xx/0x18xx:2):
- `0x00`: Synchronous, acyclic
- `0x01`–`0xF0`: Synchronous, cyclic (every N SYNCs)
- `0xFE`: Async, manufacturer-specific event
- `0xFF`: Async, device-profile event

The bus.yml `transmission:` key maps directly here.

### CiA 301 state machine (NMT)

`Pre-Operational → Operational → Stopped`, plus `Initialisation` on boot. Lely + `NodeCanopenBaseDriver` handles the NMT transitions; the proxy driver exposes `~/nmt_state` (pub) and `~/nmt_reset_node` / `~/nmt_start_node` (services). RPDOs/TPDOs only flow in **Operational**.

---

## CiA 402 — drives and motion control

Reference indices (existing implementation lives in `canopen_402_driver`):

| Index | Object | Type | Notes |
|---|---|---|---|
| 0x6040 | Controlword | uint16 | drives the state machine (bits 0..3 + reset, halt, op-mode-specific bits) |
| 0x6041 | Statusword | uint16 | mirrors device state |
| 0x6060 | Mode of operation | int8 | 1=PP, 3=PV, 4=PT, 6=Homing, 7=IP, 8=CSP, 9=CSV, 10=CST |
| 0x6061 | Mode of operation display | int8 | read-only echo |
| 0x6064 | Position actual value | int32 | inc |
| 0x606C | Velocity actual value | int32 | inc/s |
| 0x6071 | Target torque | int16 | per-mille of rated |
| 0x607A | Target position | int32 | |
| 0x6081 | Profile velocity | uint32 | |
| 0x6083 | Profile acceleration | uint32 | |
| 0x60C2:1/2 | Interpolation time period (value + base-10 exponent) | uint8 / int8 | required for CSP / IP |
| 0x60FF | Target velocity | int32 | |
| 0x60B0/0x60B1/0x60B2 | Position/velocity/torque offset | additive feed-forward |

Multi-axis devices use channel offset `0x800` per axis (so controlword for axis 2 is `0x6840`). `Motor402` applies this via `get_channel_index(base_index)`.

### CiA 402 state machine (controlword → statusword)

```
        Not-Ready-To-Switch-On
                 │
                 ▼  (auto)
        Switch-On-Disabled  ◀──  Fault-Reaction-Active ◀── Fault
                 │                                          ▲
                 │ "shutdown" (0x06)                        │
                 ▼                                          │
        Ready-To-Switch-On                                  │
                 │  "switch on" (0x07)                      │
                 ▼                                          │
        Switched-On                                         │
                 │  "enable operation" (0x0F)               │
                 ▼                                          │
        Operation-Enable  ──"quickstop" (0x02)──▶  Quick-Stop-Active
```

Statusword bits: 0=RTSO, 1=SwitchedOn, 2=OpEnabled, 3=Fault, 4=VoltageEnabled, 5=QuickStop(active-low), 6=SwitchOnDisabled, 10=TargetReached.

---

## CiA 410 — inclinometers

CiA 410 standardizes single- or dual-axis inclinometers (devices that report tilt angles around 1–2 axes). The device profile is mostly **read-only**: the master typically just receives an angle via TPDO. Some devices accept a zero-set command.

### Object dictionary (CiA 410)

| Index | Object | Type | Notes |
|---|---|---|---|
| 0x1000 | Device type | uint32 | High word = `0x019D` (= 413? — verify; CiA published 410 with `0x019A` in DSP-410. **Confirm against the EDS file the user supplies**, do not hardcode without checking 0x1000 at boot.) |
| 0x6000 | Slope long 16 | int16 | Primary tilt around long axis (X). Unit: 0.01° per LSB (standard). |
| 0x6010 | Slope long 32 | int32 | High-resolution version (some devices implement only one of 6000/6010). |
| 0x6020 | Slope long operating parameter | uint8 | Bit field — direction reversal, filter enable, etc. |
| 0x6030 | Slope long preset value | int16/int32 | Writable — used to zero the axis at the current position. |
| 0x6040 | Slope long offset | int16/int32 | Writable — additive offset. |
| 0x6100 | Slope lateral 16 | int16 | Tilt around lateral axis (Y). Same units. |
| 0x6110 | Slope lateral 32 | int32 | |
| 0x6120 | Slope lateral operating parameter | uint8 | |
| 0x6130 | Slope lateral preset value | int16/int32 | |
| 0x6140 | Slope lateral offset | int16/int32 | |
| 0x6200 | Resolution | uint16 | Inverse-LSB factor (e.g. 100 means 0.01° per LSB). |

**Note: 0x6040 in CiA 410 is "slope long offset", NOT the controlword.** This is the most important difference from 402 to remember — never confuse them. Any code that does `controlword |= ...` belongs to 402, not 410.

### Default PDO mapping (CiA 410)

A typical TPDO1 mapping is just:
```
{index: 0x6010, sub_index: 0}   # slope long 32-bit
{index: 0x6110, sub_index: 0}   # slope lateral 32-bit
```
Transmission type is usually `0xFF` (event-driven on change-of-state above a threshold) or `0x01` (sync-cyclic).

### Driver implications

- No 402-style state machine. The "device-logic class" (analog to `Motor402`) is much simpler — call it something like `Inclinometer410`.
- Two read-only "channels" per device (long + lateral); single-axis devices populate only one. Make this configurable via `num_axes` or two-axis-by-default.
- Useful ROS topic: `sensor_msgs/msg/Imu` with only orientation (roll+pitch from the two slopes, yaw=0, covariances marked unknown), **or** a plain `geometry_msgs/msg/Vector3Stamped` of `(roll, pitch, 0)` in radians.
- Convert internal hundredths-of-a-degree to radians: `radians = raw_int32 * resolution_factor * π/180`. Read 0x6200 to get the resolution at configure time; fall back to 100 (= 0.01° per LSB) if missing.
- Optional services: `~/zero_long`, `~/zero_lateral` writing to 0x6030 / 0x6130; `~/set_offset` writing to 0x6040 / 0x6140.

---

## CiA 408 — fluid power technology

CiA 408 covers proportional valves, hydrostatic transmissions, and pumps. Danfoss PVED-CL/CLS, PVG-32 EX, Plus+1 controllers etc. implement this profile. It is **very similar in shape to CiA 402** (controlword + statusword + mode + target/actual pairs), but the OD lives in different indices.

### Key object dictionary entries (CiA 408)

| Index | Object | Type | Notes |
|---|---|---|---|
| 0x1000 | Device type | uint32 | High word = `0x0198` |
| 0x6040 | Controlword | uint16 | **Same name and semantics as 402** — bits 0..3 drive the FSM, bit 7 fault reset, bit 8 halt. Reused by CiA 408. |
| 0x6041 | Statusword | uint16 | Same bit-field convention as 402 |
| 0x6042 | Device mode | uint8 | Profile-specific: 0=No mode, 1=Manual, 2=Auto/Position, 3=Pressure, 4=Flow, 5=Speed, ... (vendor-extensible) |
| 0x6043 | Device mode display | uint8 | Read-only echo of 0x6042 |
| 0x6044 | Device error | uint16 | Profile-level error register (separate from 0x1001) |
| 0x6300 | Target value (setpoint) | int32 | Profile setpoint — interpreted per mode (position / pressure / flow) |
| 0x6301 | Actual value (feedback) | int32 | Profile actual — sensor feedback for the active mode |
| 0x6302 | Min limit | int32 | |
| 0x6303 | Max limit | int32 | |
| 0x6304 | Acceleration / ramp up | uint32 | |
| 0x6305 | Deceleration / ramp down | uint32 | |
| 0x6306 | Manual command | int32 | Vendor-defined manual jog |
| 0x6310 | Target pressure | int32 | per-mode setpoint variants (vendor-extended on Danfoss) |
| 0x6311 | Actual pressure | int32 | |
| 0x6320 | Target flow | int32 | |
| 0x6321 | Actual flow | int32 | |

**Caveat: CiA 408 published indices vary between revisions and vendors. Danfoss in particular layers manufacturer-specific entries on top (range 0x2000–0x5FFF). Pull the actual indices from the device EDS once the user provides it.** The above is the canonical baseline; use it for scaffolding placeholders and lock the indices down before shipping.

### CiA 408 state machine

Effectively identical to CiA 402 (same controlword/statusword bit layout). The same `State402` enum + transition logic can be reused — see `canopen_402_driver/include/canopen_402_driver/state.hpp`. If you build the 408 driver, either depend on the 402 state machine code or copy/rename it (`State408`) to keep packages independent. Either is fine; the depend-on-402-for-state-only approach saves duplication but couples the packages — easier to copy if Danfoss adds quirks.

### Driver implications

- The 408 driver looks very much like the 402 driver but with a much smaller mode set (no homing, no interpolation, just manual / setpoint / feedback). The 402 polling-cycle scaffolding (`handleRead`/`handleWrite` per cycle, `registerDefaultModes`) is the right shape.
- Standard ROS API: `~/set_target` (COTargetDouble), `~/enable`, `~/disable`, `~/halt`, `~/recover` services + a publisher (`std_msgs/Float64` or a custom `Setpoint` msg) for actual feedback.
- Scaling: like 402, expose `scale_target_to_dev`, `scale_actual_from_dev` knobs in bus.yml.
- Mode selection: a service `~/set_mode` (taking a uint8) writing to 0x6042, or fixed per-instance from bus.yml.

### Danfoss-specific notes

- Danfoss controllers (Plus+1, PVED-CLS) layer their own profile on top of CiA 408. Many indices in the `0x2000–0x5FFF` range are vendor-specific (sensor scaling, PVED valve curves, calibration). Treat these as opaque SDO entries the user provides via `sdo:` in bus.yml.
- Danfoss PVED-CLS uses node-id 1 conventionally with a hardware coding; the master must support hardware-managed node-ids.
- Heartbeat (0x1017) must usually be set explicitly — Danfoss devices default to 0 (off) and the master times them out unless this is configured at boot via the `sdo:` section.

---

## Verifying against an EDS file

When the user provides the actual EDS, before shipping code:

1. `grep -i '^\[6...' device.eds` — list profile object entries actually implemented.
2. Compare against the table above; flag any **missing** entries the driver depends on.
3. Check the `[1000]` and `[1018]` sections to verify device type and vendor.
4. Look for `ParameterValue=` lines — these are the device's defaults; some drivers need to know them.
5. For CiA 408 Danfoss: skim `[2...]`–`[5...]` sections for vendor extensions worth exposing as bus.yml knobs.

## Related skills

- [[ros2-canopen-architecture]] — surrounding stack.
- [[ros2-canopen-new-driver]] — scaffolding recipe.
