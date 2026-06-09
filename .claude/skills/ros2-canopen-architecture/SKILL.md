---
name: ros2-canopen-architecture
description: Architectural reference for the ros2_canopen stack — package layout, driver class hierarchy (BaseDriver/ProxyDriver/Cia402Driver), lifecycle pattern, Lely integration (LelyDriverBridge, PDOs, SDOs), YAML bus configuration, fake slaves, and test layout. Invoke when modifying any driver package, adding a new CiA-profile driver, debugging boot/lifecycle issues, or wiring up bus YAML.
---

# ros2_canopen architecture

The stack wraps the Lely CANopen C++ stack (`lely_core_libraries`) in a layered ROS 2 driver framework. Every device-specific driver in this repo extends the same templated base hierarchy and is loaded as an `rclcpp_components::Component` by a `DeviceContainer` that parses a YAML bus configuration.

## Top-level packages (workspace root)

| Package | Role |
|---|---|
| `lely_core_libraries` | Vendored / packaged Lely CANopen stack (C++17). Provides `lely::canopen::AsyncMaster`, `lely::canopen::BasicSlave`, `lely::CODev`, fiber drivers, etc. |
| `canopen_interfaces` | ROS 2 msgs/srvs: `COData.msg`, `CORead`/`COWrite`/`COReadID`/`COWriteID`/`COTargetDouble`/`CONode`/`CONmtID`/`COHeartbeatID` srvs. |
| `canopen_core` | `DeviceContainer`, `ConfigurationManager` (parses bus YAML), `LifecycleManager`, `MasterNode`, `CanopenDriver`/`LifecycleCanopenDriver` base classes, `NodeCanopenDriver<NODETYPE>` interface, `COData`/`COEmcy` exchange types, `SafeQueue`. |
| `canopen_master_driver` | Implements `MasterDriver` / `LifecycleMasterDriver` rclcpp components (registered as `ros2_canopen::MasterDriver`). |
| `canopen_base_driver` | `NodeCanopenBaseDriver<NODETYPE>` adds the `LelyDriverBridge`, RPDO/NMT/EMCY listener threads, diagnostic updater, `poll_timer_`. Not normally used as a top-level driver. |
| `canopen_proxy_driver` | `NodeCanopenProxyDriver<NODETYPE>` adds ROS topics/services that expose the CANopen API generically: `~/nmt_state` pub, `~/rpdo` pub, `~/tpdo` sub, `~/nmt_reset_node` / `~/nmt_start_node` / `~/sdo_read` / `~/sdo_write` services. Concrete `ProxyDriver` / `LifecycleProxyDriver` components. |
| `canopen_402_driver` | CiA 402 (motion controllers). `NodeCanopen402Driver<NODETYPE>` + `Motor402` device-logic class + `Cia402Driver` / `LifecycleCia402Driver` components. **The reference implementation for any new device-profile driver.** |
| `canopen_fake_slaves` | Standalone executables that talk to vCAN as a slave for integration tests. `SimpleSlave` (echo) and `CIA402MockSlave` (full CiA 402 state machine + mode threads). |
| `canopen_ros2_control` / `canopen_ros2_controllers` | ros2_control hardware interfaces & controllers wrapping the drivers (out of scope unless ros2_control integration is requested). |
| `canopen_tests` | Sample `bus.yml` configs (per-profile subdirs in `config/`) + launch files. The canonical place to add an integration example for a new driver. |
| `canopen_utils` | Misc helpers + a tiny package that the buildfarm pings. |
| `canopen` | Meta-package — `<exec_depend>` listing the packages that should be installed together. Update when adding a new driver. |

## Class hierarchy

Each device driver is a pair: a thin `rclcpp::Node` (or `rclcpp_lifecycle::LifecycleNode`) wrapper that delegates to a templated `NodeCanopenXXXDriver<NODETYPE>` implementation. The template lets the same implementation work with both flavors.

```
                  CanopenDriverInterface  (canopen_core/driver_node.hpp)
                  /                    \
        CanopenDriver               LifecycleCanopenDriver        ← rclcpp::Node / LifecycleNode wrappers
              \                          /
               (own a shared_ptr to a NodeCanopenDriverInterface)
                            │
              NodeCanopenDriver<NODETYPE>            (canopen_core)
                            │
              NodeCanopenBaseDriver<NODETYPE>        (canopen_base_driver)
                            │
              NodeCanopenProxyDriver<NODETYPE>       (canopen_proxy_driver)
                            │
              NodeCanopen402Driver<NODETYPE>         (canopen_402_driver)
                            │
              [your CiA-410 / CiA-408 driver here]
```

Wrapper component classes:

- `Cia402Driver`         → owns `NodeCanopen402Driver<rclcpp::Node>`
- `LifecycleCia402Driver`→ owns `NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>`

Both wrappers are registered with `RCLCPP_COMPONENTS_REGISTER_NODE(...)` and `rclcpp_components_register_nodes(...)` in `CMakeLists.txt`. The YAML field `driver: "ros2_canopen::Cia402Driver"` selects which class the `DeviceContainer` instantiates.

## NodeCanopen lifecycle (invariant across NODETYPE)

`NodeCanopenDriverInterface` defines this set of virtual hooks; each layer overrides what it needs and calls the parent class with `called_from_base=false`:

| Hook | Where the work happens |
|---|---|
| `init(bool)` | Construction-time setup. Base creates the diagnostic updater; Proxy creates the generic SDO/RPDO/NMT topics & services; 402 creates the `~/joint_states` publisher. |
| `configure(bool)` | Read YAML config into the driver. Multi-channel resolution, per-channel scale/offset, per-channel service creation, instantiate `LelyDriverBridge` (handled in base). |
| `activate(bool)` | Start the polling timer (`poll_timer_`), enable publishers, kick the state machine. 402 calls `Motor402::registerDefaultModes()`. |
| `deactivate(bool)` | Cancel `poll_timer_`, reset per-channel device objects. |
| `cleanup(bool)` / `shutdown(bool)` | Tear down the Lely driver, join listener threads. |
| `add_to_master()` | Called when the device gets assigned to the `AsyncMaster`. Instantiate device-specific objects (e.g. `Motor402`) that need `lely_driver_`. |
| `remove_from_master()` | The inverse. |
| `poll_timer_callback()` | Called every `period_ms_` while activated. Base reads queued NMT/RPDO/EMCY; 402 calls `motor->handleRead()` / `motor->handleWrite()` then `publish()` joint states. |

The two top-level wrappers (`CanopenDriver` and `LifecycleCanopenDriver`) bridge ROS 2 lifecycle transitions / construction to these hooks. For lifecycle: `on_configure` → `configure(true)` etc. For non-lifecycle: `init()` runs `init → configure → activate` in one shot.

## Lely integration (`LelyDriverBridge`)

`LelyDriverBridge` (in `canopen_base_driver/include/canopen_base_driver/lely_driver_bridge.hpp`) is a Lely `FiberDriver` subclass that lives inside an `lely::ev::Executor`. It is what every device-specific class actually talks to.

Important members:

```cpp
std::future<bool>   async_sdo_write(COData data);
std::future<COData> async_sdo_read(COData data);
template<class T> std::future<bool> async_sdo_write_typed(uint16_t idx, uint8_t subidx, T value);
template<class T> std::future<T>    async_sdo_read_typed(uint16_t idx, uint8_t subidx);
template<class T> T universal_get_value(uint16_t idx, uint8_t subidx);   // local OD cache
template<class T> void universal_set_value(uint16_t idx, uint8_t subidx, T value);
bool has_object(uint16_t idx, uint8_t subidx);
void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;        // called by lely for every received RPDO write
void OnSync(uint8_t cnt, const time_point & t) noexcept override;
// tpdo_mapped[idx][subidx] = value; tpdo_mapped[idx][subidx].WriteEvent(); → send RPDO to remote node
```

PDOs flow into the base driver via `OnRpdoWrite`, which pushes a `COData` onto `rpdo_queue_`; an internal `rdpo_listener` thread pops items and dispatches to `on_rpdo(COData)` (a virtual in `NodeCanopenBaseDriver` overridden by `ProxyDriver` and friends). To send a value to the remote node via PDO use `tpdo_mapped[idx][subidx] = value; WriteEvent();`.

SDO operations are synchronous from the caller's perspective even though they run on Lely's fiber executor: the proxy driver's `sdo_write(COData&)` posts an `async_sdo_write` task to `exec_` and waits on the future. Same for `sdo_read`.

## YAML bus configuration

Configuration is a YAML file (canonical name `bus.yml`). `ConfigurationManager` parses it; `DeviceContainer` walks the parsed tree and instantiates one component per node.

Skeleton:

```yaml
options:
  dcf_path: "@BUS_CONFIG_PATH@"   # replaced by canopen.launch.py with the actual share dir

master:
  node_id: 1
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
  sync_period: 10000              # microseconds

defaults:                         # applied to every node unless overridden
  dcf: "cia402_slave.eds"
  driver: "ros2_canopen::Cia402Driver"
  package: "canopen_402_driver"
  period: 10                      # poll_timer period in ms
  boot_timeout_ms: 1000
  sdo:                            # SDOs written once at boot
    - {index: 0x60C2, sub_index: 1, value: 50}
  tpdo:                           # PDOs the slave sends → master ("transmit" from device POV)
    1: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6041, sub_index: 0}, {index: 0x6061, sub_index: 0}]}
  rpdo:                           # PDOs the master sends → slave ("receive" from device POV)
    1: {enabled: true, cob_id: "auto",
        mapping: [{index: 0x6040, sub_index: 0}, {index: 0x6060, sub_index: 0}]}

nodes:
  cia402_device_1:
    node_id: 2
    namespace: "/test1"           # optional; per-node ROS namespace
    # Any driver-specific knob (e.g. scale_pos_to_dev, num_channels, channel_names) can be added here
```

The driver pulls extra YAML keys it understands from `this->config_["..."]` inside `configure()` — the structure is loosely typed (driver-specific keys are tolerated and accessed defensively with try/catch around `as<T>()`).

## canopen_interfaces messages & services

```
COData.msg          : { uint16 index, uint8 subindex, uint32 data }
CORead.srv          : index, subindex → data, success
COWrite.srv         : index, subindex, data → success
COReadID/COWriteID  : extended with node_id, error code
COTargetDouble.srv  : float64 target → bool success      (used by 402 ~/target service)
CONode.srv          : node_id  → success                 (NMT-level)
CONmtID.srv         : node_id, command → success
COHeartbeatID.srv   : node_id, heartbeat → success
```

Drivers can subscribe / publish `COData` (proxy driver does), or wrap them with stronger-typed messages (402 uses `sensor_msgs/JointState` for actual position/velocity/effort).

## Fake slaves

`canopen_fake_slaves` ships standalone executables that mimic a CANopen slave on vCAN. The pattern:

1. Subclass `canopen::BasicSlave` (Lely class) into a `XxxMockSlave`. Override `OnWrite(idx, subidx)` to react to RPDO/SDO writes from the master. Maintain internal device state in worker threads.
2. Wrap the mock slave in a small `BaseSlave`-derived ROS node (`XxxSlave`) that runs the Lely event loop in `run()` against `vcan0`.
3. Write a tiny `xxx_slave.cpp` `main()` that instantiates the wrapper node and spins it.
4. Ship a launch file that takes `node_id`, `node_name`, `slave_config` arguments.

The CIA 402 mock (`canopen_fake_slaves/include/canopen_fake_slaves/cia402_slave.hpp`) is the most thorough example — it implements the full CiA 402 state machine plus per-mode threads.

## Tests

- Component-load tests (`canopen_402_driver/test/test_driver_component.cpp`): instantiate the rclcpp component factory and verify it loads.
- Integration tests (`canopen_base_driver/test/test_node_canopen_base_driver_ros.cpp`): bring up master + driver + fake slave on vcan0, call services, verify behavior.
- Tests are gated behind `if(BUILD_TESTING)` in CMakeLists.

## Driver registration

Two things must happen for the YAML field `driver: "ros2_canopen::FooDriver"` to resolve to your class:

1. CMakeLists.txt:
   ```cmake
   rclcpp_components_register_nodes(your_lib "ros2_canopen::FooDriver")
   set(node_plugins "${node_plugins}ros2_canopen::FooDriver;$<TARGET_FILE:your_lib>\n")
   ```
2. Source file:
   ```cpp
   #include "rclcpp_components/register_node_macro.hpp"
   RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::FooDriver)
   ```

Both flavors (lifecycle and non-lifecycle) must be registered for the YAML to be able to select either.

## Key files to read when in doubt

- `canopen_core/include/canopen_core/driver_node.hpp` — top-level driver wrapper classes
- `canopen_core/include/canopen_core/node_interfaces/node_canopen_driver.hpp` — interface signatures
- `canopen_base_driver/include/canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp` — Lely bridge ownership, polling
- `canopen_base_driver/include/canopen_base_driver/lely_driver_bridge.hpp` — async_sdo_*, OnRpdoWrite, tpdo_mapped
- `canopen_proxy_driver/include/canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp` — generic ROS API
- `canopen_402_driver/include/canopen_402_driver/node_interfaces/node_canopen_402_driver_impl.hpp` — the reference template impl
- `canopen_tests/config/cia402/bus.yml` — canonical YAML example

## Related skills

- [[ros2-canopen-new-driver]] — step-by-step recipe to scaffold a new device-profile driver.
- [[cia-profiles-reference]] — CiA 301 / 402 / 410 / 408 object dictionary indices and state machines.
