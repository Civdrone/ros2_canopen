---
name: ros2-canopen-new-driver
description: Step-by-step recipe for adding a new CiA-profile driver package to ros2_canopen — package skeleton, templated NodeCanopenXxxDriver, lifecycle + non-lifecycle component wrappers, CMakeLists registration, fake slave, bus.yml example, and meta-package update. Invoke when scaffolding any new device-profile driver (e.g. canopen_410_driver, canopen_408_driver, canopen_4xx_driver).
---

# Adding a new device-profile driver

Mirror the layout of `canopen_402_driver`. The pattern: write **one** templated `NodeCanopenXxxDriver<NODETYPE>` that contains all the logic, then expose it as **two** rclcpp components (lifecycle and non-lifecycle) plus one device-logic class that talks to `LelyDriverBridge`.

See [[ros2-canopen-architecture]] for the surrounding context.

## 1. Directory skeleton

```
canopen_<profile>_driver/
├── include/canopen_<profile>_driver/
│   ├── <device>.hpp                              # device-logic class (analog to motor.hpp)
│   ├── <profile>_driver.hpp                      # non-lifecycle wrapper
│   ├── lifecycle_<profile>_driver.hpp            # lifecycle wrapper
│   ├── visibility_control.h                      # copy from canopen_402_driver and rename macros
│   └── node_interfaces/
│       ├── node_canopen_<profile>_driver.hpp     # templated class declaration
│       └── node_canopen_<profile>_driver_impl.hpp # templated definitions
├── src/
│   ├── <device>.cpp                              # device-logic .cpp (optional if header-only)
│   ├── <profile>_driver.cpp                      # 3-line component registration
│   ├── lifecycle_<profile>_driver.cpp            # 3-line component registration
│   └── node_interfaces/
│       └── node_canopen_<profile>_driver.cpp     # explicit template instantiation
├── test/
│   ├── CMakeLists.txt
│   └── test_driver_component.cpp                 # rclcpp_components factory load test
├── CMakeLists.txt
├── package.xml
└── CHANGELOG.rst                                  # empty / boilerplate is fine for a new package
```

## 2. `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>canopen_<profile>_driver</name>
  <version>0.0.0</version>
  <description>Driver for devices implementing CiA <NNN> profile</description>
  <maintainer email="...">...</maintainer>
  <license>LGPL-v3</license>

  <buildtool_depend>ament_cmake_ros</buildtool_depend>

  <depend>boost</depend>
  <depend>canopen_base_driver</depend>
  <depend>canopen_core</depend>
  <depend>canopen_interfaces</depend>
  <depend>canopen_proxy_driver</depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>rclcpp_lifecycle</depend>
  <!-- profile-specific message deps, e.g. sensor_msgs, geometry_msgs, std_msgs -->

  <test_depend>ament_lint_auto</test_depend>

  <export><build_type>ament_cmake</build_type></export>
</package>
```

## 3. `CMakeLists.txt` (mirror of canopen_402_driver)

Four libraries:

1. `<device>_bridge` — device-logic library (depends on `canopen_base_driver::node_canopen_base_driver`).
2. `node_canopen_<profile>_driver` — templated impl library (depends on `<device>_bridge` and `canopen_proxy_driver::node_canopen_proxy_driver`).
3. `lifecycle_<profile>_driver` — lifecycle component (depends on the impl + `canopen_core::node_canopen_driver` + `rclcpp_components::component` + `rclcpp_lifecycle::rclcpp_lifecycle`). Registers `ros2_canopen::Lifecycle<Profile>Driver`.
4. `<profile>_driver` — non-lifecycle component. Registers `ros2_canopen::<Profile>Driver`.

Both component libs must call:

```cmake
rclcpp_components_register_nodes(<lib> "ros2_canopen::<Class>")
set(node_plugins "${node_plugins}ros2_canopen::<Class>;$<TARGET_FILE:<lib>>\n")
```

…and install all four libraries via `EXPORT export_${PROJECT_NAME}` plus the include directory.

## 4. Templated node implementation

`include/canopen_<profile>_driver/node_interfaces/node_canopen_<profile>_driver.hpp`:

```cpp
#include "canopen_<profile>_driver/<device>.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"

namespace ros2_canopen::node_interfaces {
template <class NODETYPE>
class NodeCanopen<Profile>Driver : public NodeCanopenProxyDriver<NODETYPE> {
  static_assert(std::is_base_of_v<rclcpp::Node, NODETYPE>
             || std::is_base_of_v<rclcpp_lifecycle::LifecycleNode, NODETYPE>,
             "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");
protected:
  std::shared_ptr<<DeviceClass>> device_;
  // publishers, subscribers, services, scales, timers...

  virtual void poll_timer_callback() override;
  void publish();                          // optional — if you have outbound state
  void configure_common();                 // shared configure logic

public:
  explicit NodeCanopen<Profile>Driver(NODETYPE * node);
  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void add_to_master() override;
  // …per-channel getters / setters …
};
} // ns
```

`_impl.hpp` mirrors `node_canopen_402_driver_impl.hpp`:
- Template-specialize `init()` for `rclcpp::Node` and `rclcpp_lifecycle::LifecycleNode` to create the right kind of publisher.
- `configure()` calls `NodeCanopenProxyDriver<NODETYPE>::configure(false)` first, then reads `this->config_["<your_key>"].template as<T>()` defensively (try/catch each one).
- `activate()` calls the parent's `activate(false)`, then starts your timer/state-machine.
- `add_to_master()` calls parent's `add_to_master()`, then `device_ = std::make_shared<DeviceClass>(this->lely_driver_, ...)`.

The .cpp inside `src/node_interfaces/` does **only** the explicit instantiations:

```cpp
#include ".../node_canopen_<profile>_driver.hpp"
#include ".../node_canopen_<profile>_driver_impl.hpp"
template class ros2_canopen::node_interfaces::NodeCanopen<Profile>Driver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopen<Profile>Driver<rclcpp_lifecycle::LifecycleNode>;
```

## 5. Component wrappers

`include/.../<profile>_driver.hpp` (non-lifecycle):

```cpp
class <Profile>Driver : public ros2_canopen::CanopenDriver {
  std::shared_ptr<node_interfaces::NodeCanopen<Profile>Driver<rclcpp::Node>> impl_;
public:
  explicit <Profile>Driver(rclcpp::NodeOptions o = rclcpp::NodeOptions());
  // delegating wrappers for any public API you want exposed to callers that hold a CanopenDriver*
};
```

`src/<profile>_driver.cpp`:

```cpp
#include "canopen_<profile>_driver/<profile>_driver.hpp"
using namespace ros2_canopen;
<Profile>Driver::<Profile>Driver(rclcpp::NodeOptions o) : CanopenDriver(o) {
  impl_ = std::make_shared<node_interfaces::NodeCanopen<Profile>Driver<rclcpp::Node>>(this);
  node_canopen_driver_ = std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(impl_);
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::<Profile>Driver)
```

`lifecycle_<profile>_driver.hpp/.cpp` is the same shape but with `LifecycleCanopenDriver` and `rclcpp_lifecycle::LifecycleNode` as the NODETYPE.

## 6. Device-logic class

This is what `Motor402` is for the 402 driver. It owns a `std::shared_ptr<LelyDriverBridge>`, knows the profile's object dictionary indices, and exposes setter/getter functions to the node interface. Useful helpers from `LelyDriverBridge`:

```cpp
driver->async_sdo_write_typed<int32_t>(0x607A, 0, target).get();   // blocking SDO write
driver->async_sdo_read_typed<int32_t>(0x6064, 0).get();            // blocking SDO read
driver->universal_get_value<int16_t>(0x6041, 0);                   // local cached OD read (PDO-driven)
driver->universal_set_value<uint16_t>(0x6040, 0, controlword);     // local OD write + send via tpdo_mapped
driver->has_object(idx, sub);                                      // guard before reading
driver->tpdo_mapped[0x6040][0] = 0x000F;                           // raw RPDO write
driver->tpdo_mapped[0x6040][0].WriteEvent();
```

Inbound state from the slave (TPDO from device → mapped to RPDO from master's POV) shows up via `OnRpdoWrite()` and is queued into `rpdo_queue_`; override `on_rpdo(COData)` in your node interface if you need a callback per write, otherwise read the cached local OD with `universal_get_value`.

## 7. Fake slave

In `canopen_fake_slaves/include/canopen_fake_slaves/<profile>_slave.hpp`:

1. `class <Profile>MockSlave : public canopen::BasicSlave` — implement the profile state machine. Override `OnWrite(idx, subidx)` to react to the master writing your control object(s).
2. `class <Profile>Slave : public BaseSlave` — Lely event-loop wrapper. Copy the structure of `CIA402Slave::run()`.
3. `src/<profile>_slave.cpp` — `main()` that spins the wrapper.
4. Add `add_executable(<profile>_slave_node ...)` + `install(TARGETS ...)` in `canopen_fake_slaves/CMakeLists.txt`.
5. Optional: ship a launch file in `canopen_fake_slaves/launch/<profile>_slave.launch.py`.

## 8. Integration example

In `canopen_tests/config/<profile>/`:
- `bus.yml` — `driver: "ros2_canopen::<Profile>Driver"`, `package: "canopen_<profile>_driver"`, with the profile-appropriate TPDO/RPDO mappings.
- `<profile>_slave.eds` — minimal EDS for the fake slave (or pull from vendor).

In `canopen_tests/launch/<profile>_setup.launch.py` — mirror `cia402_setup.launch.py`.

## 9. Meta-package

Add `<exec_depend>canopen_<profile>_driver</exec_depend>` to `canopen/package.xml`.

## 10. Tests

Minimum: a component-load test under `test/test_driver_component.cpp` that instantiates the rclcpp components factory for both the lifecycle and non-lifecycle classes. Copy from `canopen_402_driver/test/`.

## Smell checks before declaring done

- [ ] YAML field `driver:` matches the exact registered class name `"ros2_canopen::<Profile>Driver"`.
- [ ] Both lifecycle and non-lifecycle components are registered (or document why only one).
- [ ] `target_compile_features(... PUBLIC c_std_99 cxx_std_17)` on every lib.
- [ ] `ament_export_libraries`, `ament_export_targets`, `ament_export_include_directories`, `ament_export_dependencies` all populated.
- [ ] Meta-package `canopen/package.xml` lists the new package.
- [ ] EDS file is installed via `install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)` in `canopen_tests` (or the package that owns it).
- [ ] The driver tolerates missing optional YAML keys (use try/catch around `as<T>()`).

## Related skills

- [[ros2-canopen-architecture]] — the surrounding architecture and conventions.
- [[cia-profiles-reference]] — CiA 410 / 408 object dictionary entries you'll need to bake in.
