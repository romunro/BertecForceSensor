# BertecForceSensor (AM6817-E C++ Driver & `mc_rtc` Plugin)

Direct Linux C++17 driver and `mc_rtc` `GlobalPlugin` for the **Bertec AM6817-E** external digital force-plate amplifier (e.g., paired with a Bertec `FP4060-05-PT-1000` force plate) over USB (`/dev/ttyUSB*`), with no Windows SDK or `pyserial` dependency.

Works with any `mc_rtc` runner (`mc_rtc_ticker`, `mc_mujoco`, `kortex`, etc.) and automatically logs wrenches to the `mc_rtc` binary log.

---

## Maximum Polling / Streaming Rate

- **Native ADC & USB streaming rate:** **1000 Hz** (measured **~1002 Hz** on the AM6817-E internal clock).
- The amplifier streams unsolicited 1 kHz frames once started (`0x06 0x00`), and the plugin reads them on a dedicated background thread so `mc_rtc`'s real-time loop (`before()`) never blocks on serial I/O.
- If your `mc_rtc` controller runs at 1000 Hz (or 200–500 Hz), every tick receives the latest 1 kHz measurement.

---

## Hardware & Protocol Summary

- **USB device:** `0403:6001` FTDI FT232 (`Bertec Corporation AM6817-E`) -> `/dev/ttyUSB0`
- **Serial settings:** `1,500,000` baud, `8N1`, raw mode
- **Commands:**
  - `0x01`: Stop stream / ping (`ACK 0x01`)
  - `0x02`: Query device info (firmware, EEPROM size, `1000 Hz`, `8` ADC channels, serial)
  - `0x03`: Read 1024-byte calibration EEPROM (6×8 calibration matrix, `[Ranges]`, `[Nameplate]`)
  - `0x06 0x00`: Start 1 kHz stream (`0x7E`-delimited HDLC frames with `0x7D` / `XOR 0x20` byte-stuffing, 1-byte `seq` counter, and `8 × uint16` LE ADC counts)

### USB Permissions (udev)

Ensure your user is in the `dialout` group so `/dev/ttyUSB0` is accessible without `sudo`:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for group membership to take effect
```

---

## Repository Contents

- `include/bertec_am6817/BertecAM6817.h` & `src/BertecAM6817.cpp`: Standalone POSIX C++17 driver (`libbertec_am6817.so`)
- `src/BertecForceSensorPlugin.h` & `src/BertecForceSensorPlugin.cpp`: `mc_rtc` `GlobalPlugin` (`BertecForceSensor.so`)
- `src/cli_main.cpp`: Standalone C++ CLI test binary (`bertec_am6817_cli`)
- `etc/BertecForceSensor.in.yaml`: Default plugin configuration
- `bertec_am6817.py`: Standalone Python 3 prototype/diagnostic script (zero external dependencies)

---

## Building Standalone

```bash
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH=/home/ronan/Projects/install \
  -DCMAKE_INSTALL_PREFIX=/home/ronan/Projects/install
cmake --build build -j$(nproc)
cmake --install build
```

Test the standalone C++ CLI directly against the hardware:

```bash
./build/bertec_am6817_cli /dev/ttyUSB0 3.0
```

---

## Adding to `mc-rtc-superbuild`

Create a new extension file in `<superbuild>/extensions/bertec_force_sensor.cmake`:

```cmake
AddProject(bertec_force_sensor
  GITHUB_PRIVATE <your-org-or-user>/force_sensor
  GIT_TAG origin/main
  DEPENDS mc_rtc
)
```

*(Or use `GIT_REPOSITORY <git-url>` instead of `GITHUB_PRIVATE` if hosted outside GitHub or locally.)*

Then build/update via the superbuild:

```bash
cd <superbuild>/build
cmake ..
cmake --build . --target bertec_force_sensor
```

This will build and install `BertecForceSensor.so`, `libbertec_am6817.so`, and `etc/BertecForceSensor.yaml` into your superbuild `install/` prefix.

---

## Configuring `mc_rtc` (`mc_rtc_ticker`, `mc_mujoco`, `kortex`)

### 1. Enable the plugin globally

Add `BertecForceSensor` to your `~/.config/mc_rtc/mc_rtc.yaml` (or the YAML config passed to `mc_rtc_ticker` / `mc_mujoco` / `kortex`):

```yaml
Plugins:
  - BertecForceSensor
```

### 2. Configure the plugin

Copy or override `BertecForceSensor.yaml` in `~/.config/mc_rtc/plugins/BertecForceSensor.yaml`:

```yaml
port: /dev/ttyUSB0
sensor_name: BertecForcePlate
robot_name: ""                  # empty = main controller.robot()
inject_to_robot_sensor: true    # updates robot.forceSensor(sensor_name) if present
datastore_prefix: BertecForceSensor
tare_samples: 200               # averaged on startup (0 to skip)
log_raw: false                  # set true to also log 8 raw ADC channels
gui: true                       # adds Plugins -> BertecForceSensor tab + Tare button
```

Because `mc_rtc_ticker`, `mc_mujoco`, and `kortex` all instantiate `mc_control::MCGlobalController`, loading the plugin via `Plugins: [BertecForceSensor]` works identically across all three runners and automatically writes `<sensor_name>_wrench`, `<sensor_name>_sample_count`, and `<sensor_name>_dropped_frames` into the standard `mc_rtc` `.bin` log.

---

## Using in Your `mc_rtc` Controller Codebase

### Option A: Read from the Datastore (works with any robot model)

```cpp
// In your controller's run() or before()
if(datastore().has("BertecForceSensor::wrench"))
{
  const auto & wrench = datastore().get<sva::ForceVecd>("BertecForceSensor::wrench");
  const Eigen::Vector3d & force  = wrench.force();   // [Fx, Fy, Fz] in N
  const Eigen::Vector3d & moment = wrench.couple();  // [Mx, My, Mz] in Nm
}

// Trigger a non-blocking software tare at any time:
if(datastore().has("BertecForceSensor::tare"))
{
  datastore().call("BertecForceSensor::tare");
}
```

### Option B: Read as a Robot `ForceSensor`

If your robot module defines a `ForceSensor` named `BertecForcePlate` (matching `sensor_name` in `BertecForceSensor.yaml`) and `inject_to_robot_sensor: true`:

```cpp
const auto & wrench = robot().forceSensor("BertecForcePlate").wrench();
```

---

## Logged Signals

When `mc_rtc` logging is enabled (`Log: true` in `mc_rtc.yaml`), the plugin logs:

- `BertecForcePlate_wrench` (`sva::ForceVecd`: couple `[Mx, My, Mz]` Nm, force `[Fx, Fy, Fz]` N)
- `BertecForcePlate_sample_count` (`uint64_t`)
- `BertecForcePlate_dropped_frames` (`uint64_t`)
- `BertecForcePlate_raw` (`Eigen::VectorXd` of 8 raw ADC counts, if `log_raw: true`)
