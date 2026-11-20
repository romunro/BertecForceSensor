# BertecForceSensor (AM6817-E C++ Driver & `mc_rtc` Plugin)

Direct Linux C++17 driver and `mc_rtc` `GlobalPlugin` for the **Bertec AM6817-E** external digital force-plate amplifier (e.g., paired with a Bertec `FP4060-05-PT-1000` force plate) over USB (`/dev/ttyUSB*`), with no Windows SDK or `pyserial` dependency.

Works with any `mc_rtc` runner (`mc_rtc_ticker`, `mc_mujoco`, `kortex`, etc.) and automatically logs wrenches to the `mc_rtc` binary log.

---

## Maximum Polling / Streaming Rate & End-to-End Delay Budget

- **Native ADC & USB streaming rate:** **1000 Hz** (measured **~1002 Hz** on the AM6817-E internal clock).
- The amplifier streams unsolicited 1 kHz frames once started (`0x06 0x00`), and the plugin drains all available kernel buffer bytes on a dedicated background thread (`SCHED_FIFO` real-time priority when permitted) so `mc_rtc`'s real-time loop (`before()`) never blocks on serial I/O and always receives the **freshest** 1 kHz frame.

### Stage-by-Stage Closed-Loop Delay Budget (`FP4060-05-PT-1000` + `AM6817-E`)

| Stage | Component | Delay Contribution | Details / Live Measurement |
| :--- | :--- | :--- | :--- |
| **1. Mechanical Plate** | **FP4060-05-PT-1000** ($400 \times 600\text{ mm}$ strain-gauge plate, $f_n \approx 500\text{--}1000\text{ Hz}$) | **~0.2 – 0.5 ms** | Structural elastic wave & strain-gauge bridge response |
| **2. Amplifier Anti-Aliasing & ADC** | **AM6817-E** analog anti-aliasing filter (~500 Hz cutoff) + $1000\text{ Hz}$ synchronous ADC | **~0.8 – 1.5 ms** | Group delay of internal analog LPF + $0\text{--}1.0\text{ ms}$ ADC sample hold |
| **3. UART Serialization** | **18–19 bytes @ 1,500,000 baud (8N1)** ($10\text{ bits/byte}$) | **0.120 – 0.127 ms** | $\frac{18 \times 10}{1.5 \times 10^6} = 120\,\mu\text{s}$ wire transmission time |
| **4. FTDI USB Bulk Transfer** | **FT232R (`0403:6001`)** + Linux `ftdi_sio` (`latency_timer = 1 ms`) | **0.5 – 2.0 ms** *(measured)* | Live 1000-frame benchmark on `/dev/ttyUSB0`:<br>• Median inter-frame $\Delta t$: **`1.000 ms`**<br>• 95th percentile: **`1.018 ms`**<br>• 99th percentile / max: **`2.004 – 2.008 ms`**<br>*(If `latency_timer = 16 ms`, this stage jumps to **1.0 – 4.0 ms**)* |
| **5. C++ Driver + `mc_rtc` Handoff** | Non-blocking drain + HDLC unescape + 6×8 matrix multiply + ZOH to next `before()` tick | **0.05 ms + $[0, \Delta t_{\text{ctl}}]$** | • Reader thread decode: **$< 50\,\mu\text{s}$**<br>• Asynchronous clock phase difference to 1 kHz `mc_rtc` tick: **$0\text{ to }1.0\text{ ms}$** (avg **$0.5\text{ ms}$**) |

- **Total End-to-End Closed-Loop Delay (`latency_timer = 1 ms`, 1 kHz control loop):**
  - **Typical (mean):** **~2.5 – 3.0 ms**
  - **Worst-case (99th percentile):** **~4.0 – 4.8 ms**

### Closed-Loop Hardening Features (No Signal Filtering Applied)

To keep the raw dynamics untouched for your controller's own filtering while preventing closed-loop instabilities:
1. **Stale-Data Watchdog (`watchdog_timeout_ms: 20.0`, `zero_on_stale: true`):** Every sample is timestamped with `std::chrono::steady_clock`. If the USB cable disconnects or the stream stalls past `watchdog_timeout_ms`, `BertecForceSensor::stale` becomes `true`, `BertecForceSensor::connected` becomes `false`, and the output wrench is zeroed (when `zero_on_stale: true`) so the controller never integrates a frozen phantom force.
2. **Non-Blocking Buffer Drain (Zero Backlog Lag):** On every wakeup, the reader thread drains all bytes currently available in the kernel tty buffer and publishes the newest complete frame, preventing stale backlog accumulation after OS scheduling jitter.
3. **Wire-Corruption Sanity Guard (`max_range_multiplier: 3.0`):** Because the AM6817-E protocol has no hardware CRC byte, frames whose calibrated wrench exceeds `3.0 × [Ranges]` (or contains NaN/Inf from a UART bit flip) are rejected and counted in `<sensor_name>_corrupted_frames`.
4. **Glitch-Free Thread-Safe Tare:** All tare state and baseline updates happen under a single mutex lock per batch, and wrench output is held at zero while a live tare is accumulating to avoid a step discontinuity.

---

## Hardware & Protocol Summary

- **USB device:** `0403:6001` FTDI FT232 (`Bertec Corporation AM6817-E`) -> `/dev/ttyUSB0`
- **Serial settings:** `1,500,000` baud, `8N1`, raw mode
- **Commands:**
  - `0x01`: Stop stream / ping (`ACK 0x01`)
  - `0x02`: Query device info (firmware, EEPROM size, `1000 Hz`, `8` ADC channels, serial)
  - `0x03`: Read 1024-byte calibration EEPROM (6×8 calibration matrix, `[Ranges]`, `[Nameplate]`)
  - `0x06 0x00`: Start 1 kHz stream (`0x7E`-delimited HDLC frames with `0x7D` / `XOR 0x20` byte-stuffing, 1-byte `seq` counter, and `8 × uint16` LE ADC counts)

### USB Permissions & Low-Latency Mode (udev)

Ensure your user is in the `dialout` group so `/dev/ttyUSB0` is accessible without `sudo`:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for group membership to take effect
```

#### FTDI `latency_timer` (Important for 1 kHz closed-loop control)

Linux's `ftdi_sio` driver defaults to `latency_timer = 16 ms`. At 1000 Hz (18 bytes/frame), the FTDI chip flushes its 64-byte USB buffer every ~3–4 frames, so **average throughput is 1000 Hz with 0 drops, but frames arrive in ~3–4 ms micro-batches**.

`BertecAM6817::open()` automatically attempts to enable `ASYNC_LOW_LATENCY` (`1 ms`) via `TIOCSSERIAL` and sysfs, and logs a warning if `latency_timer` remains `> 1 ms`. To make `1 ms` latency persistent without `sudo` at runtime, add a `udev` rule:

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTR{latency_timer}="1"' | \
  sudo tee /etc/udev/rules.d/99-bertec-ftdi-latency.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Or set it temporarily for the current session:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
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
  GITHUB romunro/BertecForceSensor
  GIT_TAG origin/main
  DEPENDS mc_rtc
)
```

*(Or use `GITHUB_PRIVATE` if the repository is private, or `GIT_REPOSITORY <git-url>` if hosted elsewhere.)*

Then build/update via the superbuild:

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
watchdog_timeout_ms: 20.0       # marks stream stale if no frame arrives within 20 ms
zero_on_stale: true             # zeros wrench on timeout/disconnect for closed-loop safety
max_range_multiplier: 3.0       # rejects single-frame UART bit-flip spikes > 3x rated range
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
- `BertecForcePlate_corrupted_frames` (`uint64_t`)
- `BertecForcePlate_age_ms` (`double` — age of the latest sample in milliseconds)
- `BertecForcePlate_stale` (`bool` — `true` if `age_ms > watchdog_timeout_ms`)
- `BertecForcePlate_raw` (`Eigen::VectorXd` of 8 raw ADC counts, if `log_raw: true`)
