# LEGO 42160 Drive-by-Wire Gateway

Turns the **LEGO Technic 42160 Audi RS Q e-tron** into a drive-by-wire platform controlled by **ArduPilot**.

A **Raspberry Pi Pico W** reads PWM outputs from an ArduPilot flight controller and drives the LEGO Technic Hub over BLE using the **LEGO Wireless Protocol 3 (LWP3)**. The gateway enforces safety gating, so the car can run under manual RC or a full autonomous stack.

```
[Matek F405 Mini / ArduPilot Rover]
        │ PWM: throttle, steering, enable (deadman)
        ▼
[Raspberry Pi Pico W]  ── PWM capture · safety gating · BLE central
        │ BLE / LWP3
        ▼
[LEGO Technic Hub]
   ├─ Port A/B: drive motors      (StartPower)
   └─ Port D:   steering motor    (GotoAbsolutePosition → servo-like, auto-centering)
```

## Contents

1. [Quick start checklist](#1-quick-start-checklist)
2. [Hardware and wiring](#2-hardware-and-wiring)
3. [Install the build tools](#3-install-the-build-tools)
4. [Set your hub's address](#4-set-your-hubs-address)
5. [Build the firmware](#5-build-the-firmware)
6. [Upload the firmware to the Pico W](#6-upload-the-firmware-to-the-pico-w)
7. [Watch the serial log](#7-watch-the-serial-log)
8. [Configure ArduPilot](#8-configure-ardupilot)
9. [First run and safety test](#9-first-run-and-safety-test)
10. [Steering calibration](#10-steering-calibration)
11. [Configuration reference](#11-configuration-reference)
12. [Safety behaviour](#12-safety-behaviour)
13. [Troubleshooting](#13-troubleshooting)
14. [Repository layout and history](#14-repository-layout-and-history)

---

## 1. Quick start checklist

If you have done this before and only need a reminder:

```bash
# one-time: tools + SDK (see section 3)
export PICO_SDK_PATH=~/pico/pico-sdk

# build
cmake -S . -B build
cmake --build build -j

# upload: hold BOOTSEL, plug in USB, copy the file to the RPI-RP2 drive
cp build/lego_dbw.uf2 /media/$USER/RPI-RP2/      # Linux
# or: picotool load -fx build/lego_dbw.uf2

# serial log
screen /dev/ttyACM0 115200                        # Ctrl-A then K to quit
```

Then: hub on → Pico on → wait for `[BLE] READY` → flip the enable switch → drive.

---

## 2. Hardware and wiring

| Item | Notes |
|---|---|
| LEGO 42160 + Technic Hub | Hub powered by its own AA pack |
| Matek F405 Mini | ArduPilot **Rover** firmware; powered separately |
| Raspberry Pi Pico W | Powered from USB or a 5 V rail (VSYS) |
| 3 signal wires + 1 GND wire | **Common ground between Matek and Pico is required** |
| Micro-USB cable | For uploading firmware and reading the log |

| Matek output | Signal | Pico pin |
|---|---|---|
| S1 | Throttle | GP2 (physical pin 4) |
| S2 | Steering | GP3 (physical pin 5) |
| S3 | Enable (deadman) | GP4 (physical pin 6) |
| GND | Ground | any GND pin (e.g. physical pin 3) |

There is no wired connection to the LEGO hub; the link is BLE only.

> The Pico's inputs are 3.3 V. Standard PWM outputs on F405 boards are 3.3 V, so a direct connection is fine. Do not connect a 5 V signal source.

---

## 3. Install the build tools

You need three things: **CMake**, the **ARM GCC cross-compiler**, and the **Pico SDK 2.2.x** (with its submodules). Pick one of the options below.

### Option A: VS Code + Raspberry Pi Pico extension (easiest, all OSes)

1. Install [VS Code](https://code.visualstudio.com/).
2. Install the extension **"Raspberry Pi Pico"** (publisher: Raspberry Pi).
3. Open this repository folder in VS Code. When the extension offers to import the project, choose **SDK version 2.2.0** and board **pico_w**.
4. The extension downloads the SDK, compiler, CMake, Ninja and picotool for you. Use its **Compile Project** and **Run Project** buttons instead of sections 5–6.

If you use this option, skip to [section 4](#4-set-your-hubs-address).

### Option B: Command line on Linux (Ubuntu / Debian / WSL2)

```bash
sudo apt update
sudo apt install -y git cmake build-essential python3 \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
    libusb-1.0-0-dev pkg-config
```

### Option B: Command line on macOS

```bash
brew install cmake libusb pkg-config
brew install --cask gcc-arm-embedded
```

### Get the Pico SDK (Option B only)

```bash
mkdir -p ~/pico && cd ~/pico
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init      # pulls btstack, cyw43-driver, tinyusb, lwip, ...
```

Tell CMake where the SDK is. Add this to `~/.bashrc` (or `~/.zshrc` on macOS) so you don't have to repeat it:

```bash
export PICO_SDK_PATH=~/pico/pico-sdk
```

> Missing submodules are the most common build failure. If you see `pico/stdio_usb.h: No such file` or `btstack.h: No such file`, run `git submodule update --init` inside the SDK folder again.

---

## 4. Set your hub's address

The firmware connects to **one specific hub**, identified by its Bluetooth MAC address. Edit this line near the top of `src/lego_dbw_system.c`:

```c
static const bd_addr_t HUB_ADDR = { 0x90, 0x84, 0x2B, 0x7F, 0xD4, 0x40 };   // 90:84:2B:7F:D4:40
```

If you are still using the same hub as before, leave it as is. To find the address of a different hub, use either method:

- **Phone app:** install **nRF Connect** (Android/iOS), turn the hub on (LED blinking), scan, and look for a device named **"Technic Hub"**. The address is shown under the name.
- **From the Pico itself:** set `#define DEBUG_PRINT_ADV_REPORTS 1`, build and upload, then read the `[ADV]` lines in the serial log. Set it back to `0` afterwards.

> **Close the LEGO Control+ / Powered Up app on your phone.** The hub accepts only one connection; if the phone is connected, the Pico can never reach it.

---

## 5. Build the firmware

From the repository folder:

```bash
cmake -S . -B build
cmake --build build -j
```

The first build takes several minutes: it also builds `picotool` and `pioasm` from source (this needs internet access and a normal host C/C++ compiler). Later builds take seconds.

The result is **`build/lego_dbw.uf2`**. This is the file you upload.

If you change only the `.c` file, re-running `cmake --build build -j` is enough. If you change `CMakeLists.txt`, or the build behaves strangely, delete the `build` folder and run both commands again.

Build notes:
- BLE only: the project defines `CYW43_LWIP=0`, so no lwIP/WiFi stack is pulled in.
- `config/btstack_config.h` enables `ENABLE_LE_PERIPHERAL` in addition to `ENABLE_LE_CENTRAL`. The peripheral role is unused, but BTstack's `hci.c` in SDK 2.2 does not compile without it.

---

## 6. Upload the firmware to the Pico W

### Method 1: Drag and drop (always works)

1. **Unplug** the Pico W from USB.
2. **Hold down the white BOOTSEL button** on the Pico W.
3. **Plug in** the USB cable while still holding the button. Release after one second.
4. A USB drive named **`RPI-RP2`** appears on your computer.
5. **Copy `build/lego_dbw.uf2` onto that drive.**
6. The drive disappears by itself and the Pico reboots into the new firmware. That is normal, not an error.

On WSL2, copy the file from Windows Explorer: the build folder is reachable at `\\wsl$\<distro>\home\<user>\...`.

### Method 2: picotool (no button press after the first upload)

Once this firmware is running, picotool can reboot the Pico into upload mode over USB by itself:

```bash
picotool load -fx build/lego_dbw.uf2
# -f: force the running Pico into BOOTSEL mode, -x: start the program after loading
```

`picotool` was built during the first build. If it is not on your PATH, find it under `build/_deps/picotool/`. On Linux, use `sudo` or install the udev rules from the picotool repository if you get a permission error.

### Method 3: VS Code extension

Click **Run Project** in the extension's side panel (Pico must be connected; it uses picotool).

---

## 7. Watch the serial log

The firmware prints its log over USB as a virtual serial port (any baud rate works; 115200 is conventional).

| OS | Port | Command |
|---|---|---|
| Linux / WSL | `/dev/ttyACM0` | `screen /dev/ttyACM0 115200` (quit: Ctrl-A, then K, then Y) or `minicom -D /dev/ttyACM0` |
| macOS | `/dev/tty.usbmodem*` | `screen /dev/tty.usbmodem* 115200` |
| Windows | `COMx` (see Device Manager) | PuTTY (Serial, COMx, 115200), or the **Serial Monitor** extension in VS Code |

After boot the firmware waits up to 5 seconds for a serial monitor before continuing, so open the monitor quickly if you want to see the first lines.

A healthy start looks like this:

```
=== LEGO DBW Pico W v2 ===
[BLE] scanning...
[BLE] hub seen rssi=-58 -> connect
[BLE] connecting to 90:84:2B:7F:D4:40...
[BLE] connected handle=0x0040 interval=24 x1.25ms
[GATT] discover all primary services...
[GATT] 4 services
[GATT] selected (LEGO UUID): write=0x000e (WO_RESP) notify=0x000e
[BLE] READY
[HUB] port 0x00 attached io_type=0x002e
[HUB] battery 87%
[DBG] ble=1 st=6 | thr=1500 str=1500 en=1900 | armed=1 | thr_cmd=0 steer=0mdeg | tx ok=12 drop=0
```

The exact numbers (handles, RSSI, io types) will differ. What matters is reaching `[BLE] READY`, followed by a `[DBG]` line every second.

How to read the `[DBG]` line:
- `ble=1` means connected and ready.
- `thr`, `str` and `en` are the measured pulse widths in µs. `(stale)` means no valid signal on that pin.
- `armed=1` means all three signals are fresh and enable is above 1600 µs.
- `tx ok` / `drop` count the commands accepted and rejected by the BLE stack.

---

## 8. Configure ArduPilot

The Pico expects three standard **50 Hz PWM** signals with 1000–2000 µs range and a 1500 µs neutral point. In Mission Planner (Config → Full Parameter List), on Rover firmware:

| Parameter | Value | Meaning |
|---|---|---|
| `SERVO1_FUNCTION` | `70` | Throttle → S1 → GP2 |
| `SERVO2_FUNCTION` | `26` | GroundSteering → S2 → GP3 |
| `SERVO3_FUNCTION` | `50 + N` | `RCPassThruN`: copies RC input channel N → S3 → GP4 (e.g. `55` = RCPassThru5) |
| `SERVO_RC_FS_MSK` | `4` (bit for output 3) | **Required.** On RC loss, output 3 goes to neutral instead of holding its last value |
| `SERVO1..2_MIN / TRIM / MAX` | `1000 / 1500 / 2000` | Matches the Pico's mapping |

Notes:
- Put the **enable** function on a two-position switch on your transmitter, then use that channel number for `SERVO3_FUNCTION`. Switch high (> 1600 µs) = drive allowed; switch low = stop and centre. This acts as a deadman, independent of ArduPilot's arming state.
- **Why `SERVO_RC_FS_MSK` matters:** a passthrough output normally keeps its last valid value when the RC link is lost. Without the mask, switching the transmitter off while the enable switch is high would leave the car armed. Test it: with enable high, turn the transmitter off and check that `en` in the `[DBG]` line drops below 1600 µs and `armed` goes to 0.
- Passthrough outputs copy the input pulse exactly; their `SERVO3_MIN/TRIM/MAX` have no effect.
- The outputs must be plain PWM, not DShot or OneShot.
- If outputs stay silent, check that the vehicle is armed and that the board safety switch is not blocking outputs (`BRD_SAFETY_DEFLT`).
- Verify the parameter values against the parameter list of your ArduPilot version.

---

## 9. First run and safety test

**Put the car on a stand with the wheels off the ground** for the first run after any firmware change.

1. **Centre the front wheels by hand**, then turn on the Technic Hub (see [section 10](#10-steering-calibration) for why). The hub LED blinks.
2. Power the flight controller and the Pico W. Open the serial log.
3. Wait for `[BLE] READY`. The hub LED changes from blinking to steady once the Pico connects.
4. Check that the `[DBG]` line shows sensible pulse widths for all three channels.
5. Flip the enable switch high → `armed=1`.
6. Test steering with small stick movements first, then throttle.

Then run these checks once:

| Test | Expected result |
|---|---|
| Flip the enable switch low while driving | Motors stop, steering returns to centre |
| Turn the RC transmitter off with enable high | `armed=0`: motors stop, steering centres (needs `SERVO_RC_FS_MSK`, section 8) |
| Unplug the S1 throttle wire | Within 250 ms: motors stop, steering centres |
| **Hold throttle, then unplug the Pico's power** | **Verify the hub stops the motors by itself.** The Pico cannot send a stop once the BLE link is gone. If the car keeps driving, treat this as a known hazard |
| Turn the hub off and on again | Log shows a disconnect, then `scanning`, then `READY` again without touching the Pico |

---

## 10. Steering calibration

**Centre position.** `GotoAbsolutePosition` measures angles from wherever the steering motor was **when the hub was switched on**. If the wheels were turned at power-on, "centre" will be off. Always centre the wheels by hand before switching the hub on. (A proper calibration routine is on the roadmap.)

**End-stop limit.** Driving the steering motor into the mechanical stop at high power draws a lot of current, which can make the hub's voltage dip and reset it. To find a safe limit:

1. Set `STEER_MOTOR_DEG_LIMIT` to a low value, for example `120`. Build and upload.
2. Push the steering stick fully to one side and watch whether the wheels reach full lock.
3. Raise the limit in steps of about 20° until they just reach full lock without the motor straining or buzzing.
4. Set the final value about 10% below that point.

`STEER_GEAR_RATIO` (6) converts wheel degrees to motor degrees; `STEER_TARGET_DEG_MAX` (40) is the wheel angle requested at full stick before the limit is applied.

---

## 11. Configuration reference

All tunables are at the top of `src/lego_dbw_system.c`. Rebuild and re-upload after changing any of them.

| Setting | Default | Meaning |
|---|---|---|
| `HUB_ADDR` | `90:84:2B:7F:D4:40` | MAC of **your** Technic Hub |
| `GPIO_PWM_THROTTLE / STEERING / ENABLE` | 2 / 3 / 4 | Input pins |
| `ENABLE_THRESHOLD_US` | 1600 | Enable channel must be above this to arm |
| `PWM_TIMEOUT_MS` | 250 | Signal age before a channel counts as lost |
| `PWM_DEADBAND_US` | 30 | Dead zone around 1500 µs |
| `THROTTLE_MAX_POWER` | 100 | Maximum drive power (0–100) |
| `STEER_TARGET_DEG_MAX` | 40 | Wheel-side angle at full stick |
| `STEER_GEAR_RATIO` | 6 | Motor degrees per wheel degree (measured ~6:1) |
| `STEER_MOTOR_DEG_LIMIT` | 240 | **Measure your end-stop, see section 10** |
| `STEER_GOTO_SPEED` | 80 | Steering speed (1–100) |
| `STEER_GOTO_MAXPOWER` | 80 | Lower = less stall current at the end-stop |
| `CONN_SUPERVISION_TO` | 200 (2 s) | BLE link supervision timeout |
| `DEBUG_PRINT_ADV_REPORTS` | 0 | Set to 1 to list every BLE device seen while scanning |

---

## 12. Safety behaviour

The gateway commands **throttle 0 and steering centre** whenever any of these is true:
- the enable PWM is below threshold or stale,
- the throttle or steering PWM is stale (> 250 ms),
- a pulse is outside 800–2200 µs (rejected as noise).

When the BLE link is down, the gateway cannot send anything. Behaviour then depends on the hub firmware (see the safety test in [section 9](#9-first-run-and-safety-test)).

A 1 s hardware watchdog reboots the Pico if its run loop stalls; the boot log reports watchdog resets.

---

## 13. Troubleshooting

### Upload and build

| Symptom | Fix |
|---|---|
| No `RPI-RP2` drive appears | Hold BOOTSEL *before* plugging in. Try another cable: many micro-USB cables are charge-only |
| `pico/stdio_usb.h` / `btstack.h` not found | SDK submodules missing: run `git submodule update --init` in the SDK folder |
| `PICO_SDK_PATH` not set | `export PICO_SDK_PATH=~/pico/pico-sdk`, then delete `build/` and run CMake again |
| No serial port after upload | Wait a few seconds; check Device Manager / `ls /dev/ttyACM*`. If still nothing, the firmware may have crashed: re-upload with BOOTSEL |
| `picotool` permission denied (Linux) | Use `sudo`, or install picotool's udev rules |

### Connection

| Symptom | Likely cause |
|---|---|
| Stuck at `[BLE] scanning...` | Hub is off, `HUB_ADDR` is wrong, or the LEGO app on a phone is connected to the hub |
| `setup timeout` repeated | Weak signal or hub busy; move the Pico closer, restart the hub once |
| `[DBG]` shows `(stale)` on a channel | Wire not connected, **no common ground**, or ArduPilot output not configured/armed |
| `armed=0` with all channels fresh | Enable switch is low (≤ 1600 µs) |
| Steering not centred | Wheels were not centred when the hub was switched on (section 10) |

### Disconnect reason codes

When the link drops, the log prints a reason code:

| Reason | Meaning | Where to look |
|---|---|---|
| `0x08` | Connection timeout | RF, or the hub reset: check for `LOW VOLTAGE` / `HIGH CURRENT` alerts just before. Try fresh batteries and a lower steering limit |
| `0x13` | Remote user terminated | Hub closed the link (error, battery, button pressed) |
| `0x3E` | Failed to establish | Connection attempt failed; it retries automatically |
| `[BOOT] ... WATCHDOG` | Pico was hung | Report it with the log |

A steadily rising `drop` counter means commands are produced faster than the link accepts them.

---

## 14. Repository layout and history

```
├── src/lego_dbw_system.c     firmware (single file)
├── config/btstack_config.h   BTstack options
├── CMakeLists.txt            build definition
├── pico_sdk_import.cmake     standard SDK locator
└── docs/review-v1.md         analysis of v1 problems (Vietnamese)
```

- **v1**: first working version (commit `3a9c55f`).
- **v2**: stability rework. The control loop moved out of IRQ context, TX flow control and connection parameters were added, setup timeouts and a watchdog were introduced, and a PWM glitch that could command full reverse was fixed. See [`docs/review-v1.md`](docs/review-v1.md).

### Roadmap

- Steering calibration routine (find both end-stops, set centre)
- PIO-based PWM capture
- Speed-dependent steering gain, closed-loop speed control
- Telemetry back to ArduPilot
