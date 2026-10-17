# LEGO 42160 Drive-by-Wire Gateway

Turns the **LEGO Technic 42160 Audi RS Q e-tron** into a drive-by-wire platform controlled by **ArduPilot**.

A **Raspberry Pi Pico W** reads PWM outputs from an ArduPilot flight controller and drives the LEGO Technic Hub over BLE using the **LEGO Wireless Protocol 3 (LWP3)**. The gateway enforces safety gating, so the car can run under manual RC or a full autonomous stack.

```
[Matek F405 Mini / ArduPilot]
        │ PWM: throttle, steering, enable (deadman)
        ▼
[Raspberry Pi Pico W]  ── PWM capture · safety gating · BLE central
        │ BLE / LWP3
        ▼
[LEGO Technic Hub]
   ├─ Port A/B: drive motors      (StartPower)
   └─ Port D:   steering motor    (GotoAbsolutePosition → servo-like, auto-centering)
```

## Hardware

| Item | Notes |
|---|---|
| LEGO 42160 + Technic Hub | Hub powered by its own AA pack |
| Matek F405 Mini | ArduPilot (Rover); powered separately |
| Raspberry Pi Pico W | Powered from USB or 5 V rail |
| 3 signal wires + GND | **Common ground between Matek and Pico is required** |

### Wiring

| Matek output | Signal | Pico GPIO |
|---|---|---|
| S1 | Throttle | GP2 |
| S2 | Steering | GP3 |
| S3 | Enable (deadman) | GP4 |

There is no wired connection to the LEGO hub; the link is BLE only.

## Build

Requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.2.x (with `btstack`, `cyw43-driver` and `tinyusb` submodules) and `arm-none-eabi-gcc`.

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build
cmake --build build -j
# flash build/lego_dbw.uf2 (hold BOOTSEL while plugging in the Pico W)
```

Build notes:
- BLE only: the project defines `CYW43_LWIP=0`, so no lwIP/WiFi stack is pulled in.
- `config/btstack_config.h` enables `ENABLE_LE_PERIPHERAL` in addition to `ENABLE_LE_CENTRAL`. The peripheral role is unused, but BTstack's `hci.c` in SDK 2.2 does not compile without it.

## Configuration

All tunables are at the top of `src/lego_dbw_system.c`.

| Setting | Default | Meaning |
|---|---|---|
| `HUB_ADDR` | `90:84:2B:7F:D4:40` | MAC of **your** Technic Hub |
| `ENABLE_THRESHOLD_US` | 1600 | Enable channel must be above this to arm |
| `PWM_TIMEOUT_MS` | 250 | Signal age before a channel counts as lost |
| `STEER_TARGET_DEG_MAX` | 40 | Wheel-side angle at full stick |
| `STEER_GEAR_RATIO` | 6 | Motor degrees per wheel degree (measured ~6:1) |
| `STEER_MOTOR_DEG_LIMIT` | 240 | **Measure your end-stop and set ~10% below it** |
| `STEER_GOTO_MAXPOWER` | 80 | Lower = less stall current at the end-stop |
| `CONN_SUPERVISION_TO` | 200 (2 s) | BLE link supervision timeout |

## Safety behaviour

The gateway commands **throttle 0 and steering centre** whenever any of these is true:
- the enable PWM is below threshold or stale,
- the throttle or steering PWM is stale (> 250 ms),
- a pulse is outside 800–2200 µs (rejected as noise).

When the BLE link is down, the gateway cannot send anything. Behaviour then depends on the hub firmware: **verify on your hardware that the hub stops the motors when the link drops** (hold throttle, unplug the Pico, observe).

A 1 s hardware watchdog reboots the Pico if its run loop stalls; the boot log reports watchdog resets.

## Troubleshooting

Serial log (USB CDC) prints a status line every second, plus BLE events. On disconnect, the reason code tells you where to look:

| Reason | Meaning | Likely cause |
|---|---|---|
| `0x08` | Connection timeout | RF, or the hub reset (check for `LOW VOLTAGE` / `HIGH CURRENT` alerts just before) |
| `0x13` | Remote user terminated | Hub closed the link (error, battery, button) |
| `0x3E` | Failed to establish | Connection attempt failed; it retries automatically |
| `[BOOT] ... WATCHDOG` | Pico was hung | Report it with the log |

A steadily rising `tx drop` counter means commands are produced faster than the link accepts them.

## Project history

- **v1**: first working version (commit `3a9c55f`).
- **v2**: stability rework. The control loop moved out of IRQ context, TX flow control and connection parameters were added, setup timeouts and a watchdog were introduced, and a PWM glitch that could command full reverse was fixed. See [`docs/review-v1.md`](docs/review-v1.md) for the full analysis.

## Roadmap

- Steering calibration routine (find both end-stops, set centre) instead of relying on the encoder position at hub power-on
- PIO-based PWM capture
- Speed-dependent steering gain, closed-loop speed control
- Telemetry back to ArduPilot
