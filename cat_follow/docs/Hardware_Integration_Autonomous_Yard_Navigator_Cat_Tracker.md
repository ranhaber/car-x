# Hardware Integration
**Project:** Autonomous Yard Navigator & Cat Tracker (PiCar-X Platform)  
**Compute target:** Radxa ROCK 4D + SunFounder Robot HAT + PiCar-X chassis  
**Sensors:** Slamtec RPLidar C1 (USB), onboard camera (MIPI CSI), HC-SR04 ultrasonic  
**Version:** 1.0  
**Status:** ROCK 4D and Radxa Camera 4K bring-up validated; lidar, grayscale, and thermal tests pending

## 1. Purpose
This document defines how PiCar-X hardware connects to the Radxa ROCK 4D for
single-board operation (Option B). It covers pin mapping, power budgeting,
peripheral placement, and bring-up order.

Related software wiring is in
`Software_Integration_Autonomous_Yard_Navigator_Cat_Tracker.md`.

Architecture contracts remain in the PRD, HLD, and Interface Specification.

## 2. Platform Summary

| Component | Role |
|-----------|------|
| **Radxa ROCK 4D** (RK3576) | Main compute: `cat_follow`, ROS 2, lidar, camera, Nav2 |
| **SunFounder Robot HAT V4** | Motor driver, servo PWM MCU, ADC, ultrasonic header, battery |
| **PiCar-X chassis** | 2× rear hub motors, 3× servos, grayscale module, ultrasonic |
| **RPLidar C1** | 360° 2D lidar for mapping and obstacle-aware navigation |
| **Overhead camera system** | Global cat/car pose via UDP (external to this board) |

### 2.1 Critical compatibility note
The Robot HAT 40-pin header is **physically** the same size as Raspberry Pi,
but **not electrically identical**. GPIO numbers on ROCK 4D differ from Pi BCM.
`robot_hat` must be ported to Radxa `libgpiod` and Radxa I2C device-tree
overlays before `picarx` will work.

SunFounder documents the HAT for Raspberry Pi only. This integration is
community/engineering baseline, not vendor-supported.

## 3. PiCar-X default pin usage (Robot HAT labels)

From `picar-x/picarx/picarx.py` and SunFounder Robot HAT V4 docs.

| Function | HAT label | Implementation | Pi BCM GPIO |
|----------|-----------|----------------|-------------|
| Camera pan servo | P0 | MCU PWM ch 0 via I2C | — |
| Camera tilt servo | P1 | MCU PWM ch 1 via I2C | — |
| Steering servo | P2 | MCU PWM ch 2 via I2C | — |
| Left motor direction | D4 | Direct GPIO | GPIO23 |
| Right motor direction | D5 | Direct GPIO | GPIO24 |
| Left motor speed | P13 | MCU PWM ch 13 via I2C | — |
| Right motor speed | P12 | MCU PWM ch 12 via I2C | — |
| Ultrasonic TRIG | D2 | Direct GPIO | GPIO27 |
| Ultrasonic ECHO | D3 | Direct GPIO input | GPIO22 |
| Grayscale ×3 | A0–A2 | MCU ADC ch 0–2 via I2C | — |
| MCU I2C bus | — | Pi I2C1 | GPIO2 (SDA), GPIO3 (SCL) |
| MCU reset | MCURST | Direct GPIO | GPIO5 |

PWM and ADC do **not** use host GPIO directly. The onboard AT32F413 MCU
(address **0x14**) generates servo/motor PWM and reads ADC over I2C.

Motor ports on the HAT (XH2.54) connect to rear hub motors separately from
the 40-pin header.

## 4. ROCK 4D 40-pin mapping (Option B target)

Physical pin positions match the Pi header. **Signal names differ.**

| Phys pin | Pi (BCM) | ROCK 4D default | PiCar-X / HAT use | Notes |
|----------|----------|---------------|-------------------|-------|
| 2, 4 | 5V | 5V | Power in to ROCK 4D | See §5 — marginal alone |
| 3 | GPIO2 / SDA | GPIO1_C7 | I2C8_SDA_M1 → MCU | Enable I2C overlay |
| 5 | GPIO3 / SCL | GPIO1_C6 | I2C8_SCL_M1 → MCU | Enable I2C overlay |
| 13 | GPIO27 | GPIO2_C0 | D2 ultrasonic TRIG | libgpiod output |
| 15 | GPIO22 | GPIO1_C5 | D3 ultrasonic ECHO | libgpiod input |
| 16 | GPIO23 | GPIO2_B6 | D4 left motor DIR | libgpiod output |
| 18 | GPIO24 | GPIO2_B7 | D5 right motor DIR | libgpiod output |
| 29 | GPIO5 | GPIO3_A2 | MCURST | libgpiod output |

Servo connectors (P0, P1, P2) and motor ports remain on the HAT PCB; only
the host GPIO/I2C lines above must be correct on the 40-pin stack.

Reference: [Radxa ROCK 4D GPIO](https://docs.radxa.com/en/rock4/rock4d/hardware-use/pin-gpio),
[SunFounder Robot HAT V4](https://docs.sunfounder.com/projects/robot-hat-v4/en/stable/hardware_introduction.html).

## 5. Power architecture

### 5.1 Voltage rules
| Device | Accepted input |
|--------|----------------|
| Robot HAT battery | **6.0–8.4 V** (2× 18650 in series) |
| ROCK 4D | **5 V only** (USB Type-C or GPIO pins 2 & 4) |
| ROCK 4D | **Does not** accept 7–12 V on the GPIO header |

### 5.2 Robot HAT electrical limits (SunFounder)
| Rail | Spec |
|------|------|
| 5 V output to host (pins 2 & 4) | **3.0 A max** (~15 W) |
| Motor driver (per channel) | 5 V / 1.8 A × 2 |
| Battery input | 6.0–8.4 V |

### 5.3 ROCK 4D requirements (Radxa)
| Condition | Minimum |
|-----------|---------|
| Light use (no heavy USB) | **10 W** (~2 A @ 5 V) |
| Recommended stable operation | **≥ 3 A** @ 5 V |
| Full USB3 + PCIe load | **25 W** (~5 A) |
| Measured SoC only | ~2.7 W idle, ~5–6 W CPU load |

### 5.4 Power diagram
```text
2× 18650 (7.4 V nom)
        │
        ▼
  Robot HAT PMIC / DC-DC
        │
        ├──► 5 V / 3 A max ──► ROCK 4D GPIO 5 V (pins 2 & 4)  [marginal]
        │
        ├──► Motor driver ──► rear hub motors (≤ 1.8 A × 2)
        │
        └──► 5 V servo rail ──► pan / tilt / steering servos
```

Motors and servos share the **same battery** but use **separate rails**.
Motor inrush can sag battery voltage and brown out the host. SunFounder
warns that high starting current can restart the Raspberry Pi; ROCK 4D with
ROS 2 + lidar + camera is **more sensitive**.

### 5.5 Recommended power strategies

| Strategy | When | Wiring |
|----------|------|--------|
| **A — Bench bring-up (validated)** | GPIO/I2C/motor tests | Wall **5 V / 4 A** USB-C → ROCK 4D; battery → HAT for motors only; isolate header pins 2 & 4 |
| **B — Dual rail (recommended)** | Autonomous runs | Battery → HAT (motors/servos); **separate 5 V / 3 A+ buck** → ROCK USB-C; common ground |
| **C — HAT 5 V only (rejected)** | Do not use on this build | ROCK failed to finish boot; observed behavior is consistent with brownout |

If ROCK 4D board revision is **v1.12+**, the dedicated external 5 V input
may be used instead of USB-C ([Radxa power header](https://docs.radxa.com/en/rock4/rock4d/hardware-use/power_header)).

### 5.6 Peripheral power notes
| Device | Connection | Est. draw @ 5 V |
|--------|------------|-----------------|
| RPLidar C1 | USB to ROCK 4D | ~0.3–0.5 A |
| MIPI camera | ROCK CSI | ~0.2–0.3 A |
| WiFi | Onboard | included in SoC budget |

Use a **powered USB hub** for the C1 only if the main 5 V rail has headroom.

## 6. Sensor and actuator placement

| Device | Mount | Interface to ROCK 4D |
|--------|-------|----------------------|
| RPLidar C1 | Centered on chassis, scan plane ~10–15 cm above ground | USB `/dev/ttyUSB0` (typical) |
| Camera | PiCar-X pan/tilt | MIPI CSI (Radxa cable; not Pi CSI ribbon) |
| Ultrasonic HC-SR04 | Front bumper | D2/D3 via Robot HAT |
| Grayscale (optional) | Underside | A0–A2 via MCU |

### 6.1 Lidar
- Use powered USB or adequate 5 V budget.
- Add udev rule for dialout/render group on serial device.
- C1 baud rate: follow `sllidar_ros2` / SDK launch file (model-specific).

### 6.2 Camera
The validated onboard camera is the **Radxa Camera 4K**, connected to the
ROCK 4D **31-pin CSI connector**. It uses a Sony IMX415 sensor and a four-lane
MIPI CSI-2 interface.

#### Sensor specification
- Sensor: Sony IMX415
- Optical format: diagonal 6.43 mm (Type 1/2.8)
- Effective resolution: 8.29 megapixels
- Unit-cell size: 1.45 µm horizontal × 1.45 µm vertical

#### Sensor output
- Interface: MIPI CSI-2, four serial data lanes
- Raw formats: RAW10 and RAW12

#### Lens specification
- Mount/interface: M12 × P0.5
- Effective focal length (EFL): 2.95 mm ±5%
- Back focal length (BFL): 4.64 mm
- Flange back length (FBL): 4.00 mm
- Single-lens operating temperature: −40 °C to +85 °C

#### Optical field of view
- Diagonal: 88.2° ±5°
- Horizontal: 75° ±3°
- Vertical: 59° ±2°
- Chief ray angle (CRA): 15°

#### Validated ROCK 4D interface
- Sensor control: I2C5, address `0x1a`
- Native sensor mode: 3864×2192, SGBRG10
- Runtime capture: RKISP main path `/dev/video11`
- Runtime format: 640×480 NV12 at 30 FPS, converted to 640×480 BGR
- Device-tree overlay: `rock-4d-radxa-camera-4k`

The overlay routes the sensor through
`csi2_dphy0 → mipi1_csi2 → rkcif_mipi_lvds1 → rkisp_vir1`.
The IMX415 was detected with chip ID `0x0000e0`, streamed 30 frames without
errors, and produced a visible image on Armbian.

The original PiCar-X OV5647 module is not used on the ROCK 4D. Its Pi ribbon
and control-voltage requirements are not compatible with this validated
31-pin IMX415 integration.

## 7. Integration options

### Option A — Dual board (lowest risk)
```text
ROCK 4D  → compute, ROS 2, lidar, cat_follow
Raspberry Pi 4 → Robot HAT only (motors/servos)
Link: UDP motor command channel
```
No `robot_hat` port required. Extra board and wiring.

### Option B — Single board (this document)
```text
ROCK 4D stacked on Robot HAT (or jumpered 40-pin signals)
Port robot_hat → libgpiod + Radxa I2C
```
Best long-term if power and GPIO port succeed.

## 8. Hardware bring-up checklist

### Phase H1 — Power and I2C
- [x] Flash Armbian Ubuntu 24.04 (see Software Integration doc).
- [x] Enable I2C8_M1 on header pins 3 & 5.
- [x] `i2cdetect -y 8` shows MCU at **0x14**.
- [x] Pulse MCURST (phys pin 29); MCU re-enumerates.

### Phase H2 — GPIO
- [x] Configure D2, D3, D4, D5, MCURST as libgpiod lines.
- [x] Ultrasonic returns a plausible distance.
- [x] Motor direction pins toggle (wheels off ground).

### Phase H3 — PWM via MCU
- [x] Servo P0/P1/P2 respond (pan, tilt, steer).
- [x] Motor PWM P12/P13 spin wheels at low speed.

### Phase H4 — Sensors
- [ ] C1 publishes scan on USB.
- [x] Radxa Camera 4K captures frames through RKISP at 30 FPS.
- [ ] Grayscale A0–A2 read (if used).

### Phase H5 — Integrated drive
- [x] Combined motor + servo + I2C bench test under dual-rail power without brownout.
- [ ] Thermal: heatsink/fan on ROCK 4D; monitor throttle.

### Verified calibration
| Setting | Value |
|---------|-------|
| Motor direction | `[1, 1]` |
| Steering center | `-6°` |
| Camera pan center | `+8°` |
| Camera tilt center | `-3°` |

The values are stored in `/opt/picar-x/picar-x.conf`.

## 9. Mechanical and safety
- Secure ROCK 4D stack; vibration affects lidar mount and USB connectors.
- Emergency stop must cut motion via `DecisionEngine` / `MotorInterface`
  regardless of ROS 2 state.
- Detection and tracking must work without web UI (architecture rule).
- Keep ultrasonic/dToF forward veto active even when Nav2 is running.

## 10. References
- SunFounder Robot HAT V4: https://docs.sunfounder.com/projects/robot-hat-v4/en/stable/hardware_introduction.html
- Radxa ROCK 4D GPIO: https://docs.radxa.com/en/rock4/rock4d/hardware-use/pin-gpio
- Radxa ROCK 4D power: https://docs.radxa.com/en/rock4/rock4d/hardware-use/usb-type-c
- Radxa product brief: https://dl.radxa.com/rock4/4d/docs/radxa_rock4d_product_brief.pdf
- Repo: `picar-x/picarx/picarx.py`, `robot-hat/robot_hat/pin.py`, `robot-hat/robot_hat/pwm.py`
- Slamtec C1 SDK: `rplidar_sdk-master/` (repo root)
