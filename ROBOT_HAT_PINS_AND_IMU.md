# Robot HAT Connection to Raspberry Pi & Free Pins for IMU

Summary of how the Robot HAT connects to the Pi, which pins are used, and which IMUs can be connected to the free interfaces.

---

## 1. How the Robot HAT is connected to the Raspberry Pi

- **Physical:** The Robot HAT is a **HAT (Hardware Attached on Top)** that plugs directly onto the **40-pin GPIO header** of the Raspberry Pi (same as Pi 4/5/3B+/Zero 2 W used by PiCar-X).
- **Electrical:** The HAT uses the Pi’s power rails (3.3 V, 5 V, GND) and specific GPIO pins for:
  - **I2C** — The HAT’s onboard MCU (and any devices on the HAT’s I2C port) use the Pi’s I2C bus: **GPIO2 (SDA)** and **GPIO3 (SCL)**.
  - **SPI** — HAT uses Pi SPI: **MOSI (GPIO10), MISO (GPIO9), SCLK (GPIO11), CE0 (GPIO8)**, plus **GPIO6** as BSY.
  - **UART** — HAT exposes Pi UART on a 4-pin connector: **GPIO14 (TXD), GPIO15 (RXD)**.
  - **GPIO** — Various Pi GPIOs are wired to HAT functions (motors, buttons, LED, I2S, MCU reset, etc.) as in the pin mapping below.

So the Robot HAT does **not** use a cable to the Pi; it sits on the header and uses the same pins as in the [SunFounder Robot HAT hardware docs](https://docs.sunfounder.com/projects/robot-hat-v4/en/latest/robot_hat_v5/hardware_introduction.html).

---

## 2. Pin mapping (Robot HAT ↔ Raspberry Pi)

From the codebase (`robot-hat/robot_hat/pin.py`, `device.py`) and [Robot HAT Hardware Introduction](https://docs.sunfounder.com/projects/robot-hat-v4/en/latest/robot_hat_v5/hardware_introduction.html):

| Robot HAT label      | Raspberry Pi GPIO | Used by PiCar-X / HAT |
|----------------------|-------------------|------------------------|
| **I2C** SDA          | GPIO2             | I2C (MCU + port)      |
| **I2C** SCL          | GPIO3             | I2C (MCU + port)      |
| D0                   | GPIO17            | Available              |
| D1                   | GPIO4             | UART TX                |
| D2                   | GPIO27            | **Ultrasonic (trig)**  |
| D3                   | GPIO22            | **Ultrasonic (echo)**  |
| D4                   | GPIO23            | **Motor 1 direction**  |
| D5                   | GPIO24            | **Motor 2 direction**  |
| P0, P1, P2           | via MCU I2C       | **Servos (cam pan, tilt, steering)** |
| P12, P13             | via MCU I2C       | **Motor PWM**          |
| A0, A1, A2           | via MCU I2C       | **Grayscale**          |
| Speaker enable       | GPIO12 (V5) / 20 (V4) | HAT speaker        |
| MCU reset            | GPIO5             | HAT                    |
| RST button           | GPIO16            | HAT                    |
| USR button           | GPIO25            | HAT                    |
| USER LED             | GPIO26            | HAT                    |
| I2S (BCLK, LRCLK, SDATA) | GPIO18, 19, 21 | HAT speaker        |
| **UART** TXD/RXD     | GPIO14, GPIO15    | **Not used by PiCar-X** |
| **SPI** (MOSI, MISO, SCLK, CE0, BSY) | GPIO10, 9, 11, 8, 6 | HAT (CE1 = NC) |

**PWM (P0–P11) and ADC (A0–A3)** are not direct Pi GPIOs; they are provided by the **onboard MCU** over I2C (addresses 0x14, 0x15).

---

## 3. Which pins are free for an IMU

### I2C (best option)

- **Connection:** The HAT has an **I2C port** (P2.54 4-pin and/or **SH1.0 4-pin**, compatible with **QWIIC / STEMMA QT**).
- **Bus:** Same as the Pi’s I2C (GPIO2/3). The onboard MCU uses **0x14** and **0x15**. Any I2C device with a **different address** (e.g. IMU at 0x68, 0x28) can share the bus.
- **Verdict:** **I2C is free for an IMU** — no conflict with PiCar-X; just plug into the HAT’s I2C/QWIIC port.

### UART

- **Connection:** HAT exposes **UART** on a 4-pin P2.54 interface (TXD, RXD, GND, 3.3 V).
- **Usage:** PiCar-X and the examples do **not** use UART.
- **Verdict:** **UART is free** for an IMU that supports UART (e.g. some BNO055 or other modules).

### SPI

- **Connection:** HAT has a 7-pin SPI interface; **CE1** is listed as NC (not connected).
- **Usage:** PiCar-X does not use SPI. You could use **SPI with CE1** (or a separate CS on a free GPIO) for an IMU that supports SPI.
- **Verdict:** **SPI is available** if you wire the IMU to the SPI header and use CE1 or another free GPIO for chip select.

### Summary

| Interface | Pi pins      | Free for IMU? | Notes                                      |
|-----------|-------------|----------------|--------------------------------------------|
| **I2C**   | GPIO2, 3    | **Yes**        | Use HAT’s I2C / QWIIC port; MCU at 0x14/0x15 |
| **UART**  | GPIO14, 15  | **Yes**        | Not used by PiCar-X                        |
| **SPI**   | 9,10,11,8,6 | **Yes** (CE1)  | Use CE1 or extra GPIO for CS               |

**Recommended:** Use **I2C** and the HAT’s **QWIIC/STEMMA QT** port so you don’t need to touch the Pi header.

---

## 4. IMUs that can connect to the free pins

All of these work with **I2C** on the Raspberry Pi; many are available in **QWIIC/STEMMA QT** form and plug straight into the Robot HAT’s I2C port.

### 4.1 I2C IMUs (plug into HAT I2C / QWIIC port)

| IMU           | Axis | I2C address | Notes |
|---------------|------|-------------|--------|
| **MPU6050**   | 6-DoF (accel + gyro) | 0x68 (or 0x69) | Very common, cheap; no magnetometer; you do sensor fusion in software. [Adafruit guide](https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/python-docs). |
| **MPU9250**   | 9-DoF (+ mag)       | 0x68 (or 0x69) | Gyro + accel + magnetometer; good for heading with mag. |
| **BNO055**    | 9-DoF, on-chip fusion | 0x28 or 0x29 | Gives quaternion/heading directly; minimal coding. |
| **BNO085/BNO080** | 9-DoF, fusion   | I2C            | STEMMA QT; quaternion and activity detection. [Adafruit BNO085](https://www.adafruit.com/product/4754). |
| **ICM-20948** | 9-DoF (MPU-9250 upgrade) | I2C       | STEMMA QT; level-shifted for 3.3 V. [Adafruit ICM-20948](https://www.adafruit.com/product/4554). |
| **LSM6DSOX + LIS3MDL** | 9-DoF (accel+gyro + mag) | I2C | STEMMA QT; low drift. [Pimoroni](https://shop.pimoroni.com/products/adafruit-lsm6dsox-lis3mdl-precision-9-dof-imu-stemma-qt-qwiic). |

### 4.2 Suggested choices for PiCar-X (position/heading)

- **Easiest (I2C + QWIIC):** **MPU6050** (6-DoF, very cheap) or **ICM-20948** / **BNO055** (9-DoF, better heading) on the HAT’s I2C/QWIIC port.
- **Best heading with minimal code:** **BNO055** or **BNO085** — built-in fusion, quaternion/heading out.
- **Raspberry Pi setup:** Enable I2C (`raspi-config` → Interface Options → I2C); the Robot HAT install already uses I2C.

### 4.3 Wiring (if not using QWIIC)

If you use a bare I2C IMU on the 4-pin I2C header:

- **SDA** → HAT I2C SDA (Pi GPIO2)  
- **SCL** → HAT I2C SCL (Pi GPIO3)  
- **VCC** → 3.3 V (do **not** use 5 V unless the module is 5 V tolerant)  
- **GND** → GND  

---

## 5. References

- [SunFounder Robot HAT – Hardware Introduction (V4/V5)](https://docs.sunfounder.com/projects/robot-hat-v4/en/latest/robot_hat_v5/hardware_introduction.html)
- [SunFounder PiCar-X – Robot HAT](https://docs.sunfounder.com/projects/picar-x-v20/en/latest/hardware/cpn_robot_hat.html)
- [Adafruit MPU6050 + Raspberry Pi](https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/python-docs)
- [Adafruit BNO085 (STEMMA QT)](https://www.adafruit.com/product/4754)
- [Adafruit ICM-20948 9-DoF IMU (STEMMA QT)](https://www.adafruit.com/product/4554)

---

*Summary: The Robot HAT connects via the Pi’s 40-pin GPIO header. I2C (and optionally UART/SPI) is free for an IMU. Using the HAT’s I2C/QWIIC port, you can add an MPU6050, BNO055, BNO085, or ICM-20948 for heading and position estimation without using any pins already used by PiCar-X.*
