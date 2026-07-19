# Radxa ROCK 4D and Robot HAT Power Problem

## Summary

The Radxa ROCK 4D does not complete boot reliably when powered from the
SunFounder Robot HAT through the 40-pin header.

Observed behavior:

- The ROCK 4D green LED blinks for several seconds during startup.
- Green LEDs on the Robot HAT blink at the same time.
- The LEDs on both boards then stop together.
- The ROCK 4D does not become reachable on the network.
- The ROCK 4D boots normally from a suitable USB Type-C supply.

This behavior strongly indicates a power brownout. The Robot HAT's 5 V output
is rated for up to approximately 3 A, while the ROCK 4D can demand more current
during boot or under load. Voltage loss in the HAT regulator, battery, header,
or connections can make the available voltage fall below the ROCK 4D's safe
operating level. Lidar, camera, motors, and servos would further reduce the
available power margin.

## Safety rules

- Do not continue repeated boot attempts from the HAT's 5 V header supply.
- Keep both drive motors disconnected during hardware bring-up.
- Do not connect USB-C power while the HAT is also feeding the ROCK 4D's 5 V
  header pins (physical pins 2 and 4). The two supplies could back-feed or
  oppose each other.
- All separately powered devices that exchange I2C signals must share ground.

## Safe I2C test arrangement

For testing communication with the Robot HAT MCU:

1. Power the ROCK 4D from an adequate 5 V USB-C supply.
2. Power the Robot HAT from its battery.
3. Connect only common ground, I2C SDA, and I2C SCL between the boards.
4. Do not connect the HAT's 5 V output to ROCK 4D header pins 2 or 4.
5. Leave the motors disconnected.
6. Run `sudo i2cdetect -y 8` on the ROCK 4D.
7. Confirm that the Robot HAT MCU appears at I2C address `0x14`.

If the HAT must remain physically stacked, pins 2 and 4 need a reliable
electrical isolation method. Temporary pin covering is suitable only when it
cannot move, tear, or expose either 5 V contact.

## Recommended permanent power architecture

- Robot battery to Robot HAT for motors, servos, and the HAT MCU.
- A separate regulated 5 V supply with adequate current capacity to the ROCK
  4D through its supported power input.
- Common ground between the ROCK 4D and Robot HAT.
- I2C SDA and SCL connected between the boards.
- No connection from the HAT's 5 V output to the ROCK 4D's 5 V header pins.

The regulator and wiring should be sized for the ROCK 4D plus its USB and
camera peripherals, including startup and transient loads. The RPLidar C1
should be included in this power budget.

## Status

- I2C bus 8 is enabled on the ROCK 4D using the `rk3576-i2c8-m1` overlay.
- `/dev/i2c-8` is present.
- A scan with the HAT unpowered correctly showed no devices.
- With both boards powered correctly, the Robot HAT MCU is detected at `0x14`.
- A non-root Python `smbus2` transaction on bus 8 successfully read the HAT's
  battery ADC, confirming bidirectional I2C communication.
