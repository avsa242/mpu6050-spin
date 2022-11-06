# mpu6050-spin 
--------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for the InvenSense MPU6050.

**IMPORTANT**: This software is meant to be used with the [spin-standard-library](https://github.com/avsa242/spin-standard-library) (P8X32A) or [p2-spin-standard-library](https://github.com/avsa242/p2-spin-standard-library) (P2X8C4M64P). Please install the applicable library first before attempting to use this code, otherwise you will be missing several files required to build the project.

## Salient Features

* I2C connection at up to 400kHz
* Read accelerometer (raw, micro-g's), gyroscope (raw, micro-dps)
* Set accel, gyro full-scale
* Data-ready flags
* Interrupt support: pin active state, output type, latching, read state
* Set bias offsets
* Set output data rates
* Set optional accel/gyro/temp data low-pass filter
* Clock source: internal oscillator, PLL w/gyro. reference, external
* FIFO modes, status flags

## Requirements

P1/SPIN1:
* spin-standard-library
* 1 extra core/cog for the PASM I2C engine
* sensor.imu.common.spinh (provided by spin-standard-library)

P2/SPIN2:
* p2-spin-standard-library
* sensor.imu.common.spin2h (provided by p2-spin-standard-library)

## Compiler Compatibility

| Processor | Language | Compiler               | Backend     | Status                |
|-----------|----------|------------------------|-------------|-----------------------|
| P1        | SPIN1    | FlexSpin (5.9.18-beta) | Bytecode    | OK                    |
| P1        | SPIN1    | FlexSpin (5.9.18-beta) | Native code | OK                    |
| P1        | SPIN1    | OpenSpin (1.00.81)     | Bytecode    | Untested (deprecated) |
| P2        | SPIN2    | FlexSpin (5.9.18-beta) | NuCode      | FTBFS                 |
| P2        | SPIN2    | FlexSpin (5.9.18-beta) | Native code | Build OK, Run untested|
| P1        | SPIN1    | Brad's Spin Tool (any) | Bytecode    | Unsupported           |
| P1, P2    | SPIN1, 2 | Propeller Tool (any)   | Bytecode    | Unsupported           |
| P1, P2    | SPIN1, 2 | PNut (any)             | Bytecode    | Unsupported           |

## Limitations

* Very early in development - may malfunction, or outright fail to build
* I2C sensor slaves not supported (not currently planned)

