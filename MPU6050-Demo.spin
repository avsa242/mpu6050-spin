{
----------------------------------------------------------------------------------------------------
    Filename:       MPU6050-Demo.spin
    Description:    Demo of the MPU6050 driver
        * 6DoF data output
    Author:         Jesse Burt
    Started:        Nov 6, 2022
    Updated:        Aug 10, 2024
    Copyright (c) 2024 - See end of file for terms of use.
----------------------------------------------------------------------------------------------------
}

' Uncomment the two lines below to use the bytecode-based I2C engine
#define MPU6050_I2C_BC
#pragma exportdef(MPU6050_I2C_BC)

CON

    _clkmode    = cfg._clkmode
    _xinfreq    = cfg._xinfreq


OBJ

    cfg:    "boardcfg.flip"
    time:   "time"
    ser:    "com.serial.terminal.ansi" | SER_BAUD=115_200
    sensor: "sensor.imu.6dof.mpu6050" | SCL=28, SDA=29, I2C_FREQ=400_000, I2C_ADDR=0


PUB setup()

    ser.start()
    time.msleep(30)
    ser.clear()
    ser.strln(@"Serial terminal started")

    if ( sensor.start() )
        ser.strln(@"MPU6050 driver started")
    else
        ser.strln(@"MPU6050 driver failed to start - halting")
        repeat

    sensor.preset_active()

    repeat
        ser.pos_xy(0, 3)
        show_accel_data()
        show_gyro_data()
        if ( ser.getchar_noblock() == "c" )
            cal_accel()
            cal_gyro()

#include "acceldemo.common.spinh"               ' use code common to all accelerometer
#include "gyrodemo.common.spinh"                '   and gyroscope demos


DAT
{
Copyright 2024 Jesse Burt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
}

