{
----------------------------------------------------------------------------------------------------
    Filename:       sensor.imu.6dof.mpu6050.spin
    Description:    Driver for the InvenSense MPU6050 IMU
    Author:         Jesse Burt
    Started:        Nov 5, 2022
    Updated:        Jan 22, 2026
    Copyright (c) 2026 - See end of file for terms of use.
----------------------------------------------------------------------------------------------------
}
#include "sensor.accel.common.spinh"
#include "sensor.gyroscope.common.spinh"

CON

    { default I/O configuration - these can be overridden by the parent object }
    SCL                 = 28
    SDA                 = 29
    I2C_FREQ            = 100_000
    I2C_ADDR            = 0

    X_AXIS              = 0
    Y_AXIS              = 1
    Z_AXIS              = 2

' Indicate to user apps how many Degrees of Freedom each sub-sensor has
'   (also imply whether or not it has a particular sensor)
    ACCEL_DOF           = 3
    GYRO_DOF            = 3
    MAG_DOF             = 0
    BARO_DOF            = 0
    DOF                 = ACCEL_DOF + GYRO_DOF + MAG_DOF + BARO_DOF

' Scales and data rates used during calibration/bias/offset process
    CAL_XL_SCL          = 2
    CAL_G_SCL           = 250
    CAL_M_SCL           = 0
    CAL_XL_DR           = 400
    CAL_G_DR            = 400
    CAL_M_DR            = 0

' Interrupt active level
    HIGH                = 0
    LOW                 = 1

' Interrupt output type
    INT_PP              = 0
    INT_OD              = 1

' Clear interrupt status options
    READ_INT_FLAG       = 0
    ANY                 = 1

' Interrupt sources
    INT_WAKE_ON_MOTION  = 64
    INT_FIFO_OVERFL     = 16
    INT_FSYNC           = 8
    INT_SENSOR_READY    = 1

' Temperature scales
    C                   = 0
    F                   = 1

' FIFO modes
    BYPASS              = 0
    STREAM              = 1
    FIFO                = 2

' Clock sources
    INT8                = 0
    PLL_GYRO_X          = 1
    PLL_GYRO_Y          = 2
    PLL_GYRO_Z          = 3
    PLL_EXT_32K         = 4
    PLL_EXT_19M2        = 5
    CLKSTOP             = 7


    SLAVE               = core.SLAVE_ADDR
    SLAVE_WR            = core.SLAVE_ADDR
    SLAVE_RD            = core.SLAVE_ADDR|1


VAR

    byte _temp_scale
    byte _addr_bits


OBJ
{ decide: Bytecode I2C engine, or PASM? Default is PASM if BC isn't specified }
#ifdef MPU6050_I2C_BC
    i2c:    "com.i2c.nocog"                     ' BC I2C engine
#else
    i2c:    "com.i2c"                           ' PASM I2C engine
#endif
    core:   "core.con.mpu6050"                  ' MPU6050-specific constants
    time:   "time"                              ' timekeeping methods


PUB null()
' This is not a top-level object


PUB start(): status
' Start using default I/O configuration
    return startx(SCL, SDA, I2C_FREQ, I2C_ADDR)


PUB startx(SCL_PIN, SDA_PIN, I2C_HZ, ADDR_BITS): status
' Start using custom I/O pins and I2C bus speed
    if ( lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31) )
        if ( status := i2c.init(SCL_PIN, SDA_PIN, I2C_HZ) )
            time.usleep(core.TREGRW)            ' wait for device startup
            _addr_bits := (ADDR_BITS << 1)
            if ( dev_id() == core.DEVID_RESP )
                return status
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE


PUB stop()
' Stop the driver
    i2c.deinit()
    longfill(@_abias_fact, 0, 3)


PUB defaults()
' Factory default settings
'   * accel scale: 2g
'   * gyro scale: 250dps
'   * temp scale: Celsius
    reset()


PUB preset_active()
' Like defaults(), but
'   * sets scaling factors for both sub-sensors
    reset()

    ' the registers modified by the following are actually changed by the call
    ' to reset() above, but they need to be called explicitly to set the
    ' scaling factors used by the calculated output data methods
    ' accel_g(), gyro_dps()
    accel_scale(2)
    gyro_scale(250)
    temp_scale(C)
    sleep(false)
    accel_data_rate(CAL_XL_DR)
    gyro_data_rate(CAL_G_DR)


PUB accel_axis_ena(m=-2): c
' Enable data output for Accelerometer - per axis
'   Valid values: 0 or 1, for each axis:
'       Bits    210
'               XYZ
'   Any other value polls the chip and returns the current setting
    c := readreg(core.PWR_MGMT_2)
    case m
        %000..%111:
            ' invert bits because the logic in the chip is actually the reverse
            ' of the method name, i.e., a bit set to 1 _disables_ that axis
            m := ((m ^ core.STBY_INVERT) & core.STBY_XYZA_BITS) << core.STBY_XYZA
            m := ((c & core.STBY_XYZA_MASK) | m)
            writereg(core.PWR_MGMT_2, m)
        other:
            return ((c >> core.STBY_XYZA) & core.STBY_XYZA_BITS) ^ core.STBY_INVERT


PUB accel_bias(x, y, z) | tmp[ACCEL_DOF]
' Read or write/manually set accelerometer calibration offset values
'   x, y, z: pointers to copy offsets to
    long[x] := ~~_abias[X_AXIS]
    long[y] := ~~_abias[Y_AXIS]
    long[z] := ~~_abias[Z_AXIS]


PUB accel_set_bias(x, y, z) | tmp[ACCEL_DOF]
' Write accelerometer calibration offset values
'   Valid values:
'       -32768..32767 (clamped to range)
    _abias[X_AXIS] := -32768 #> x <# 32767
    _abias[Y_AXIS] := -32768 #> y <# 32767
    _abias[Z_AXIS] := -32768 #> z <# 32767


PUB accel_data(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read accelerometer data
    tmp[0] := tmp[1] := 0
    readreg(core.ACCEL_XOUT_H, 6, @tmp)

    long[ptr_x] := ~~tmp.word[2] - _abias[X_AXIS]
    long[ptr_y] := ~~tmp.word[1] - _abias[Y_AXIS]
    long[ptr_z] := ~~tmp.word[0] - _abias[Z_AXIS]


PUB accel_data_rate = xlg_data_rate
' Set accelerometer output data rate, in Hz
'   Valid values: 32..1000
'   Any other value polls the chip and returns the current setting


PUB accel_data_rdy(): f
' Flag indicating new accelerometer data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlg_data_rdy()


PUB accel_lpf_freq(f=-2): c
' Set accelerometer output data low-pass filter cutoff frequency, in Hz
'   Valid values: 0 (disable), 5, 10, 20, 42, 98, 188
'   Any other value polls the chip and returns the current setting
    c := readreg(core.CONFIG)
    case f
        5, 10, 21, 44, 94, 184, 260:
            f := lookdownz(f: 260, 184, 94, 44, 21, 10, 5)
            f := (c & core.DLPF_CFG_MASK) | f
            writereg(core.CONFIG, f)
        other:
            c &= core.DLPF_CFG_BITS
            return lookupz(c: 260, 184, 94, 44, 21, 10, 5)


PUB accel_scale(s=-2): c
' Set accelerometer full-scale range, in g's
'   Valid values: *2, 4, 8, 16
'   Any other value polls the chip and returns the current setting
    c := readreg(core.ACCEL_CFG)
    case s
        2, 4, 8, 16:
            s := lookdownz(s: 2, 4, 8, 16) << core.AFS_SEL
            _ares := lookupz(s >> core.AFS_SEL: 61, 122, 244, 488)
            ' (1/16384, 1/8192, 1/4096, 1/2048) * 1_000_000
            s := ((c & core.AFS_SEL_MASK) | s)
            writereg(core.ACCEL_CFG, s)
        other:
            c := (c >> core.AFS_SEL) & core.AFS_SEL_BITS
            return lookupz(c: 2, 4, 8, 16)


PUB clock_s(s=-2): c
' Set sensor clock source
'   Valid values:
'       INT8 (0): Internal 8MHz oscillator
'       PLL_GYRO_X (1): PLL with X axis gyroscope reference
'       PLL_GYRO_Y (2): PLL with Y axis gyroscope reference
'       PLL_GYRO_Z (3): PLL with Z axis gyroscope reference
'       PLL_EXT_32K (4): PLL with external 32.768kHz reference
'       PLL_EXT_19M2 (5): PLL with external 19.2MHz reference
'       CLKSTOP (7): Stop clock and hold in reset
    c := readreg(core.PWR_MGMT_1)
    case s
        INT8, PLL_GYRO_X..PLL_EXT_19M2:
            s := (c & core.CLKSEL_MASK) | s
            writereg(core.PWR_MGMT_1, s)
        other:
            return c & core.CLKSEL_BITS


PUB dev_id(): id
' Read device ID
'   Returns: $68
    return readreg(core.WHO_AM_I)


PUB i2c_mast_dis() | tmp
' Disable on-chip I2C master
    tmp := readreg(core.INT_PIN_CFG)
    tmp := ((tmp & core.I2C_BYPASS_EN_MASK) | (1 << core.I2C_BYPASS_EN))
    writereg(core.INT_PIN_CFG, tmp)


PUB fifo_ena(e=-2): c
' Enable the FIFO
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
'   NOTE: FALSE disables the interface to the FIFO, but the chip will still write data to it, if FIFO data sources are defined with fifo_src()
    c := readreg(core.USER_CTRL)
    case ||(e)
        0, 1:
            e := ||(e) << core.FIFOEN
            e := ((c & core.FIFOEN_MASK) | e)
            writereg(core.USER_CTRL, e)
        other:
            return (((c >> core.FIFOEN) & 1) == 1)


PUB fifo_full(): f
' Flag indicating FIFO is full
'   Returns: TRUE (-1) if FIFO is full, FALSE (0) otherwise
'   NOTE: If this flag is set, the oldest data has already been dropped from the FIFO
    f := readreg(core.INT_STATUS)
    return (((f >> core.FIFO_OVERFL_INT) & 1) == 1)


PUB fifo_read(len, p_dest)
' Read FIFO data
    readreg(core.FIFO_R_W, len, p_dest)


PUB fifo_reset() | tmp
' Reset the FIFO    XXX - expand..what exactly does it do?
    tmp := 1 << core.FIFO_RST
    writereg(core.USER_CTRL, tmp)


PUB fifo_src(m=-2): c
' Set FIFO source data, as a bitmask
'   Valid values:
'       Bits: 76543210
'           7: Temperature
'           6: Gyro X-axis
'           5: Gyro Y-axis
'           4: Gyro Z-axis
'           3: Accelerometer
'           2: I2C Slave #2
'           1: I2C Slave #1
'           0: I2C Slave #0
'   Any other value polls the chip and returns the current setting
'   NOTE: If any one of the Gyro axis bits or the temperature bits are set,
'   all will be buffered, even if they're not explicitly enabled (chip limitation)
    case m
        %00000000..%11111111:
            writereg(core.FIFO_EN, m)
        other:
            c := readreg(core.FIFO_EN)
            return


PUB fifo_nr_unread(): n
' Number of unread samples stored in FIFO
'   Returns: unsigned 13bit
    return readreg(core.FIFO_COUNTH, 2)


PUB fsync_polarity(p=-2): c
' Set FSYNC pin active state/logic level
'   Valid values: LOW (1), *HIGH (0)
'   Any other value polls the chip and returns the current setting
    c := readreg(core.INT_PIN_CFG)
    case p
        LOW, HIGH:
            p := p << core.FSYNC_INT_LVL
            p := ((c & core.FSYNC_INT_LVL_MASK) | p)
            writereg(core.INT_PIN_CFG, p)
        other:
            return (c >> core.FSYNC_INT_LVL) & 1


PUB gyro_axis_ena(m=-2): c
' Enable data output for Gyroscope - per axis
'   Valid values: 0 or 1, for each axis:
'       Bits    210
'               XYZ
'   Any other value polls the chip and returns the current setting
    c := readreg(core.PWR_MGMT_2)
    case m
        %000..%111:
            ' invert bits because the logic in the chip is actually the reverse
            ' of the method name, i.e., a bit set to 1 _disables_ that axis
            m := ((m ^ core.STBY_INVERT) & core.STBY_XYZG_BITS) << core.STBY_XYZG
            m := ((c & core.STBY_XYZG_MASK) | m)
            writereg(core.PWR_MGMT_2, m)
        other:
            return ((c >> core.STBY_XYZG) & core.STBY_XYZG_BITS) ^ core.STBY_INVERT


PUB gyro_bias(x, y, z) | tmp[GYRO_DOF]
' Read gyroscope calibration offset values
'   x, y, z: pointers to copy offsets to
    long[x] := ~~_gbias[X_AXIS]
    long[y] := ~~_gbias[Y_AXIS]
    long[z] := ~~_gbias[Z_AXIS]


PUB gyro_set_bias(x, y, z)
' Write gyroscope calibration offset values
'   Valid values:
'       -32768..32767 (clamped to range)
    _gbias[X_AXIS] := -32768 #> x <# 32767
    _gbias[Y_AXIS] := -32768 #> y <# 32767
    _gbias[Z_AXIS] := -32768 #> z <# 32767


PUB gyro_data(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read gyro data
    tmp := 0
    readreg(core.GYRO_XOUT_H, 6, @tmp)

    long[ptr_x] := ~~tmp.word[2] - _gbias[X_AXIS]
    long[ptr_y] := ~~tmp.word[1] - _gbias[Y_AXIS]
    long[ptr_z] := ~~tmp.word[0] - _gbias[Z_AXIS]


PUB gyro_data_rate = xlg_data_rate
' Set gyroscope output data rate, in Hz
'   Valid values: 32..1000
'   Any other value polls the chip and returns the current setting


PUB gyro_data_rdy(): f
' Flag indicating new gyroscope data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlg_data_rdy()


PUB gyro_lpf_ena(e=-2): s
' Enable gyroscope data low-pass filter
'   Returns:    TRUE (-1) if enabled, FALSE (0) otherwise
    s := readreg(core.CONFIG)
    return (s & core.DLPF_CFG_BITS) <> 0


PUB gyro_lpf_freq(f=-2): c
' Set gyroscope output data low-pass filter cutoff frequency, in Hz
'   Valid values: 5, 10, 21, 44, 94, 184, 260 
'   Any other value polls the chip and returns the current setting
    c := readreg(core.CONFIG)
    case f
        5, 10, 21, 44, 94, 184, 260:
            f := lookdownz(f: 260, 184, 94, 44, 21, 10, 5)
            f := (c & core.DLPF_CFG_MASK) | f
            writereg(core.CONFIG, f)
        other:
            return lookup(c & core.DLPF_CFG_BITS: 260, 184, 94, 44, 21, 10, 5)


PUB gyro_scale(s=-2): c
' Set gyroscope full-scale range, in degrees per second
'   Valid values: *250, 500, 1000, 2000
'   Any other value polls the chip and returns the current setting
    c := readreg(core.GYRO_CFG)
    case s
        250, 500, 1000, 2000:
            s := lookdownz(s: 250, 500, 1000, 2000) << core.GYRO_FS_SEL
            _gres := lookupz(s >> core.GYRO_FS_SEL: 7633, 15_267, 30_487, 60_975)
            ' (1/131, 1/65.5, 1/32.8, 1/16.4) * 1_000_000
            s := ((c & core.GYRO_FS_SEL_MASK) | s)
            writereg(core.GYRO_CFG, s)
        other:
            c := (c >> core.GYRO_FS_SEL) & core.GYRO_FS_SEL_BITS
            return lookupz(c: 250, 500, 1000, 2000)


PUB int_polarity(p=-2): c
' Set interrupt pin active state/logic level
'   Valid values: LOW (1), *HIGH (0)
'   Any other value polls the chip and returns the current setting
    c := readreg(core.INT_PIN_CFG)
    case p
        LOW, HIGH:
            p := p << core.LEVEL
            p := ((c & core.LEVEL_MASK) | p)
            writereg(core.INT_PIN_CFG, p)
        other:
            return ((c >> core.LEVEL) & 1)


PUB int_clear_mode(m=-2): c
' Select mode by which interrupt status may be cleared
'   Valid values:
'      *READ_INT_FLAG (0): Only by reading interrupt flags
'       ANY (1): By any read operation
'   Any other value polls the chip and returns the current setting
    c := readreg(core.INT_PIN_CFG)
    case m
        ANY, READ_INT_FLAG:
            m := m << core.INT_RD_CLEAR
            m := ((c & core.INT_RD_CLEAR_MASK) | m)
            writereg(core.INT_PIN_CFG, m)
        other:
            return ((c >> core.INT_RD_CLEAR) & 1)


PUB interrupt(): f
' Indicates one or more interrupts have been asserted
'   Returns: non-zero result if any interrupts have been asserted:
'       INT_WAKE_ON_MOTION (64) - Wake on motion interrupt occurred
'       INT_FIFO_OVERFL (16) - FIFO overflowed
'       INT_FSYNC (8) - FSYNC interrupt occurred
'       INT_SENSOR_READY (1) - Sensor raw data updated
    return readreg(core.INT_STATUS)


PUB int_latch_ena(l=-2): c
' Latch interrupt pin when interrupt asserted
'   Valid values:
'      *FALSE (0): Interrupt pin is pulsed (width = 50uS)
'       TRUE (-1): Interrupt pin is latched, and must be cleared explicitly
'   Any other value polls the chip and returns the current setting
    c := readreg(core.INT_PIN_CFG)
    case ||(l)
        0, 1:
            l := ||(l) << core.LATCH_INT_EN
            l := ((c & core.LATCH_INT_EN_MASK) | l)
            writereg(core.INT_PIN_CFG, l)
        other:
            return (((c >> core.LATCH_INT_EN) & 1) == 1)


PUB int_mask(m=-2): c
' Allow interrupts to assert INT pin, set by mask, or by ORing together symbols shown below
'   Valid values:
'       Bits: %x6x43xx0 (bit positions marked 'x' aren't supported by the device; setting any of them to '1' will be considered invalid and will query the current setting, instead)
'               Function                                Symbol              Value
'           6: Enable interrupt for wake on motion      INT_WAKE_ON_MOTION (64)
'           4: Enable interrupt for FIFO overflow       INT_FIFO_OVERFL  (16)
'           3: Enable FSYNC interrupt                   INT_FSYNC           (8)
'           1: Enable raw Sensor Data Ready interrupt   INT_SENSOR_READY    (1)
'   Any other value polls the chip and returns the current setting
    case m & (core.INT_ENABLE_MASK ^ $FF)    ' check for any invalid bits:
        0:                                      ' result should be 0 if all ok
            m &= core.INT_ENABLE_MASK
            writereg(core.INT_ENABLE, m)
        other:                                  ' one or more invalid bits;
            c := readreg(core.INT_ENABLE)
            return c & core.INT_ENABLE_MASK


PUB int_outp_type(t=-2): c
' Set interrupt pin output mode
'   Valid values:
'      *INT_PP (0): Push-pull
'       INT_OD (1): Open-drain
'   Any other value polls the chip and returns the current setting
    c := readreg(core.INT_PIN_CFG)
    case t
        INT_PP, INT_OD:
            t := t << core.OPEN
            t := ((c & core.OPEN_MASK) | t)
            writereg(core.INT_PIN_CFG, t)
        other:
            return ((c >> core.OPEN) & 1)


PUB reset() | tmp
' Perform soft-reset
    tmp := core.XLG_SOFT_RST
    writereg(core.PWR_MGMT_1, tmp)


PUB sleep(s=-2): c
' Enable low-power sleep mode
    c := readreg(core.PWR_MGMT_1)
    case s
        0, 1:
            s := s << core.SLEEP
            s := ((c & core.SLEEP_MASK) | s)
            writereg(core.PWR_MGMT_1, s)
        other:
            return (((c >> core.SLEEP) & 1) == 1)


PUB temp_data_rate = xlg_data_rate
' Set temperature output data rate, in Hz
'   Valid values: 32..1000
'   Any other value polls the chip and returns the current setting
'   NOTE: This setting affects the accelerometer and gyroscope data rate
'   (hardware limitation)


PUB temperature(): t
' Read temperature, in hundredths of a degree
    t := readreg(core.TEMP_OUT_H, 2)
    case _temp_scale
        F:
        other:
            return ((t * 1_0000) / 333_87) + 21_00 'XXX unverified


PUB temp_scale(s=-2): c
' Set temperature scale used by Temperature method
'   Valid values:
'       C (0): Celsius
'       F (1): Fahrenheit
'   Any other value returns the current setting
    case s
        C, F:
            _temp_scale := s
        other:
            return _temp_scale


PUB xlg_data_rate(r=-2): c | mx
' Set accelerometer/gyro/temp sensor output data rate, in Hz
'   r:
'       32..1000 when low-pass filtering is enabled
'       32..8000 when low-pass filtering is disabled
'       other values:   returns the current setting
    if ( gyro_lpf_ena() )
        mx := 1000
    else
        mx := 8000

    case r
        31..mx:
            r := (mx / r) - 1
            writereg(core.SMPLRT_DIV, r)
        other:
            c := readreg(core.SMPLRT_DIV)
            return 1000 / (c + 1)


PUB xlg_data_rdy(): f
' Flag indicating new gyroscope/accelerometer data is ready to be read
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    f := readreg(core.INT_STATUS)
    return ((f & 1) == 1)


PRI readreg(reg_nr, len=1, p_dest=0): v | cmd_pkt
' Read nr_bytes from the slave device ptr_buff
    v := 0
    case reg_nr                                 ' validate reg
        core.SELF_TEST_X..core.SELF_TEST_A, core.SMPLRT_DIV..core.ACCEL_CFG, ...
        core.FIFO_EN..core.INT_ENABLE, core.INT_STATUS..core.EXT_SENS_DATA_23, ...
        core.I2C_SLV0_DO..core.SIGNAL_PATH_RESET, core.USER_CTRL..core.PWR_MGMT_2, ...
        core.FIFO_COUNTH..core.WHO_AM_I:
            { accel/gyro regs }
            cmd_pkt.byte[0] := (SLAVE_WR | _addr_bits)
            cmd_pkt.byte[1] := reg_nr.byte[0]
            if ( len =< 4 )
                p_dest := @v
            i2c.start()
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start()
            i2c.write(SLAVE_RD | _addr_bits)
            i2c.rdblock_msbf(p_dest, len, i2c.NAK)
            i2c.stop()
        other:
            return


PRI writereg(reg_nr, val) | cmd_pkt
' Write nr_bytes to the slave device from ptr_buff
    case reg_nr                                 ' validate reg
        core.SELF_TEST_X..core.SELF_TEST_A, ...
        core.SMPLRT_DIV..core.ACCEL_CFG, core.FIFO_EN..core.I2C_SLV4_CTRL, core.INT_PIN_CFG, ...
        core.INT_ENABLE, core.I2C_SLV0_DO..core.SIGNAL_PATH_RESET, ...
        core.USER_CTRL..core.PWR_MGMT_2, core.FIFO_COUNTH..core.FIFO_R_W:
            { accel/gyro regs }
            cmd_pkt.byte[0] := (SLAVE_WR | _addr_bits)
            cmd_pkt.byte[1] := reg_nr.byte[0]
            i2c.start()
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.wrblock_msbf(@val, 1)
            i2c.stop()
        other:
            return


DAT
{
Copyright 2026 Jesse Burt

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

