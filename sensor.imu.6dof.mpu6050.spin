{
    --------------------------------------------
    Filename: sensor.imu.6dof.mpu6050.spin
    Author: Jesse Burt
    Description: Driver for the InvenSense MPU6050
    Copyright (c) 2022
    Started Nov 5, 2022
    Updated Nov 13, 2022
    See end of file for terms of use.
    --------------------------------------------
}
#include "sensor.accel.common.spinh"
#include "sensor.gyroscope.common.spinh"

CON

    SLAVE               = core#SLAVE_ADDR
    SLAVE_WR            = core#SLAVE_ADDR
    SLAVE_RD            = core#SLAVE_ADDR|1

    DEF_SCL             = 28
    DEF_SDA             = 29
    DEF_HZ              = 100_000
    DEF_ADDR            = 0
    I2C_MAX_FREQ        = core#I2C_MAX_FREQ

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

VAR

    byte _temp_scale
    byte _addr_bits

OBJ
{ decide: Bytecode I2C engine, or PASM? Default is PASM if BC isn't specified }
#ifdef MPU6050_I2C_BC
    i2c : "com.i2c.nocog"                       ' BC I2C engine
#else
    i2c : "com.i2c"                             ' PASM I2C engine
#endif
    core: "core.con.mpu6050"
    time: "time"

PUB null{}
' This is not a top-level object

PUB start{}: status
' Start using "standard" Propeller I2C pins and 100kHz
    return startx(DEF_SCL, DEF_SDA, DEF_HZ, DEF_ADDR)

PUB startx(SCL_PIN, SDA_PIN, I2C_HZ, ADDR_BITS): status
' Start using custom I/O pins and I2C bus speed
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31) and {
}   I2C_HZ =< core#I2C_MAX_FREQ
        if (status := i2c.init(SCL_PIN, SDA_PIN, I2C_HZ))
            time.usleep(core#TREGRW)            ' startup time
            _addr_bits := (ADDR_BITS << 1)
            if (dev_id{} == core#DEVID_RESP)
                return status
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

PUB stop{}
' Stop the driver
    i2c.deinit{}
    longfill(@_abias_fact, 0, 3)

PUB defaults{}
' Factory default settings
'   * accel scale: 2g
'   * gyro scale: 250dps
'   * temp scale: Celsius
    reset{}

PUB preset_active{}
' Like defaults(), but
'   * sets scaling factors for both sub-sensors
    reset{}

    ' the registers modified by the following are actually changed by the call
    ' to reset() above, but they need to be called explicitly to set the
    ' scaling factors used by the calculated output data methods
    ' accel_g(), gyro_dps()
    accel_scale(2)
    gyro_scale(250)
    temp_scale(C)
    sleep(false)

PUB accel_axis_ena(xyz_mask): curr_mask
' Enable data output for Accelerometer - per axis
'   Valid values: 0 or 1, for each axis:
'       Bits    210
'               XYZ
'   Any other value polls the chip and returns the current setting
    curr_mask := 0
    readreg(core#PWR_MGMT_2, 1, @curr_mask)
    case xyz_mask
        %000..%111:
            ' invert bits because the logic in the chip is actually the reverse
            ' of the method name, i.e., a bit set to 1 _disables_ that axis
            xyz_mask := ((xyz_mask ^ core#STBY_INVERT) & core#STBY_XYZA_BITS) << core#STBY_XYZA
        other:
            return ((curr_mask >> core#STBY_XYZA) & core#STBY_XYZA_BITS) ^ core#STBY_INVERT

    xyz_mask := ((curr_mask & core#STBY_XYZA_MASK) | xyz_mask)
    writereg(core#PWR_MGMT_2, 1, @xyz_mask)

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
    tmp := 0
    readreg(core#ACCEL_XOUT_H, 6, @tmp)

    long[ptr_x] := ~~tmp.word[2] - _abias[X_AXIS]
    long[ptr_y] := ~~tmp.word[1] - _abias[Y_AXIS]
    long[ptr_z] := ~~tmp.word[0] - _abias[Z_AXIS]

PUB accel_data_rate(rate): curr_rate
' Set accelerometer output data rate, in Hz
'   Valid values: 4..1000
'   Any other value polls the chip and returns the current setting
    return xlg_data_rate(rate)

PUB accel_data_rdy{}: flag
' Flag indicating new accelerometer data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlg_data_rdy{}

PUB accel_lpf_freq(freq): curr_freq
' Set accelerometer output data low-pass filter cutoff frequency, in Hz
'   Valid values: 0 (disable), 5, 10, 20, 42, 98, 188
'   Any other value polls the chip and returns the current setting
    curr_freq := 0
    readreg(core#CONFIG, 1, @curr_freq)
    case freq
        5, 10, 21, 44, 94, 184, 260:
            freq := lookdownz(freq: 260, 184, 94, 44, 21, 10, 5)
        other:
            curr_freq &= core#DLPF_CFG_BITS
            return lookupz(curr_freq: 260, 184, 94, 44, 21, 10, 5)

    freq := (curr_freq & core#DLPF_CFG_MASK) | freq
    writereg(core#CONFIG, 1, @freq)

PUB accel_scale(g): curr_scl
' Set accelerometer full-scale range, in g's
'   Valid values: *2, 4, 8, 16
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
    readreg(core#ACCEL_CFG, 1, @curr_scl)
    case g
        2, 4, 8, 16:
            g := lookdownz(g: 2, 4, 8, 16) << core#AFS_SEL
            _ares := lookupz(g >> core#AFS_SEL: 61, 122, 244, 488)
            ' (1/16384, 1/8192, 1/4096, 1/2048) * 1_000_000
        other:
            curr_scl := (curr_scl >> core#AFS_SEL) & core#AFS_SEL_BITS
            return lookupz(curr_scl: 2, 4, 8, 16)

    g := ((curr_scl & core#AFS_SEL_MASK) | g) & core#ACCEL_CFG_MASK
    writereg(core#ACCEL_CFG, 1, @g)

PUB clock_src(src): curr_src
' Set sensor clock source
'   Valid values:
'       INT8 (0): Internal 8MHz oscillator
'       PLL_GYRO_X (1): PLL with X axis gyroscope reference
'       PLL_GYRO_Y (2): PLL with Y axis gyroscope reference
'       PLL_GYRO_Z (3): PLL with Z axis gyroscope reference
'       PLL_EXT_32K (4): PLL with external 32.768kHz reference
'       PLL_EXT_19M2 (5): PLL with external 19.2MHz reference
'       CLKSTOP (7): Stop clock and hold in reset
    curr_src := 0
    readreg(core#PWR_MGMT_1, 1, @curr_src)
    case src
        INT8, PLL_GYRO_X..PLL_EXT_19M2:
        other:
            return curr_src & core#CLKSEL_BITS

    src := (curr_src & core#CLKSEL_MASK) | src
    writereg(core#PWR_MGMT_1, 1, @src)

PUB dev_id{}: id
' Read device ID
'   Returns: $68
    id := 0
    readreg(core#WHO_AM_I, 1, @id)

PUB i2c_mast_dis{} | tmp
' Disable on-chip I2C master
    tmp := 0
    readreg(core#INT_PIN_CFG, 1, @tmp)
    tmp := ((tmp & core#I2C_BYPASS_EN_MASK) | (1 << core#I2C_BYPASS_EN))
    writereg(core#INT_PIN_CFG, 1, @tmp)

PUB fifo_ena(state): curr_state
' Enable the FIFO
'   Valid values: TRUE (-1 or 1), FALSE (0)
'   Any other value polls the chip and returns the current setting
'   NOTE: FALSE disables the interface to the FIFO, but the chip will still write data to it, if FIFO data sources are defined with fifo_src()
    curr_state := 0
    readreg(core#USER_CTRL, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#FIFOEN
            state := ((curr_state & core#FIFOEN_MASK) | state)
            writereg(core#USER_CTRL, 1, @state)
        other:
            return (((curr_state >> core#FIFOEN) & 1) == 1)

PUB fifo_full{}: flag
' Flag indicating FIFO is full
'   Returns: TRUE (-1) if FIFO is full, FALSE (0) otherwise
'   NOTE: If this flag is set, the oldest data has already been dropped from the FIFO
    readreg(core#INT_STATUS, 1, @flag)
    return (((flag >> core#FIFO_OVERFL_INT) & 1) == 1)

PUB fifo_read(nr_bytes, ptr_data)
' Read FIFO data
    readreg(core#FIFO_R_W, nr_bytes, ptr_data)

PUB fifo_reset{} | tmp
' Reset the FIFO    XXX - expand..what exactly does it do?
    tmp := 1 << core#FIFO_RST
    writereg(core#USER_CTRL, 1, @tmp)

PUB fifo_src(mask): curr_mask
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
    case mask
        %00000000..%11111111:
            writereg(core#FIFO_EN, 1, @mask)
        other:
            curr_mask := 0
            readreg(core#FIFO_EN, 1, @curr_mask)
            return

PUB fifo_nr_unread{}: nr_samples
' Number of unread samples stored in FIFO
'   Returns: unsigned 13bit
    readreg(core#FIFO_COUNTH, 2, @nr_samples)

PUB fsync_polarity(state): curr_state
' Set FSYNC pin active state/logic level
'   Valid values: LOW (1), *HIGH (0)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(core#INT_BYPASS_CFG, 1, @curr_state)
    case state
        LOW, HIGH:
            state := state << core#FSYNC_INT_LVL
        other:
            return (curr_state >> core#FSYNC_INT_LVL) & 1

    state := ((curr_state & core#FSYNC_INT_LVL_MASK) | state) & core#INT_BYPASS_CFG_MASK
    writereg(core#INT_BYPASS_CFG, 1, @state)

PUB gyro_axis_ena(xyz_mask): curr_mask
' Enable data output for Gyroscope - per axis
'   Valid values: 0 or 1, for each axis:
'       Bits    210
'               XYZ
'   Any other value polls the chip and returns the current setting
    curr_mask := 0
    readreg(core#PWR_MGMT_2, 1, @curr_mask)
    case xyz_mask
        %000..%111:
            ' invert bits because the logic in the chip is actually the reverse
            ' of the method name, i.e., a bit set to 1 _disables_ that axis
            xyz_mask := ((xyz_mask ^ core#STBY_INVERT) & core#STBY_XYZG_BITS) << core#STBY_XYZG
        other:
            return ((curr_mask >> core#STBY_XYZG) & core#STBY_XYZG_BITS) ^ core#STBY_INVERT

    xyz_mask := ((curr_mask & core#STBY_XYZG_MASK) | xyz_mask)
    writereg(core#PWR_MGMT_2, 1, @xyz_mask)

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
    readreg(core#GYRO_XOUT_H, 6, @tmp)

    long[ptr_x] := ~~tmp.word[2]
    long[ptr_y] := ~~tmp.word[1]
    long[ptr_z] := ~~tmp.word[0]

PUB gyro_data_rate(rate): curr_rate
' Set gyroscope output data rate, in Hz
'   Valid values: 4..1000
'   Any other value polls the chip and returns the current setting
    return xlg_data_rate(rate)

PUB gyro_data_rdy{}: flag
' Flag indicating new gyroscope data available
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    return xlg_data_rdy{}

PUB gyro_lpf_freq(freq): curr_freq
' Set gyroscope output data low-pass filter cutoff frequency, in Hz
'   Valid values: 5, 10, 21, 44, 94, 184, 260 
'   Any other value polls the chip and returns the current setting
    curr_freq := lpf_byp_bits := 0
    readreg(core#CONFIG, 1, @curr_freq)
    case freq
        5, 10, 21, 44, 94, 184, 260:
            freq := lookdownz(freq: 260, 184, 94, 44, 21, 10, 5)
        other:
            return lookup(curr_freq & core#DLPF_CFG_BITS: 260, 184, 94, 44, 21, 10, 5)

    freq := (curr_freq & core#DLPF_CFG_MASK) | freq
    writereg(core#CONFIG, 1, @freq)

PUB gyro_scale(scale): curr_scl
' Set gyroscope full-scale range, in degrees per second
'   Valid values: *250, 500, 1000, 2000
'   Any other value polls the chip and returns the current setting
    curr_scl := 0
    readreg(core#GYRO_CFG, 1, @curr_scl)
    case scale
        250, 500, 1000, 2000:
            scale := lookdownz(scale: 250, 500, 1000, 2000) << core#GYRO_FS_SEL
            _gres := lookupz(scale >> core#GYRO_FS_SEL: 7633, 15_267, 30_487, 60_975)
            ' (1/131, 1/65.5, 1/32.8, 1/16.4) * 1_000_000
        other:
            curr_scl := (curr_scl >> core#GYRO_FS_SEL) & core#GYRO_FS_SEL_BITS
            return lookupz(curr_scl: 250, 500, 1000, 2000)

    scale := ((curr_scl & core#GYRO_FS_SEL_MASK) | scale)
    writereg(core#GYRO_CFG, 1, @scale)

PUB int_polarity(state): curr_state
' Set interrupt pin active state/logic level
'   Valid values: LOW (1), *HIGH (0)
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(core#INT_BYPASS_CFG, 1, @curr_state)
    case state
        LOW, HIGH:
            state := state << core#LEVEL
        other:
            return ((curr_state >> core#LEVEL) & 1)

    state := ((curr_state & core#LEVEL_MASK) | state) & core#INT_BYPASS_CFG_MASK
    writereg(core#INT_BYPASS_CFG, 1, @state)

PUB int_clear_mode(mode): curr_mode
' Select mode by which interrupt status may be cleared
'   Valid values:
'      *READ_INT_FLAG (0): Only by reading interrupt flags
'       ANY (1): By any read operation
'   Any other value polls the chip and returns the current setting
    curr_mode := 0
    readreg(core#INT_BYPASS_CFG, 1, @curr_mode)
    case mode
        ANY, READ_INT_FLAG:
            mode := mode << core#INT_RD_CLEAR
        other:
            return ((curr_mode >> core#INT_RD_CLEAR) & 1)

    mode := ((curr_mode & core#INT_RD_CLEAR_MASK) | mode) & core#INT_BYPASS_CFG_MASK
    writereg(core#INT_BYPASS_CFG, 1, @mode)

PUB interrupt{}: flag
' Indicates one or more interrupts have been asserted
'   Returns: non-zero result if any interrupts have been asserted:
'       INT_WAKE_ON_MOTION (64) - Wake on motion interrupt occurred
'       INT_FIFO_OVERFL (16) - FIFO overflowed
'       INT_FSYNC (8) - FSYNC interrupt occurred
'       INT_SENSOR_READY (1) - Sensor raw data updated
    flag := 0
    readreg(core#INT_STATUS, 1, @flag)

PUB int_latch_ena(state): curr_state
' Latch interrupt pin when interrupt asserted
'   Valid values:
'      *FALSE (0): Interrupt pin is pulsed (width = 50uS)
'       TRUE (-1): Interrupt pin is latched, and must be cleared explicitly
'   Any other value polls the chip and returns the current setting
    curr_state := 0
    readreg(core#INT_BYPASS_CFG, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#LATCH_INT_EN
        other:
            return (((curr_state >> core#LATCH_INT_EN) & 1) == 1)

    state := ((curr_state & core#LATCH_INT_EN_MASK) | state) & core#INT_BYPASS_CFG_MASK
    writereg(core#INT_BYPASS_CFG, 1, @state)

PUB int_mask(mask): curr_mask
' Allow interrupts to assert INT pin, set by mask, or by ORing together symbols shown below
'   Valid values:
'       Bits: %x6x43xx0 (bit positions marked 'x' aren't supported by the device; setting any of them to '1' will be considered invalid and will query the current setting, instead)
'               Function                                Symbol              Value
'           6: Enable interrupt for wake on motion      INT_WAKE_ON_MOTION (64)
'           4: Enable interrupt for FIFO overflow       INT_FIFO_OVERFL  (16)
'           3: Enable FSYNC interrupt                   INT_FSYNC           (8)
'           1: Enable raw Sensor Data Ready interrupt   INT_SENSOR_READY    (1)
'   Any other value polls the chip and returns the current setting
    case mask & (core#INT_ENABLE_MASK ^ $FF)    ' check for any invalid bits:
        0:                                      ' result should be 0 if all ok
            mask &= core#INT_ENABLE_MASK
            writereg(core#INT_ENABLE, 1, @mask)
        other:                                  ' one or more invalid bits;
            curr_mask := 0                      ' return current setting
            readreg(core#INT_ENABLE, 1, @curr_mask)
            return curr_mask & core#INT_ENABLE_MASK

PUB int_outp_type(mode): curr_mode
' Set interrupt pin output mode
'   Valid values:
'      *INT_PP (0): Push-pull
'       INT_OD (1): Open-drain
'   Any other value polls the chip and returns the current setting
    curr_mode := 0
    readreg(core#INT_BYPASS_CFG, 1, @curr_mode)
    case mode
        INT_PP, INT_OD:
            mode := mode << core#OPEN
        other:
            return ((curr_mode >> core#OPEN) & 1)

    mode := ((curr_mode & core#OPEN_MASK) | mode) & core#INT_BYPASS_CFG_MASK
    writereg(core#INT_BYPASS_CFG, 1, @mode)

PUB reset{} | tmp
' Perform soft-reset
    tmp := core#XLG_SOFT_RST
    writereg(core#PWR_MGMT_1, 1, @tmp)

PUB sleep(state): curr_state
' Enable low-power sleep mode
    curr_state := 0
    readreg(core#PWR_MGMT_1, 1, @curr_state)
    case state
        0, 1:
            state := state << core#SLEEP
        other:
            return (((curr_state >> core#SLEEP) & 1) == 1)

    state := ((curr_state & core#SLEEP_MASK) | state)
    writereg(core#PWR_MGMT_1, 1, @state)

PUB temp_data_rate(rate): curr_rate
' Set temperature output data rate, in Hz
'   Valid values: 4..1000
'   Any other value polls the chip and returns the current setting
'   NOTE: This setting affects the accelerometer and gyroscope data rate
'   (hardware limitation)
    return xlg_data_rate(rate)

PUB temperature{}: temp
' Read temperature, in hundredths of a degree
    temp := 0
    readreg(core#TEMP_OUT_H, 2, @temp)
    case _temp_scale
        F:
        other:
            return ((temp * 1_0000) / 333_87) + 21_00 'XXX unverified

PUB temp_scale(scale): curr_scl
' Set temperature scale used by Temperature method
'   Valid values:
'       C (0): Celsius
'       F (1): Fahrenheit
'   Any other value returns the current setting
    case scale
        C, F:
            _temp_scale := scale
        other:
            return _temp_scale

PUB xlg_data_rate(rate): curr_rate
' Set accelerometer/gyro/temp sensor output data rate, in Hz
'   Valid values: 4..1000
'   Any other value polls the chip and returns the current setting
    case rate
        4..1000:
            rate := (1000 / rate) - 1
            writereg(core#SMPLRT_DIV, 1, @rate)
        other:
            curr_rate := 0
            readreg(core#SMPLRT_DIV, 1, @curr_rate)
            return 1000 / (curr_rate + 1)

PUB xlg_data_rdy{}: flag
' Flag indicating new gyroscope/accelerometer data is ready to be read
'   Returns: TRUE (-1) if new data available, FALSE (0) otherwise
    flag := 0
    readreg(core#INT_STATUS, 1, @flag)
    return ((flag & 1) == 1)

PRI readreg(reg_nr, nr_bytes, ptr_buff) | cmd_pkt
' Read nr_bytes from the slave device ptr_buff
    case reg_nr                                 ' validate reg
        core#SELF_TEST_X..core#SELF_TEST_A, {
}       core#SMPLRT_DIV..core#ACCEL_CFG, {
}       core#FIFO_EN..core#INT_ENABLE, {
}       core#INT_STATUS..core#EXT_SENS_DATA_23, {
}       core#I2C_SLV0_DO..core#SIGNAL_PATH_RESET, {
}       core#USER_CTRL..core#PWR_MGMT_2, {
}       core#FIFO_COUNTH..core#WHO_AM_I:
            { accel/gyro regs }
            cmd_pkt.byte[0] := (SLAVE_WR | _addr_bits)
            cmd_pkt.byte[1] := reg_nr.byte[0]
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start{}
            i2c.write(SLAVE_RD | _addr_bits)
            i2c.rdblock_msbf(ptr_buff, nr_bytes, i2c#NAK)
            i2c.stop{}
        other:
            return

PRI writereg(reg_nr, nr_bytes, ptr_buff) | cmd_pkt
' Write nr_bytes to the slave device from ptr_buff
    case reg_nr                                 ' validate reg
        core#SELF_TEST_X..core#SELF_TEST_A,{
}       core#SMPLRT_DIV..core#ACCEL_CFG, core#FIFO_EN..core#I2C_SLV4_CTRL,{
}       core#INT_PIN_CFG, core#INT_ENABLE, {
}       core#I2C_SLV0_DO..core#SIGNAL_PATH_RESET, core#USER_CTRL..core#PWR_MGMT_2, {
}       core#FIFO_COUNTH..core#FIFO_R_W:
            { accel/gyro regs }
            cmd_pkt.byte[0] := (SLAVE_WR | _addr_bits)
            cmd_pkt.byte[1] := reg_nr.byte[0]
            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.wrblock_msbf(ptr_buff, nr_bytes)
            i2c.stop{}
        other:
            return


DAT
{
Copyright 2022 Jesse Burt

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

