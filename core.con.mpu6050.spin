{
    --------------------------------------------
    Filename: core.con.mpu6050.spin
    Author: Jesse Burt
    Description: MPU6050-specific constants
    Copyright (c) 2022
    Started Nov 5, 2022
    Updated Nov 6, 2022
    See end of file for terms of use.
    --------------------------------------------
}

CON

    I2C_MAX_FREQ                = 400_000
    SLAVE_ADDR                  = $68 << 1

    DEVID_RESP                  = $68

' Startup time
    TREGRW                      = 100_000       ' usec

' Accelerometer / Gyroscope registers
    SELF_TEST_X                 = $0D
    SELF_TEST_Y                 = $0E
    SELF_TEST_Z                 = $0F
    SELF_TEST_A                 = $10

    SMPLRT_DIV                  = $19

    CONFIG                      = $1A
    CONFIG_MASK                 = $3F
        EXT_SYNC_SET            = 3
        DLPF_CFG                = 0
        EXT_SYNC_SET_BITS       = %111
        DLPF_CFG_BITS           = %111
        EXT_SYNC_SET_MASK       = (EXT_SYNC_SET_BITS << EXT_SYNC_SET) ^ CONFIG_MASK
        DLPF_CFG_MASK           = (DLPF_CFG_BITS << DLPF_CFG) ^ CONFIG_MASK

    GYRO_CFG                    = $1B
    GYRO_CFG_MASK               = $F8
        XG_ST                   = 7
        YG_ST                   = 6
        ZG_ST                   = 5
        XYZG_ST                 = 4
        GYRO_FS_SEL             = 3
        GYRO_FS_SEL_BITS        = %11
        XYZG_ST_BITS            = %111
        XYZG_ST_MASK            = (XYZG_ST_BITS << XYZG_ST) ^ GYRO_CFG_MASK
        GYRO_FS_SEL_MASK        = (GYRO_FS_SEL_BITS << GYRO_FS_SEL) ^ GYRO_CFG_MASK

    ACCEL_CFG                   = $1C
    ACCEL_CFG_MASK              = $F8
        XA_ST                   = 7
        YA_ST                   = 6
        ZA_ST                   = 5
        XYZA_ST                 = 5
        AFS_SEL                 = 3
        AFS_SEL_BITS            = %11
        XYZA_ST_BITS            = %111
        XA_ST_MASK              = (1 << XA_ST) ^ ACCEL_CFG_MASK
        YA_ST_MASK              = (1 << YA_ST) ^ ACCEL_CFG_MASK
        ZA_ST_MASK              = (1 << ZA_ST) ^ ACCEL_CFG_MASK
        XYZA_ST_MASK            = (1 << XYZA_ST) ^ ACCEL_CFG_MASK
        AFS_SEL_MASK            = AFS_SEL_BITS ^ ACCEL_CFG_MASK

    FIFO_EN                     = $23
    FIFO_EN_MASK                = $FF
        TEMP_FIFO_EN            = 7
        XG_FIFO_EN              = 6
        YG_FIFO_EN              = 5
        ZG_FIFO_EN              = 4
        ACCEL_FIFO_EN           = 3
        SLV2_FIFO_EN            = 2
        SLV1_FIFO_EN            = 1
        SLV0_FIFO_EN            = 0

    I2C_MST_CTRL                = $24
    I2C_SLV0_ADDR               = $25
    I2C_SLV0_REG                = $26
    I2C_SLV0_CTRL               = $27
    I2C_SLV1_ADDR               = $28
    I2C_SLV1_REG                = $29
    I2C_SLV1_CTRL               = $2A
    I2C_SLV2_ADDR               = $2B
    I2C_SLV2_REG                = $2C
    I2C_SLV2_CTRL               = $2D
    I2C_SLV3_ADDR               = $2E
    I2C_SLV3_REG                = $2F
    I2C_SLV3_CTRL               = $30
    I2C_SLV4_ADDR               = $31
    I2C_SLV4_REG                = $32
    I2C_SLV4_DO                 = $33
    I2C_SLV4_CTRL               = $34
    I2C_SLV4_DI                 = $35
    I2C_MST_STATUS              = $36

    INT_PIN_CFG                 = $37
    INT_PIN_CFG_MASK            = $FE
        LEVEL                   = 7
        OPEN                    = 6
        LATCH_INT_EN            = 5
        INT_RD_CLEAR            = 4
        FSYNC_INT_LVL           = 3
        FSYNC_INT_EN            = 2
        I2C_BYPASS_EN           = 1
        LEVEL_MASK              = (1 << LEVEL) ^ INT_PIN_CFG_MASK
        OPEN_MASK               = (1 << OPEN) ^ INT_PIN_CFG_MASK
        LATCH_INT_EN_MASK       = (1 << LATCH_INT_EN) ^ INT_PIN_CFG_MASK
        INT_RD_CLEAR_MASK       = (1 << INT_RD_CLEAR) ^ INT_PIN_CFG_MASK
        FSYNC_INT_LVL_MASK      = (1 << FSYNC_INT_LVL) ^ INT_PIN_CFG_MASK
        FSYNC_INT_EN_MASK       = (1 << FSYNC_INT_EN) ^ INT_PIN_CFG_MASK
        I2C_BYPASS_EN_MASK      = (1 << I2C_BYPASS_EN) ^ INT_PIN_CFG_MASK

    INT_ENABLE                  = $38
    INT_ENABLE_MASK             = $19
        FIFO_OVERFL_EN          = 4
        I2C_MST_INT_EN          = 3
        DATA_RDY_EN             = 0
        FIFO_OVERFL_MASK        = (1 << FIFO_OVERFL_EN) ^ INT_ENABLE_MASK
        I2C_MST_INT_EN_MASK     = (1 << I2C_MST_INT_EN) ^ INT_ENABLE_MASK
        DATA_RDY_EN_MASK        = (1 << DATA_RDY_EN) ^ INT_ENABLE_MASK

    INT_STATUS                  = $3A
    INT_STATUS_MASK             = $19
        FIFO_OVERFL_INT         = 4
        I2C_MST_INT             = 3
        DATA_RDY_INT            = 0

    ACCEL_XOUT_H                = $3B
    ACCEL_XOUT_L                = $3C
    ACCEL_YOUT_H                = $3D
    ACCEL_YOUT_L                = $3E
    ACCEL_ZOUT_H                = $3F
    ACCEL_ZOUT_L                = $40

    TEMP_OUT_H                  = $41
    TEMP_OUT_L                  = $42

    GYRO_XOUT_H                 = $43
    GYRO_XOUT_L                 = $44
    GYRO_YOUT_H                 = $45
    GYRO_YOUT_L                 = $46
    GYRO_ZOUT_H                 = $47
    GYRO_ZOUT_L                 = $48

    EXT_SENS_DATA_00            = $49
    EXT_SENS_DATA_01            = $4A
    EXT_SENS_DATA_02            = $4B
    EXT_SENS_DATA_03            = $4C
    EXT_SENS_DATA_04            = $4D
    EXT_SENS_DATA_05            = $4E
    EXT_SENS_DATA_06            = $4F
    EXT_SENS_DATA_07            = $50
    EXT_SENS_DATA_08            = $51
    EXT_SENS_DATA_09            = $52
    EXT_SENS_DATA_10            = $53
    EXT_SENS_DATA_11            = $54
    EXT_SENS_DATA_12            = $55
    EXT_SENS_DATA_13            = $56
    EXT_SENS_DATA_14            = $57
    EXT_SENS_DATA_15            = $58
    EXT_SENS_DATA_16            = $59
    EXT_SENS_DATA_17            = $5A
    EXT_SENS_DATA_18            = $5B
    EXT_SENS_DATA_19            = $5C
    EXT_SENS_DATA_20            = $5D
    EXT_SENS_DATA_21            = $5E
    EXT_SENS_DATA_22            = $5F
    EXT_SENS_DATA_23            = $60

    I2C_SLV0_DO                 = $63
    I2C_SLV1_DO                 = $64
    I2C_SLV2_DO                 = $65
    I2C_SLV3_DO                 = $66

    I2C_MST_DELAY_CTRL          = $67

    SIGNAL_PATH_RESET           = $68
    SIGNAL_PATH_RESET_MASK      = $07
        GYRO_RESET              = 2
        ACCEL_RESET             = 1
        TEMP_RESET              = 0

    USER_CTRL                   = $6A
    USER_CTRL_MASK              = $77
        FIFOEN                  = 6
        I2C_MST_EN              = 5
        I2C_IF_DIS              = 4             ' 6000: DISABLE PRIMARY I2C, ENA SPI; 6050: ALWAYS WRITE 0
        FIFO_RST                = 2
        I2C_MST_RST             = 1
        SIG_COND_RST            = 0
        FIFOEN_MASK             = (1 << FIFOEN) ^ USER_CTRL_MASK
        I2C_MST_EN_MASK         = (1 << I2C_MST_EN) ^ USER_CTRL_MASK
        I2C_IF_DIS_MASK         = (1 << I2C_IF_DIS) ^ USER_CTRL_MASK
        FIFO_RST_MASK           = (1 << FIFO_RST) ^ USER_CTRL_MASK
        I2C_MST_RST_MASK        = (1 << I2C_MST_RST) ^ USER_CTRL_MASK
        SIG_COND_RST_MASK       = (1 << SIG_COND_RST) ^ USER_CTRL_MASK

    PWR_MGMT_1                  = $6B
    PWR_MGMT_1_MASK             = $EF
        DEV_RESET               = 7
        SLEEP                   = 6
        CYCLE                   = 5
        TEMP_DIS                = 3
        CLKSEL                  = 0
        CLKSEL_BITS             = %111
        DEV_RESET_MASK          = (1 << DEV_RESET) ^ PWR_MGMT_1_MASK
        SLEEP_MASK              = (1 << SLEEP) ^ PWR_MGMT_1_MASK
        CYCLE_MASK              = (1 << CYCLE) ^ PWR_MGMT_1_MASK
        TEMP_DIS_MASK           = (1 << TEMP_DIS) ^ PWR_MGMT_1_MASK
        CLKSEL_MASK             = CLKSEL_BITS ^ PWR_MGMT_1_MASK
        XLG_SOFT_RST            = 1 << DEV_RESET

    PWR_MGMT_2                  = $6C
    PWR_MGMT_2_MASK             = $3F
        LP_WAKE_CTRL            = 6
        STBY_XYZA               = 3
        STBY_XYZG               = 0
        STBY_XYZA_BITS          = %111
        STBY_XYZG_BITS          = %111
        STBY_XYZA_MASK          = (STBY_XYZA_BITS << STBY_XYZA) ^ PWR_MGMT_2_MASK
        STBY_XYZG_MASK          = (STBY_XYZG_BITS << STBY_XYZG) ^ PWR_MGMT_2_MASK
        STBY_INVERT             = %111

    FIFO_COUNTH                 = $72
    FIFO_COUNTL                 = $73

    FIFO_R_W                    = $74

    WHO_AM_I                    = $75
        WHO_AM_I_RESP           = $71

PUB null{}
' This is not a top-level object

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

