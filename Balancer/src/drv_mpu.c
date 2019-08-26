/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include "drv_mpu.h"
#include "hal_intf.h"

/* ****************************************************************************************************************** */

#define MPU_READ_LEN                14      // AXH|AXL|AYH|AYL|AZH|AZL|TH|TL|GXH|GXL|GYH|GYL|GZH|GZL
#define MPU_SPI_SPEED_KHZ_INIT      1000
#define MPU_SPI_SPEED_KHZ_READ      21500   // optimised speed (recommended speed is 20MHz)
#define MPU_WHOAMI_9255             0x73
#define MPU_ZERO_TOLERANCE_DPS      15      // MPU9255's gyroscope ZERO tolerance in 25°C is ±5

/**
 * MPU9250 register command
 */

#define MPUREG_RD(r)        (r | 0x80)
#define MPUREG_WR(r)        (r)

#define PWR_MGMT_1_H_RESET              (1 << 7)
#define PWR_MGMT_1_CLKSEL_AUTO          3

#define PWR_MGMT_2_GYRO_ACCEL_EN        0

#define SIGNAL_PATH_RESET_GYRO          (1 << 2)
#define SIGNAL_PATH_RESET_ACCEL         (1 << 1)
#define SIGNAL_PATH_RESET_TEMP          (1 << 0)

#define USER_CTRL_I2C_MST_EN            (1 << 5)
#define USER_CTRL_I2C_IF_DIS            (1 << 4)
#define USER_CTRL_FIFO_RST              (1 << 2)
#define USER_CTRL_I2C_MST_RST           (1 << 1)
#define USER_CTRL_SIG_COND_RST          (1 << 0)

#define SMPLRT_DIV_FULL                 0
#define SMPLRT_DIV_HALF                 1

#define CONFIG_DLPF_250HZ_970US         0   // 8K
#define CONFIG_DLPF_184HZ_2900US        1   // 1K
#define CONFIG_DLPF_3600HZ_170US        7   // 32K

#define GYRO_CONFIG_FS_SEL_500DPS       (0b01 << 3)
#define GYRO_CONFIG_FS_SEL_1000DPS      (0b10 << 3)
#define GYRO_CONFIG_FS_SEL_2000DPS      (0b11 << 3)
#define GYRO_CONFIG_FCHOICE_B_1K        0b00
#define GYRO_CONFIG_FCHOICE_B_8K        0b00

#define ACCEL_CONFIG_FS_SEL_2G          (0b00 << 3)
#define ACCEL_CONFIG_FS_SEL_4G          (0b01 << 3)
#define ACCEL_CONFIG_FS_SEL_8G          (0b10 << 3)
#define ACCEL_CONFIG_FS_SEL_16G         (0b11 << 3)

#define ACCEL_CONFIG2_FCHOICE_B_1K      (0 << 3)
#define ACCEL_CONFIG2_FCHOICE_B_4K      (1 << 3)
#define ACCEL_CONFIG2_DLPF_1046HZ_503US 0   // 4K
#define ACCEL_CONFIG2_DLPF_218HZ_1880US 1   // 1K

#define INT_PIN_CFG_LATCH_INT_EN        (1 << 5)
#define INT_PIN_CFG_ANYRD_2CLEAR        (1 << 4)

#define INT_STATUS_RAW_DATA_RDY_INT     (1 << 0)

#define INT_ENABLE_RAW_DRY_EN           (1 << 0)

#define I2C_MST_CTRL_400KHZ             13
#define I2C_MST_CTRL_500KHZ             9

#define I2C_SLVx_EN                     (1 << 7)
#define I2C_SLVx_LEN(x)                 ((x) & 0x0F)

#define I2C_SLV0_DLY_EN                 (1 << 0)

enum eRegs
{
    eRegs_SELF_TEST_X_GYRO   = 0x00,
    eRegs_SELF_TEST_Y_GYRO   = 0x01,
    eRegs_SELF_TEST_Z_GYRO   = 0x02,
    eRegs_SELF_TEST_X_ACCEL  = 0x0D,
    eRegs_SELF_TEST_Y_ACCEL  = 0x0E,
    eRegs_SELF_TEST_Z_ACCEL  = 0x0F,
    eRegs_XG_OFFSET_H        = 0x13,
    eRegs_XG_OFFSET_L        = 0x14,
    eRegs_YG_OFFSET_H        = 0x15,
    eRegs_YG_OFFSET_L        = 0x16,
    eRegs_ZG_OFFSET_H        = 0x17,
    eRegs_ZG_OFFSET_L        = 0x18,
    eRegs_SMPLRT_DIV         = 0x19,
    eRegs_CONFIG             = 0x1A,
    eRegs_GYRO_CONFIG        = 0x1B,
    eRegs_ACCEL_CONFIG       = 0x1C,
    eRegs_ACCEL_CONFIG2      = 0x1D,
    eRegs_LP_ACCEL_ODR       = 0x1E,
    eRegs_WOM_THR            = 0x1F,
    eRegs_FIFO_EN            = 0x23,
    eRegs_I2C_MST_CTRL       = 0x24,
    eRegs_I2C_SLV0_ADDR      = 0x25,
    eRegs_I2C_SLV0_REG       = 0x26,
    eRegs_I2C_SLV0_CTRL      = 0x27,
    eRegs_I2C_SLV1_ADDR      = 0x28,
    eRegs_I2C_SLV1_REG       = 0x29,
    eRegs_I2C_SLV1_CTRL      = 0x2A,
    eRegs_I2C_SLV2_ADDR      = 0x2B,
    eRegs_I2C_SLV2_REG       = 0x2C,
    eRegs_I2C_SLV2_CTRL      = 0x2D,
    eRegs_I2C_SLV3_ADDR      = 0x2E,
    eRegs_I2C_SLV3_REG       = 0x2F,
    eRegs_I2C_SLV3_CTRL      = 0x30,
    eRegs_I2C_SLV4_ADDR      = 0x31,
    eRegs_I2C_SLV4_REG       = 0x32,
    eRegs_I2C_SLV4_DO        = 0x33,
    eRegs_I2C_SLV4_CTRL      = 0x34,
    eRegs_I2C_SLV4_DI        = 0x35,
    eRegs_I2C_MST_STATUS     = 0x36,
    eRegs_INT_PIN_CFG        = 0x37,
    eRegs_INT_ENABLE         = 0x38,
    eRegs_INT_STATUS         = 0x3A,
    eRegs_ACCEL_XOUT_H       = 0x3B,
    eRegs_ACCEL_XOUT_L       = 0x3C,
    eRegs_ACCEL_YOUT_H       = 0x3D,
    eRegs_ACCEL_YOUT_L       = 0x3E,
    eRegs_ACCEL_ZOUT_H       = 0x3F,
    eRegs_ACCEL_ZOUT_L       = 0x40,
    eRegs_TEMP_OUT_H         = 0x41,
    eRegs_TEMP_OUT_L         = 0x42,
    eRegs_GYRO_XOUT_H        = 0x43,
    eRegs_GYRO_XOUT_L        = 0x44,
    eRegs_GYRO_YOUT_H        = 0x45,
    eRegs_GYRO_YOUT_L        = 0x46,
    eRegs_GYRO_ZOUT_H        = 0x47,
    eRegs_GYRO_ZOUT_L        = 0x48,
    eRegs_EXT_SENS_DATA_00   = 0x49,
    eRegs_EXT_SENS_DATA_01   = 0x4A,
    eRegs_EXT_SENS_DATA_02   = 0x4B,
    eRegs_EXT_SENS_DATA_03   = 0x4C,
    eRegs_EXT_SENS_DATA_04   = 0x4D,
    eRegs_EXT_SENS_DATA_05   = 0x4E,
    eRegs_EXT_SENS_DATA_06   = 0x4F,
    eRegs_EXT_SENS_DATA_07   = 0x50,
    eRegs_EXT_SENS_DATA_08   = 0x51,
    eRegs_EXT_SENS_DATA_09   = 0x52,
    eRegs_EXT_SENS_DATA_10   = 0x53,
    eRegs_EXT_SENS_DATA_11   = 0x54,
    eRegs_EXT_SENS_DATA_12   = 0x55,
    eRegs_EXT_SENS_DATA_13   = 0x56,
    eRegs_EXT_SENS_DATA_14   = 0x57,
    eRegs_EXT_SENS_DATA_15   = 0x58,
    eRegs_EXT_SENS_DATA_16   = 0x59,
    eRegs_EXT_SENS_DATA_17   = 0x5A,
    eRegs_EXT_SENS_DATA_18   = 0x5B,
    eRegs_EXT_SENS_DATA_19   = 0x5C,
    eRegs_EXT_SENS_DATA_20   = 0x5D,
    eRegs_EXT_SENS_DATA_21   = 0x5E,
    eRegs_EXT_SENS_DATA_22   = 0x5F,
    eRegs_EXT_SENS_DATA_23   = 0x60,
    eRegs_I2C_SLV0_DO        = 0x63,
    eRegs_I2C_SLV1_DO        = 0x64,
    eRegs_I2C_SLV2_DO        = 0x65,
    eRegs_I2C_SLV3_DO        = 0x66,
    eRegs_I2C_MST_DELAY_CTRL = 0x67,
    eRegs_SIGNAL_PATH_RESET  = 0x68,
    eRegs_MOT_DETECT_CTRL    = 0x69,
    eRegs_USER_CTRL          = 0x6A,
    eRegs_PWR_MGMT_1         = 0x6B,
    eRegs_PWR_MGMT_2         = 0x6C,
    eRegs_FIFO_COUNTH        = 0x72,
    eRegs_FIFO_COUNTL        = 0x73,
    eRegs_FIFO_R_W           = 0x74,
    eRegs_WHO_AM_I           = 0x75,
    eRegs_XA_OFFSET_H        = 0x77,
    eRegs_XA_OFFSET_L        = 0x78,
    eRegs_YA_OFFSET_H        = 0x7A,
    eRegs_YA_OFFSET_L        = 0x7B,
    eRegs_ZA_OFFSET_H        = 0x7D,
    eRegs_ZA_OFFSET_L        = 0x7E,
    eRegs_End
};
typedef enum eRegs          eRegs;

enum eMpuState
{
    eMpuState_Idle,
    eMpuState_Wait,
    eMpuState_Ready,
    eMpuState_N
};
typedef enum eMpuState      eMpuState;

struct sMpuData
{
    hHalSpi     _spi;
    uint8_t     _rx_buf[1 + MPU_READ_LEN];
    float       _gyro_factor;
    float       _accl_factor;
    uint32_t    _duration_us;
    uint32_t    _prev_readus;
    eMpuState   _state;
};
typedef struct sMpuData     sMpuData;

/* ****************************************************************************************************************** */

static sMpuData     mpu_data =
{
        ._spi         = NULL,
        ._gyro_factor = 0.0f,
        ._accl_factor = 0.0f,
        ._duration_us = 0,
        ._prev_readus = 0,
        ._state       = eMpuState_Idle
};

/* ****************************************************************************************************************** */

static void write_reg(sMpuData *this, uint8_t reg, uint8_t val)
{
    uint8_t     dat[2] = { MPUREG_WR(reg), val };
    hal_spi_lock(this->_spi);
    hal_spi_tx_timeout(this->_spi, dat, 2, 5);
    hal_spi_unlock(this->_spi);
}

static uint8_t read_reg(sMpuData *this, uint8_t reg)
{
    uint8_t     dat[2] = { MPUREG_RD(reg), 0 };
    hal_spi_lock(this->_spi);
    hal_spi_txrx_timeout(this->_spi, dat, dat, 2, 5);
    hal_spi_unlock(this->_spi);
    return dat[1];
}

static void spi_callback(int16_t len, bool error, void *param)
{
    ((sMpuData*)param)->_state = eMpuState_Ready;
}

bool drv_mpu_init(eDrvMpuMode mode)
{
    mpu_data._spi = hal_spi_init(eHalSpi_Port1, eHalGpio_MPU9255_nCS);
    if (mpu_data._spi != NULL)
    {
        hal_spi_set_speed(mpu_data._spi, MPU_SPI_SPEED_KHZ_INIT);
        hal_spi_unlock(mpu_data._spi);

        uint8_t     rate_div, config, gyro_cfg, accl_cfg, accl_cfg2;

        switch (mode)
        {
        default:
        case eDrvMpuMode_500:
            rate_div  = SMPLRT_DIV_HALF;
            config    = CONFIG_DLPF_250HZ_970US;
            gyro_cfg  = GYRO_CONFIG_FS_SEL_1000DPS | GYRO_CONFIG_FCHOICE_B_1K;
            accl_cfg  = ACCEL_CONFIG_FS_SEL_4G;
            accl_cfg2 = ACCEL_CONFIG2_FCHOICE_B_1K | ACCEL_CONFIG2_DLPF_218HZ_1880US;
            mpu_data._gyro_factor = 1000.0f / 32768.0f;
            mpu_data._accl_factor = 4.0f / 32768.0f;
            mpu_data._duration_us = 2000;
            break;
        case eDrvMpuMode_1K:
            rate_div  = SMPLRT_DIV_FULL;
            config    = CONFIG_DLPF_250HZ_970US;
            gyro_cfg  = GYRO_CONFIG_FS_SEL_1000DPS | GYRO_CONFIG_FCHOICE_B_1K;
            accl_cfg  = ACCEL_CONFIG_FS_SEL_4G;
            accl_cfg2 = ACCEL_CONFIG2_FCHOICE_B_1K | ACCEL_CONFIG2_DLPF_218HZ_1880US;
            mpu_data._gyro_factor = 1000.0f / 32768.0f;
            mpu_data._accl_factor = 4.0f / 32768.0f;
            mpu_data._duration_us = 1000;
            break;
        case eDrvMpuMode_8K:
            rate_div  = SMPLRT_DIV_FULL;
            config    = CONFIG_DLPF_250HZ_970US;
            gyro_cfg  = GYRO_CONFIG_FS_SEL_2000DPS | GYRO_CONFIG_FCHOICE_B_8K;
            accl_cfg  = ACCEL_CONFIG_FS_SEL_8G;
            accl_cfg2 = ACCEL_CONFIG2_FCHOICE_B_4K | ACCEL_CONFIG2_DLPF_1046HZ_503US;
            mpu_data._gyro_factor = 2000.0f / 32768.0f;
            mpu_data._accl_factor = 8.0f / 32768.0f;
            mpu_data._duration_us = 125;
            break;
        }

        write_reg(&mpu_data, eRegs_PWR_MGMT_1, PWR_MGMT_1_H_RESET);
        do { hal_delay_millis(1); } while (read_reg(&mpu_data, eRegs_PWR_MGMT_1) & PWR_MGMT_1_H_RESET);
        hal_delay_millis(3);

        if (read_reg(&mpu_data, eRegs_WHO_AM_I) != MPU_WHOAMI_9255)
            return false;

        write_reg(&mpu_data, eRegs_USER_CTRL,
                USER_CTRL_I2C_IF_DIS | USER_CTRL_FIFO_RST | USER_CTRL_I2C_MST_RST | USER_CTRL_SIG_COND_RST);
        do { hal_delay_millis(1); } while (read_reg(&mpu_data, eRegs_USER_CTRL) &
                (USER_CTRL_FIFO_RST | USER_CTRL_I2C_MST_RST | USER_CTRL_SIG_COND_RST));

        write_reg(&mpu_data, eRegs_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_PWR_MGMT_2, PWR_MGMT_2_GYRO_ACCEL_EN);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_SIGNAL_PATH_RESET,
                SIGNAL_PATH_RESET_GYRO | SIGNAL_PATH_RESET_ACCEL | SIGNAL_PATH_RESET_TEMP);
        hal_delay_millis(10);
        write_reg(&mpu_data, eRegs_SMPLRT_DIV, rate_div);
        hal_delay_millis(10);
        write_reg(&mpu_data, eRegs_CONFIG, config);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_GYRO_CONFIG, gyro_cfg);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_ACCEL_CONFIG, accl_cfg);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_ACCEL_CONFIG2, accl_cfg2);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_INT_PIN_CFG, INT_PIN_CFG_LATCH_INT_EN | INT_PIN_CFG_ANYRD_2CLEAR);
        hal_delay_millis(1);
        write_reg(&mpu_data, eRegs_INT_ENABLE, INT_ENABLE_RAW_DRY_EN);

        hal_spi_set_speed(mpu_data._spi, MPU_SPI_SPEED_KHZ_READ);
        mpu_data._prev_readus = hal_get_micros();
        return true;
    }
    return false;
}

inline float drv_mpu_get_gyro_factor(void)
{
    return mpu_data._gyro_factor;
}

inline float drv_mpu_get_accl_factor(void)
{
    return mpu_data._accl_factor;
}

float drv_mpu_get_gyro_tolernace(void)
{
    return (float)MPU_ZERO_TOLERANCE_DPS * (1.0f / mpu_data._gyro_factor);
}

bool drv_mpu_is_new(void)
{
    bool    is_new = false;
    if (mpu_data._state == eMpuState_Idle)
    {
        uint32_t    now_us = hal_get_micros();
        if (((now_us - mpu_data._prev_readus) >= mpu_data._duration_us) && (read_reg(&mpu_data, eRegs_INT_STATUS) & 0x01))
        {
            mpu_data._rx_buf[0] = MPUREG_RD(eRegs_ACCEL_XOUT_H);
            hal_spi_lock(mpu_data._spi);
            hal_spi_txrx(mpu_data._spi, mpu_data._rx_buf, mpu_data._rx_buf, 1 + MPU_READ_LEN, spi_callback, &mpu_data);
            mpu_data._prev_readus = now_us;
            mpu_data._state =  eMpuState_Wait;
        }
    }
    else if (mpu_data._state == eMpuState_Ready)
    {
        hal_spi_unlock(mpu_data._spi);
        is_new = true;
    }
    return is_new;
}

void drv_mpu_get_motion(int16_t *gx, int16_t *gy, int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az)
{
    mpu_data._state = eMpuState_Idle;

    *ax = (int16_t)(mpu_data._rx_buf[ 1] << 8) | (int16_t)mpu_data._rx_buf[ 2];
    *ay = (int16_t)(mpu_data._rx_buf[ 3] << 8) | (int16_t)mpu_data._rx_buf[ 4];
    *az = (int16_t)(mpu_data._rx_buf[ 5] << 8) | (int16_t)mpu_data._rx_buf[ 6];

    *gx = (int16_t)(mpu_data._rx_buf[ 9] << 8) | (int16_t)mpu_data._rx_buf[10];
    *gy = (int16_t)(mpu_data._rx_buf[11] << 8) | (int16_t)mpu_data._rx_buf[12];
    *gz = (int16_t)(mpu_data._rx_buf[13] << 8) | (int16_t)mpu_data._rx_buf[14];
}

/* end of file ****************************************************************************************************** */
