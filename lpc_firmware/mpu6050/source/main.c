#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "fsl_common.h"
#include "fsl_gpio.h"

#define MPU6050_I2C_ADDRESS   0x68u /*!< I2C address with AD0 pin low */
#define MPU6050_I2C_ADDRESS_1 0x69u /*!< I2C address with AD0 pin high */
#define MPU6050_WHO_AM_I_VAL  0x68u

/* MPU6050 register */
#define MPU6050_GYRO_CONFIG  0x1Bu
#define MPU6050_ACCEL_CONFIG 0x1Cu
#define MPU6050_INTR_PIN_CFG 0x37u
#define MPU6050_INTR_ENABLE  0x38u
#define MPU6050_INTR_STATUS  0x3Au
#define MPU6050_ACCEL_XOUT_H 0x3Bu
#define MPU6050_GYRO_XOUT_H  0x43u
#define MPU6050_TEMP_XOUT_H  0x41u
#define MPU6050_PWR_MGMT_1   0x6Bu
#define MPU6050_WHO_AM_I     0x75u
#define MPU6050_SMPRT_DIV    0x19u

#define MPU6050_DATA_RDY_INT_BIT      ((uint8_t)(0x01 << 0))
#define MPU6050_I2C_MASTER_INT_BIT    ((uint8_t)(0x01 << 3))
#define MPU6050_FIFO_OVERFLOW_INT_BIT ((uint8_t)(0x01 << 4))
#define MPU6050_MOT_DETECT_INT_BIT    ((uint8_t)(0x01 << 6))
#define MPU6050_ALL_INTERRUPTS        (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT)

typedef enum
{
    ACCEL_FS_2G  = 0, /*!< Accelerometer full scale range is +/- 2g */
    ACCEL_FS_4G  = 1, /*!< Accelerometer full scale range is +/- 4g */
    ACCEL_FS_8G  = 2, /*!< Accelerometer full scale range is +/- 8g */
    ACCEL_FS_16G = 3, /*!< Accelerometer full scale range is +/- 16g */
} mpu6050_accel_fs_t;

typedef enum
{
    GYRO_FS_250DPS  = 0, /*!< Gyroscope full scale range is +/- 250 degree per second */
    GYRO_FS_500DPS  = 1, /*!< Gyroscope full scale range is +/- 500 degree per second */
    GYRO_FS_1000DPS = 2, /*!< Gyroscope full scale range is +/- 1000 degree per second */
    GYRO_FS_2000DPS = 3, /*!< Gyroscope full scale range is +/- 2000 degree per second */
} mpu6050_gyro_fs_t;

typedef struct
{
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
} mpu6050_raw_accel_value_t;

typedef struct
{
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6050_raw_gyro_value_t;

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
} mpu6050_accel_value_t;

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_gyro_value_t;

mpu6050_accel_value_t accel_value;
mpu6050_gyro_value_t  gyro_value;
float                 gyro_sensitivity;
float                 accel_sensitivity;
bool                  sensor_data_ready = false;

static void MPU6050_Read(uint8_t reg_addr, uint8_t * const rx_buffer, uint32_t rx_size)
{
    status_t              status;
    i2c_master_transfer_t masterXfer = {
        .slaveAddress   = MPU6050_I2C_ADDRESS,
        .direction      = kI2C_Read,
        .subaddress     = reg_addr,
        .subaddressSize = 1,
        .data           = rx_buffer,
        .dataSize       = rx_size,
        .flags          = kI2C_TransferDefaultFlag,
    };
    status = I2C_MasterTransferBlocking(MPU6050_I2C_PERIPHERAL, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C read failed (%d)\r\n", status);
    }
}

static void MPU6050_Write(uint8_t reg_addr, uint8_t * const tx_buffer, uint32_t tx_size)
{
    status_t              status;
    i2c_master_transfer_t masterXfer = {
        .slaveAddress   = MPU6050_I2C_ADDRESS,
        .direction      = kI2C_Write,
        .subaddress     = reg_addr,
        .subaddressSize = 1,
        .data           = tx_buffer,
        .dataSize       = tx_size,
        .flags          = kI2C_TransferDefaultFlag,
    };
    status = I2C_MasterTransferBlocking(MPU6050_I2C_PERIPHERAL, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C write failed (%d)\r\n", status);
    }
}

void MPU6050_WakeUp(void)
{
    uint8_t tmp;
    MPU6050_Read(MPU6050_PWR_MGMT_1, &tmp, 1);
    tmp &= ~(uint8_t)(0x01 << 6);
    MPU6050_Write(MPU6050_PWR_MGMT_1, &tmp, 1);
}

void MPU6050_Sleep(void)
{
    uint8_t tmp;
    MPU6050_Read(MPU6050_PWR_MGMT_1, &tmp, 1);
    tmp |= (uint8_t)(0x01 << 6);
    MPU6050_Write(MPU6050_PWR_MGMT_1, &tmp, 1);
}

float MPU6050_GetAccelSensitivity(void)
{
    uint8_t accel_fs;
    float   accel_sensitivity = 0.0f;

    MPU6050_Read(MPU6050_ACCEL_CONFIG, &accel_fs, 1);
    accel_fs = (accel_fs >> 3) & 0x03;
    switch (accel_fs)
    {
    case ACCEL_FS_2G:
        accel_sensitivity = 16384;
        break;
    case ACCEL_FS_4G:
        accel_sensitivity = 8192;
        break;
    case ACCEL_FS_8G:
        accel_sensitivity = 4096;
        break;
    case ACCEL_FS_16G:
        accel_sensitivity = 2048;
        break;
    default:
        break;
    }
    return accel_sensitivity;
}

float MPU6050_GetGyroSensitivity(void)
{
    uint8_t gyro_fs;
    float   gyro_sensitivity = 0.0f;

    MPU6050_Read(MPU6050_ACCEL_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs)
    {
    case GYRO_FS_250DPS:
        gyro_sensitivity = 16384;
        break;
    case GYRO_FS_500DPS:
        gyro_sensitivity = 8192;
        break;
    case GYRO_FS_1000DPS:
        gyro_sensitivity = 4096;
        break;
    case GYRO_FS_2000DPS:
        gyro_sensitivity = 2048;
        break;
    default:
        break;
    }
    return gyro_sensitivity;
}

void MPU6050_GetRawAccel(mpu6050_raw_accel_value_t * const raw_accel_value)
{
    uint8_t data_rd[6];
    MPU6050_Read(MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_accel_value->raw_accel_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_accel_value->raw_accel_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_accel_value->raw_accel_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
}

void MPU6050_GetRawGyro(mpu6050_raw_gyro_value_t * const raw_gyro_value)
{
    uint8_t data_rd[6];
    MPU6050_Read(MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
}

void MPU6050_GetAccel(void)
{
    mpu6050_raw_accel_value_t raw_accel;
    MPU6050_GetRawAccel(&raw_accel);
    accel_value.accel_x = raw_accel.raw_accel_x / accel_sensitivity;
    accel_value.accel_y = raw_accel.raw_accel_y / accel_sensitivity;
    accel_value.accel_z = raw_accel.raw_accel_z / accel_sensitivity;
}

void MPU6050_GetGyro(void)
{
    mpu6050_raw_gyro_value_t raw_gyro;
    MPU6050_GetRawGyro(&raw_gyro);
    gyro_value.gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value.gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value.gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
}

void MPU6050_SetSampleRate(uint8_t rate)
{
    MPU6050_Write(MPU6050_SMPRT_DIV, &rate, 1);
}

void MPU6050_EnableInterrupts(uint8_t interrupt_sources)
{
    uint8_t enabled_interrupts = 0x00;

    MPU6050_Read(MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (enabled_interrupts != interrupt_sources)
    {
        enabled_interrupts |= interrupt_sources;
        MPU6050_Write(MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }
}

void MPU6050_DisableInterrupts(uint8_t interrupt_sources)
{
    uint8_t enabled_interrupts = 0x00;

    MPU6050_Read(MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (0 != (enabled_interrupts & interrupt_sources))
    {
        enabled_interrupts &= (~interrupt_sources);

        MPU6050_Write(MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }
}

static inline uint8_t MPU6050_GetInterruptStatus(void)
{
    uint8_t out_intr_status;
    MPU6050_Read(MPU6050_INTR_STATUS, &out_intr_status, 1);
    return out_intr_status;
}

void MPU6050_Init(const mpu6050_accel_fs_t accel_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t who_am_i;
    MPU6050_Read(MPU6050_WHO_AM_I, &who_am_i, 1);
    assert(who_am_i == MPU6050_WHO_AM_I_VAL);

    uint8_t config_regs[2] = {gyro_fs << 3, accel_fs << 3};
    MPU6050_Write(MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));

    gyro_sensitivity  = MPU6050_GetGyroSensitivity();
    accel_sensitivity = MPU6050_GetAccelSensitivity();

    MPU6050_SetSampleRate(0x01);

    MPU6050_EnableInterrupts(MPU6050_DATA_RDY_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT);

    MPU6050_WakeUp();
}

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    // PRINTF("MPU6050 example\r\n");
    MPU6050_Init(ACCEL_FS_16G, GYRO_FS_2000DPS);

    PINT_EnableCallbackByIndex(MPU6050_PINT_PERIPHERAL, kPINT_PinInt0);

    for (;;)
    {
        if (sensor_data_ready)
        {
            MPU6050_GetGyro();
            MPU6050_GetAccel();
            PRINTF("accel_x = %6.3f, accel_y = %6.3f, accel_z = %6.3f, gyro_x = %6.3f, gyro_y = %6.3f, gyro_z = %6.3f\r\n",
                   accel_value.accel_x, accel_value.accel_y, accel_value.accel_z,
                   gyro_value.gyro_x, gyro_value.gyro_y, gyro_value.gyro_z);
            sensor_data_ready = false;
            EnableIRQ(PIN_INT0_IRQn);
        }
    }
}

void MPU6050_PINTCallback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    if (MPU6050_PINT_INT == pintr)
    {
        DisableIRQ(PIN_INT0_IRQn);
        uint8_t mpu6050_status = MPU6050_GetInterruptStatus();

        if (MPU6050_DATA_RDY_INT_BIT & mpu6050_status)
        {
            sensor_data_ready = true;
            GPIO_PortToggle(GPIO, LED_BLUE_PORT, LED_BLUE_PIN_MASK);
        }
        if (MPU6050_FIFO_OVERFLOW_INT_BIT & mpu6050_status)
        {
            PRINTF("FIFO overflow\r\n");
        }
        if (MPU6050_MOT_DETECT_INT_BIT & mpu6050_status)
        {
            PRINTF("Motion detected\r\n");
        }
        if (MPU6050_I2C_MASTER_INT_BIT & mpu6050_status)
        {
            PRINTF("I2C master interrupt\r\n");
        }
    }
}
