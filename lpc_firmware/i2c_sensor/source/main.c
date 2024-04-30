#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MMA8652_I2C_ADDRESS 0x1Du

#define MMA8652_STATUS       0x00U
#define MMA8652_WHO_AM_I     0x0Du
#define MMA8652_CTRL_REG1    0x2Au
#define MMA8652_XYZ_DATA_CFG 0x0Eu

#define MMA8652_WHO_AM_I_VALUE 0x4Au

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t read_buffer[7];
int16_t x, y, z;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void ACCEL_Read(uint8_t reg_addr, uint8_t * const rx_buffer, uint32_t rx_size)
{
    status_t              status;
    i2c_master_transfer_t masterXfer = {
        .slaveAddress   = MMA8652_I2C_ADDRESS,
        .direction      = kI2C_Read,
        .subaddress     = reg_addr,
        .subaddressSize = 1,
        .data           = rx_buffer,
        .dataSize       = rx_size,
        .flags          = kI2C_TransferDefaultFlag,
    };
    status = I2C_MasterTransferBlocking(ACCEL_I2C_PERIPHERAL, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C read failed (%d)\r\n", status);
    }
}

static void ACCEL_Write(uint8_t reg_addr, uint8_t * const tx_buffer, uint32_t tx_size)
{
    status_t              status;
    i2c_master_transfer_t masterXfer = {
        .slaveAddress   = MMA8652_I2C_ADDRESS,
        .direction      = kI2C_Write,
        .subaddress     = reg_addr,
        .subaddressSize = 1,
        .data           = tx_buffer,
        .dataSize       = tx_size,
        .flags          = kI2C_TransferDefaultFlag,
    };
    status = I2C_MasterTransferBlocking(ACCEL_I2C_PERIPHERAL, &masterXfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C write failed (%d)\r\n", status);
    }
}

void ACCEL_Init(void)
{
    uint8_t who_am_i;
    ACCEL_Read(MMA8652_WHO_AM_I, &who_am_i, 1);
    assert(who_am_i == MMA8652_WHO_AM_I_VALUE);

    uint8_t ctrl_reg1 = 0x00;
    ACCEL_Write(MMA8652_CTRL_REG1, &ctrl_reg1, 1);

    uint8_t xyz_data_cfg = 0x01;
    ACCEL_Write(MMA8652_XYZ_DATA_CFG, &xyz_data_cfg, 1);

    ctrl_reg1 = 0x0d;
    ACCEL_Write(MMA8652_CTRL_REG1, &ctrl_reg1, 1);
}

void ACCEL_ReadSensor(void)
{
    /*  wait for new data are ready. */
    uint8_t status = 0x00u;
    while (status != 0xffu)
    {
        ACCEL_Read(MMA8652_STATUS, &status, 1);
    }

    /*  Multiple-byte Read from STATUS (0x00) register */
    ACCEL_Read(MMA8652_STATUS, read_buffer, 7);

    status = read_buffer[0];
    x      = ((int16_t)(((read_buffer[1] << 8) | read_buffer[2]))) >> 2;
    y      = ((int16_t)(((read_buffer[3] << 8) | read_buffer[4]))) >> 2;
    z      = ((int16_t)(((read_buffer[5] << 8) | read_buffer[6]))) >> 2;

    PRINTF("status = 0x%x , x = %5d , y = %5d , z = %5d \r\n", status, x, y, z);
}

/*!
 * @brief Main function
 */
int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    ACCEL_Init();

    for (;;)
    {
        ACCEL_ReadSensor();
    }
}
