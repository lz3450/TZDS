#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "fsl_gpio.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* L3G4200D Connection */
/*************************
 * GND----------------GND
 * VCC----------------3.3V
 * SCL----------------D13
 * SDA----------------D11
 * SDO----------------D12
 * CS-----------------D10
 * INT2---------------D6
 * INT1---------------D7
 *************************/

/* L3G4200D Registers */
#define WHO_AM_I      0x0F
#define CTRL_REG1     0x20
#define CTRL_REG2     0x21
#define CTRL_REG3     0x22
#define CTRL_REG4     0x23
#define CTRL_REG5     0x24
#define REFERENCE     0x25
#define OUT_TEMP      0x26
#define STATUS_REG    0x27
#define OUT_X_L       0x28
#define OUT_X_H       0x29
#define OUT_Y_L       0x2A
#define OUT_Y_H       0x2B
#define OUT_Z_L       0x2C
#define OUT_Z_H       0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG  0x2F
#define INT1_CFG      0x30
#define INT1_SRC      0x31
#define INT1_TSH_XH   0x32
#define INT1_TSH_XL   0x33
#define INT1_TSH_YH   0x34
#define INT1_TSH_YL   0x35
#define INT1_TSH_ZH   0x36
#define INT1_TSH_ZL   0x37
#define INT1_DURATION 0x38

bool    sensorReady = false;
int16_t x, y, z;
// uint8_t temp;

uint8_t L3G4200D_ReadRegister(uint8_t address)
{
    status_t status;
    // This tells the L3G4200D we're reading
    uint8_t        txData[2] = {address | 0x80, 0x00};
    uint8_t        rxData[2] = {0};
    spi_transfer_t xfer      = {
             .txData      = txData,
             .rxData      = rxData,
             .dataSize    = 2,
             .configFlags = kSPI_FrameAssert,
    };

    status = SPI_MasterTransferBlocking(SPI_PERIPHERAL, &xfer);
    assert(status == kStatus_Success);

    return rxData[1];
}

void L3G4200D_WriteRegister(uint8_t address, uint8_t data)
{
    status_t status;
    // This to tell the L3G4200D we're writing
    uint8_t        txData[2] = {address & 0x7F, data};
    uint8_t        rxData[2] = {0};
    spi_transfer_t xfer      = {
             .txData      = txData,
             .rxData      = rxData,
             .dataSize    = 2,
             .configFlags = kSPI_FrameAssert,
    };

    status = SPI_MasterTransferBlocking(SPI_PERIPHERAL, &xfer);
    assert(status == kStatus_Success);
}

void L3G4200D_EnableINT2(void)
{
    NVIC_SetPriority(PIN_INT0_IRQn, 1);
    PINT_EnableCallbackByIndex(L3G4200D_INT2_PERIPHERAL, kPINT_PinInt0);
}

/* Configure L3G4200 with selectable full scale range
 * 0: 250 dps
 * 1: 500 dps
 * 2: 2000 dps
 */
void L3G4200D_Init(uint8_t fullScale)
{
    uint8_t id;

    id = L3G4200D_ReadRegister(WHO_AM_I);
    assert(id == 0xD3);

    // Enable x, y, z and turn off power down:
    L3G4200D_WriteRegister(CTRL_REG1, 0b00001111);

    // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
    L3G4200D_WriteRegister(CTRL_REG2, 0b00000000);

    // Configure CTRL_REG3 to generate data ready interrupt on INT2
    // No interrupts used on INT1, if you'd like to configure INT1
    // or INT2 otherwise, consult the datasheet:
    L3G4200D_WriteRegister(CTRL_REG3, 0b00001000);

    // CTRL_REG4 controls the full-scale range, among other things:
    fullScale &= 0x03;
    L3G4200D_WriteRegister(CTRL_REG4, fullScale << 4);

    // CTRL_REG5 controls high-pass filtering of outputs, use it if you'd like:
    L3G4200D_WriteRegister(CTRL_REG5, 0b00000000);
}

void L3G4200D_GetGyroValues(void)
{
    x = L3G4200D_ReadRegister(OUT_X_L);
    x |= L3G4200D_ReadRegister(OUT_X_H) << 8;

    y = L3G4200D_ReadRegister(OUT_Y_L);
    y |= L3G4200D_ReadRegister(OUT_Y_H) << 8;

    z = L3G4200D_ReadRegister(OUT_Z_L);
    z |= L3G4200D_ReadRegister(OUT_Z_H) << 8;

    // temp = L3G4200D_ReadRegister(OUT_TEMP);
}

void ReadSensor(void)
{
    // PRINTF("%d\r\n", GPIO_PinRead(GPIO, L3G4200D_INT2_PORT, L3G4200D_INT2_PIN));
    if (sensorReady)
    {
        // PRINTF("x: %8d, y: %8d, z: %8d, temp: %8.3f\r\n", x, y, z, (float)temp * ((85.0f + 40.0f) / 255.0f));
        PRINTF("x = %8.3f, y = %8.3f, z = %8.3f\r\n", (float)x * 8.75 / 1000, (float)y * 8.75 / 1000, (float)z * 8.75 / 1000);
        sensorReady = false;
    }
    // delay(100);
    // while (!GPIO_PinRead(GPIO, L3G4200D_INT2_PORT, L3G4200D_INT2_PIN)) {}
    // L3G4200D_GetGyroValues();
    // PRINTF("x: %8d, y: %8d, z: %8d\r\n", x, y, z);
}

void ReadSensorTask(void *pvParameters)
{
    for (;;)
    {
        ReadSensor();
    }
}

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    if (pdPASS != xTaskCreate(ReadSensorTask,
                              "ReadSensorTask",
                              configMINIMAL_STACK_SIZE + 64,
                              NULL,
                              configMAX_PRIORITIES - 1,
                              NULL))
    {
        for (;;) {}
    }

    vTaskStartScheduler();
    for (;;) {}
}

void L3G4200D_SensorReadyCallback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    if (kPINT_PinInt0 == pintr)
    {
        GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PORT, LED_BLUE_PIN, 0);
        L3G4200D_GetGyroValues();
        sensorReady = true;
        GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PORT, LED_BLUE_PIN, 1);
    }
}
