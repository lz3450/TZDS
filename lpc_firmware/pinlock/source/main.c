#include "fsl_common.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "fsl_hashcrypt.h"

#include "tzds.h"

#define BUFFER_SIZE (16 * 4 - 1)

// "123456"
const uint8_t key_stored[32] TZDS_R = {
    0x8d,
    0x96,
    0x9e,
    0xef,
    0x6e,
    0xca,
    0xd3,
    0xc2,
    0x9a,
    0x3a,
    0x62,
    0x92,
    0x80,
    0xe6,
    0x86,
    0xcf,
    0x0c,
    0x3f,
    0x5d,
    0x5a,
    0x86,
    0xaf,
    0xf3,
    0xca,
    0x12,
    0x02,
    0x0c,
    0x92,
    0x3a,
    0xdc,
    0x6c,
    0x92,
};

uint8_t           buffer[BUFFER_SIZE] TZDS_RW = {0};
volatile uint32_t rx_index            = 0;
volatile bool     pin_received        = false;

void lock(void)
{
    GPIO_PinWrite(LEDS_GPIO, LED_RED_PORT, LED_RED_PIN, 0);
    GPIO_PinWrite(LEDS_GPIO, LED_GREEN_PORT, LED_GREEN_PIN, 1);
}

void unlock(void)
{
    GPIO_PinWrite(LEDS_GPIO, LED_RED_PORT, LED_RED_PIN, 1);
    GPIO_PinWrite(LEDS_GPIO, LED_GREEN_PORT, LED_GREEN_PIN, 0);
}

void clear_buffer(void)
{
    rx_index = 0;
    (void)memset(buffer, 0, BUFFER_SIZE);
}

bool match(uint8_t *key_received)
{
    if (memcmp(key_received, key_stored, 32) == 0)
        return true;
    return false;
}

void check(void)
{
    uint8_t  key_received[32];
    status_t status;
    size_t   output_size;

    if (!pin_received)
        return;

    // USART_WriteByte(DEBUG_USART_PERIPHERAL, '\r');
    // USART_WriteByte(DEBUG_USART_PERIPHERAL, '\n');
    // USART_WriteBlocking(DEBUG_USART_PERIPHERAL, buffer, rx_index);
    // USART_WriteByte(DEBUG_USART_PERIPHERAL, '\r');
    // USART_WriteByte(DEBUG_USART_PERIPHERAL, '\n');

    DisableIRQ(DEBUG_USART_FLEXCOMM_IRQN);

    status = HASHCRYPT_SHA(HASHCRYPT, kHASHCRYPT_Sha256, buffer, rx_index, key_received, &output_size);
    assert(kStatus_Success == status);
    assert(output_size == 32u);

    // char hex_string[3];
    // for (int i = 0; i < 32; i++)
    // {
    //     sprintf(hex_string, "%02x", key_received[i]);
    //     USART_WriteBlocking(DEBUG_USART_PERIPHERAL, (uint8_t *)hex_string, 2);
    // }
    // USART_WriteByte(DEBUG_USART_PERIPHERAL, '\r');
    // USART_WriteByte(DEBUG_USART_PERIPHERAL, '\n');

    if (match(key_received))
    {
        unlock();
        SDK_DelayAtLeastUs(3000000, BOARD_CLOCK_PLL150M_CORE_CLOCK);
        lock();
    }

    EnableIRQ(DEBUG_USART_FLEXCOMM_IRQN);

    clear_buffer();
    pin_received = false;
}

int main(void)
{
    /* Board pin init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    lock();

    for (;;)
    {
        check();
    }
}

/* FLEXCOMM0_IRQn interrupt handler */
void DEBUG_USART_IRQHandler(void)
{
    uint32_t status;
    /* Reading all interrupt flags of status registers */
    status = USART_GetStatusFlags(DEBUG_USART_PERIPHERAL);

    /* Flags can be cleared by reading the status register and reading/writing data registers/status registers.
      See the reference manual for details of each flag.
      The USART_ClearStatusFlags() function can be also used for clearing of flags .
      For example:
          USART_ClearStatusFlags(DEBUG_USART_PERIPHERAL, intStatus);
    */

    /* Place your code here */
    if (kUSART_RxFifoNotEmptyFlag & status)
    {
        // GPIO_PinWrite(LEDS_GPIO, LED_BLUE_PORT, LED_BLUE_PIN, 0);
        uint8_t data TZDS_RW;
        data = USART_ReadByte(DEBUG_USART_PERIPHERAL);
        // USART_WriteByte(DEBUG_USART_PERIPHERAL, data);
        if (data == '\r')
        {
            pin_received = true;
            return;
        }
        if (rx_index < BUFFER_SIZE)
        {
            buffer[rx_index] = data;
            rx_index++;
        }
        // GPIO_PinWrite(LEDS_GPIO, LED_BLUE_PORT, LED_BLUE_PIN, 1);
    }
}
