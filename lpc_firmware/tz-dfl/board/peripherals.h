/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_pint.h"
#include "fsl_lpadc.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "fsl_debug_console.h"
#include "fsl_usart.h"
#include "fsl_clock.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals_cm33_core0 functional group */
/* BOARD_InitPeripherals_cm33_core0 defines for PINT */
/* Definition of peripheral ID */
#define PINT_PERIPHERAL ((PINT_Type *) PINT_BASE)
/* PINT interrupt vector ID (number). */
#define PINT_PINT_0_IRQN PIN_INT0_IRQn
/* Definition of PINT interrupt ID for interrupt 0  */
#define PINT_S3 kPINT_PinInt0
/* Alias for ADC0 peripheral */
#define ADC0_PERIPHERAL ADC0
/* ADC0 interrupt vector ID (number). */
#define ADC0_IRQN ADC0_IRQn
/* ADC0 interrupt handler identifier. */
#define ADC0_IRQHANDLER ADC0_IRQHandler
/* ADC0 OFSTRIM_A value for the offset calibration */
#define ADC0_OFSTRIM_A 16U
/* ADC0 OFSTRIM_B value for the offset calibration */
#define ADC0_OFSTRIM_B 16U
/* Debug console is initialized in the peripheral tool */
#define BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL 
/* Definition of serial peripheral */
#define DEBUGCONSOLE_SERIAL_PERIPHERAL USART0
/* Definition of serial peripheral instance */
#define DEBUGCONSOLE_INSTANCE 0U
/* Definition of serial peripheral type */
#define DEBUGCONSOLE_TYPE kSerialPort_Uart
/* Definition of the Baud rate */
#define DEBUGCONSOLE_BAUDRATE 115200UL
/* Definition of the clock source frequency */
#define DEBUGCONSOLE_CLK_FREQ 48000000UL

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const lpadc_config_t ADC0_config;
extern lpadc_conv_command_config_t ADC0_commandsConfig[1];
extern lpadc_conv_trigger_config_t ADC0_triggersConfig[1];

/***********************************************************************************************************************
 * Callback functions
 **********************************************************************************************************************/
/* S3 callback function for the PINT component */
extern void UserButton(pint_pin_int_t pintr, uint32_t pmatch_status);

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/

void BOARD_InitPeripherals_cm33_core0(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
