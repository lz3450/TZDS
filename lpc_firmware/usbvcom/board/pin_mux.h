/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define IOCON_PIO_DIGITAL_EN 0x0100u  /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC1 0x01u         /*!<@brief Selects pin function 1 */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x00u    /*!<@brief No addition pin function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u /*!<@brief Standard mode, output slew rate control is enabled */

/*! @name PIO0_29 (number 92), P8[2]/U6[13]/FC0_USART_RXD
  @{ */
/*!
 * @brief PORT peripheral base pointer */
#define BOARD_INITDEBUG_UARTPINS_DEBUG_UART_RX_PORT 0U
/*!
 * @brief PORT pin number */
#define BOARD_INITDEBUG_UARTPINS_DEBUG_UART_RX_PIN 29U
/*!
 * @brief PORT pin mask */
#define BOARD_INITDEBUG_UARTPINS_DEBUG_UART_RX_PIN_MASK (1U << 29U)
/* @} */

/*! @name PIO0_30 (number 94), P8[3]/U6[12]/FC0_USART_TXD
  @{ */
/*!
 * @brief PORT peripheral base pointer */
#define BOARD_INITDEBUG_UARTPINS_DEBUG_UART_TX_PORT 0U
/*!
 * @brief PORT pin number */
#define BOARD_INITDEBUG_UARTPINS_DEBUG_UART_TX_PIN 30U
/*!
 * @brief PORT pin mask */
#define BOARD_INITDEBUG_UARTPINS_DEBUG_UART_TX_PIN_MASK (1U << 30U)
/* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitDEBUG_UARTPins(void); /* Function assigned for the Cortex-M33 (Core #0) */

/*!
 * @brief Enables digital function */
#define IOCON_PIO_DIGITAL_EN 0x0100u
/*!
 * @brief Selects pin function 7 */
#define IOCON_PIO_FUNC7 0x07u
/*!
 * @brief Input function is not inverted */
#define IOCON_PIO_INV_DI 0x00u
/*!
 * @brief No addition pin function */
#define IOCON_PIO_MODE_INACT 0x00u
/*!
 * @brief Open drain is disabled */
#define IOCON_PIO_OPENDRAIN_DI 0x00u
/*!
 * @brief Standard mode, output slew rate control is enabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_28_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 7. */
#define PIO0_28_FUNC_ALT7 0x07u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_28_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_12_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 4. */
#define PIO1_12_FUNC_ALT4 0x04u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_12_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_29_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 4. */
#define PIO1_29_FUNC_ALT4 0x04u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_29_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_30_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 4. */
#define PIO1_30_FUNC_ALT4 0x04u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_30_MODE_PULL_UP 0x02u

/*! @name USB0_DP (number 97), P10[3]/D10[3]/USB0_FS_P
  @{ */
/* @} */

/*! @name USB0_DM (number 98), P10[2]/D10[2]/USB0_FS_N
  @{ */
/* @} */

/*! @name PIO0_22 (number 78), P10[1]/USB0_VBUS
  @{ */
#define USB0_VBUS_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define USB0_VBUS_PIN 22U                   /*!<@brief PORT pin number */
#define USB0_VBUS_PIN_MASK (1U << 22U)      /*!<@brief PORT pin mask */
                                            /* @} */

/*! @name USB1_DM (number 35), P9[2]/D9[2]/USB1_HS_N
  @{ */
/* @} */

/*! @name USB1_DP (number 34), P9[3]/D9[3]/USB1_HS_P
  @{ */
/* @} */

/*! @name USB1_VBUS (number 36), P9[1]/USB1_VBUS
  @{ */
/* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitUSBPins(void); /* Function assigned for the Cortex-M33 (Core #0) */

#define IOCON_PIO_DIGITAL_EN 0x0100u  /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC0 0x00u         /*!<@brief Selects pin function 0 */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_PULLUP 0x20u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u /*!<@brief Standard mode, output slew rate control is enabled */

/*! @name PIO1_4 (number 1), R78/P18[5]/LEDR/PWM_ARD
  @{ */

/* Symbols to be used with GPIO driver */
#define LED_BLUE_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define LED_BLUE_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */
#define LED_BLUE_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define LED_BLUE_PIN 4U                   /*!<@brief PORT pin number */
#define LED_BLUE_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*! @name PIO1_6 (number 5), R80/P18[9]/LEDB/PWM_ARD
  @{ */

/* Symbols to be used with GPIO driver */
#define LED_RED_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define LED_RED_GPIO_PIN_MASK (1U << 6U) /*!<@brief GPIO pin mask */
#define LED_RED_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define LED_RED_PIN 6U                   /*!<@brief PORT pin number */
#define LED_RED_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                         /* @} */

/*! @name PIO1_7 (number 9), R79/P18[7]/LEDG/PWM_ARD
  @{ */

/* Symbols to be used with GPIO driver */
#define LED_GREEN_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define LED_GREEN_GPIO_PIN_MASK (1U << 7U) /*!<@brief GPIO pin mask */
#define LED_GREEN_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define LED_GREEN_PIN 7U                   /*!<@brief PORT pin number */
#define LED_GREEN_PIN_MASK (1U << 7U)      /*!<@brief PORT pin mask */
                                           /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLEDsPins(void); /* Function assigned for the Cortex-M33 (Core #0) */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins_Core0(void); /* Function assigned for the Cortex-M33 (Core #0) */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/