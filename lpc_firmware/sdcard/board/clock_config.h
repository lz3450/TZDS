/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_XTAL0_CLK_HZ                         16000000U  /*!< Board xtal frequency in Hz */
#define BOARD_XTAL32K_CLK_HZ                          32768U  /*!< Board xtal32K frequency in Hz */

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes default configuration of clocks.
 *
 */
void BOARD_InitBootClocks(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ******************** Configuration BOARD_BootClockFRO12M **********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockFRO12M configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKFRO12M_CORE_CLOCK           12000000U  /*!< Core clock frequency: 12000000Hz */


/* Clock outputs (values are in Hz): */
#define BOARD_BOOTCLOCKFRO12M_ASYNCADC_CLOCK          0UL
#define BOARD_BOOTCLOCKFRO12M_CLKOUT_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_CTIMER0_CLOCK           0UL
#define BOARD_BOOTCLOCKFRO12M_CTIMER1_CLOCK           0UL
#define BOARD_BOOTCLOCKFRO12M_CTIMER2_CLOCK           0UL
#define BOARD_BOOTCLOCKFRO12M_CTIMER3_CLOCK           0UL
#define BOARD_BOOTCLOCKFRO12M_CTIMER4_CLOCK           0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM0_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM1_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM2_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM3_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM4_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM5_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM6_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_FXCOM7_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_HSLSPI_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_MCLK_CLOCK              0UL
#define BOARD_BOOTCLOCKFRO12M_OSC32KHZ_CLOCK          0UL
#define BOARD_BOOTCLOCKFRO12M_OSTIMER32KHZ_CLOCK      0UL
#define BOARD_BOOTCLOCKFRO12M_PLUCLKIN_CLOCK          0UL
#define BOARD_BOOTCLOCKFRO12M_PLU_GLITCH_12MHZ_CLOCK  0UL
#define BOARD_BOOTCLOCKFRO12M_PLU_GLITCH_1MHZ_CLOCK   0UL
#define BOARD_BOOTCLOCKFRO12M_RTC1HZ_CLOCK            0UL
#define BOARD_BOOTCLOCKFRO12M_RTC1KHZ_CLOCK           0UL
#define BOARD_BOOTCLOCKFRO12M_SCT_CLOCK               0UL
#define BOARD_BOOTCLOCKFRO12M_SDIO_CLOCK              0UL
#define BOARD_BOOTCLOCKFRO12M_SYSTICK0_CLOCK          0UL
#define BOARD_BOOTCLOCKFRO12M_SYSTICK1_CLOCK          0UL
#define BOARD_BOOTCLOCKFRO12M_SYSTEM_CLOCK            12000000UL
#define BOARD_BOOTCLOCKFRO12M_TRACE_CLOCK             0UL
#define BOARD_BOOTCLOCKFRO12M_USB0_CLOCK              0UL
#define BOARD_BOOTCLOCKFRO12M_USB1_PHY_CLOCK          0UL
#define BOARD_BOOTCLOCKFRO12M_UTICK_CLOCK             0UL
#define BOARD_BOOTCLOCKFRO12M_WDT_CLOCK               0UL

/*******************************************************************************
 * API for BOARD_BootClockFRO12M configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockFRO12M(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ******************* Configuration BOARD_BootClockFROHF96M *********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockFROHF96M configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKFROHF96M_CORE_CLOCK         96000000U  /*!< Core clock frequency: 96000000Hz */


/* Clock outputs (values are in Hz): */
#define BOARD_BOOTCLOCKFROHF96M_ASYNCADC_CLOCK        0UL
#define BOARD_BOOTCLOCKFROHF96M_CLKOUT_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_CTIMER0_CLOCK         0UL
#define BOARD_BOOTCLOCKFROHF96M_CTIMER1_CLOCK         0UL
#define BOARD_BOOTCLOCKFROHF96M_CTIMER2_CLOCK         0UL
#define BOARD_BOOTCLOCKFROHF96M_CTIMER3_CLOCK         0UL
#define BOARD_BOOTCLOCKFROHF96M_CTIMER4_CLOCK         0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM0_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM1_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM2_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM3_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM4_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM5_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM6_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_FXCOM7_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_HSLSPI_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_MCLK_CLOCK            0UL
#define BOARD_BOOTCLOCKFROHF96M_OSC32KHZ_CLOCK        0UL
#define BOARD_BOOTCLOCKFROHF96M_OSTIMER32KHZ_CLOCK    0UL
#define BOARD_BOOTCLOCKFROHF96M_PLUCLKIN_CLOCK        0UL
#define BOARD_BOOTCLOCKFROHF96M_PLU_GLITCH_12MHZ_CLOCK0UL
#define BOARD_BOOTCLOCKFROHF96M_PLU_GLITCH_1MHZ_CLOCK 0UL
#define BOARD_BOOTCLOCKFROHF96M_RTC1HZ_CLOCK          0UL
#define BOARD_BOOTCLOCKFROHF96M_RTC1KHZ_CLOCK         0UL
#define BOARD_BOOTCLOCKFROHF96M_SCT_CLOCK             0UL
#define BOARD_BOOTCLOCKFROHF96M_SDIO_CLOCK            0UL
#define BOARD_BOOTCLOCKFROHF96M_SYSTICK0_CLOCK        0UL
#define BOARD_BOOTCLOCKFROHF96M_SYSTICK1_CLOCK        0UL
#define BOARD_BOOTCLOCKFROHF96M_SYSTEM_CLOCK          96000000UL
#define BOARD_BOOTCLOCKFROHF96M_TRACE_CLOCK           0UL
#define BOARD_BOOTCLOCKFROHF96M_USB0_CLOCK            0UL
#define BOARD_BOOTCLOCKFROHF96M_USB1_PHY_CLOCK        0UL
#define BOARD_BOOTCLOCKFROHF96M_UTICK_CLOCK           0UL
#define BOARD_BOOTCLOCKFROHF96M_WDT_CLOCK             0UL

/*******************************************************************************
 * API for BOARD_BootClockFROHF96M configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockFROHF96M(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ******************** Configuration BOARD_BootClockPLL100M *********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockPLL100M configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKPLL100M_CORE_CLOCK         100000000U  /*!< Core clock frequency: 100000000Hz */


/* Clock outputs (values are in Hz): */
#define BOARD_BOOTCLOCKPLL100M_ASYNCADC_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL100M_CLKOUT_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_CTIMER0_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL100M_CTIMER1_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL100M_CTIMER2_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL100M_CTIMER3_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL100M_CTIMER4_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM0_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM1_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM2_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM3_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM4_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM5_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM6_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_FXCOM7_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_HSLSPI_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_MCLK_CLOCK             0UL
#define BOARD_BOOTCLOCKPLL100M_OSC32KHZ_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL100M_OSTIMER32KHZ_CLOCK     0UL
#define BOARD_BOOTCLOCKPLL100M_PLUCLKIN_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL100M_PLU_GLITCH_12MHZ_CLOCK 0UL
#define BOARD_BOOTCLOCKPLL100M_PLU_GLITCH_1MHZ_CLOCK  0UL
#define BOARD_BOOTCLOCKPLL100M_RTC1HZ_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL100M_RTC1KHZ_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL100M_SCT_CLOCK              0UL
#define BOARD_BOOTCLOCKPLL100M_SDIO_CLOCK             0UL
#define BOARD_BOOTCLOCKPLL100M_SYSTICK0_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL100M_SYSTICK1_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL100M_SYSTEM_CLOCK           100000000UL
#define BOARD_BOOTCLOCKPLL100M_TRACE_CLOCK            0UL
#define BOARD_BOOTCLOCKPLL100M_USB0_CLOCK             0UL
#define BOARD_BOOTCLOCKPLL100M_USB1_PHY_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL100M_UTICK_CLOCK            0UL
#define BOARD_BOOTCLOCKPLL100M_WDT_CLOCK              0UL

/*******************************************************************************
 * API for BOARD_BootClockPLL100M configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockPLL100M(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ******************** Configuration BOARD_BootClockPLL150M *********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockPLL150M configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKPLL150M_CORE_CLOCK         150000000U  /*!< Core clock frequency: 150000000Hz */


/* Clock outputs (values are in Hz): */
#define BOARD_BOOTCLOCKPLL150M_ASYNCADC_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL150M_CLKOUT_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_CTIMER0_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL150M_CTIMER1_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL150M_CTIMER2_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL150M_CTIMER3_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL150M_CTIMER4_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM0_CLOCK           48000000UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM1_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM2_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM3_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM4_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM5_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM6_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_FXCOM7_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_HSLSPI_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_MCLK_CLOCK             0UL
#define BOARD_BOOTCLOCKPLL150M_OSC32KHZ_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL150M_OSTIMER32KHZ_CLOCK     0UL
#define BOARD_BOOTCLOCKPLL150M_PLUCLKIN_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL150M_PLU_GLITCH_12MHZ_CLOCK 0UL
#define BOARD_BOOTCLOCKPLL150M_PLU_GLITCH_1MHZ_CLOCK  0UL
#define BOARD_BOOTCLOCKPLL150M_RTC1HZ_CLOCK           0UL
#define BOARD_BOOTCLOCKPLL150M_RTC1KHZ_CLOCK          0UL
#define BOARD_BOOTCLOCKPLL150M_SCT_CLOCK              0UL
#define BOARD_BOOTCLOCKPLL150M_SDIO_CLOCK             0UL
#define BOARD_BOOTCLOCKPLL150M_SYSTICK0_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL150M_SYSTICK1_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL150M_SYSTEM_CLOCK           150000000UL
#define BOARD_BOOTCLOCKPLL150M_TRACE_CLOCK            0UL
#define BOARD_BOOTCLOCKPLL150M_USB0_CLOCK             0UL
#define BOARD_BOOTCLOCKPLL150M_USB1_PHY_CLOCK         0UL
#define BOARD_BOOTCLOCKPLL150M_UTICK_CLOCK            0UL
#define BOARD_BOOTCLOCKPLL150M_WDT_CLOCK              0UL

/*******************************************************************************
 * API for BOARD_BootClockPLL150M configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockPLL150M(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _CLOCK_CONFIG_H_ */
