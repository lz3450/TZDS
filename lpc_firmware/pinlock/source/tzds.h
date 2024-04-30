#ifndef _TZDS_H_
#define _TZDS_H_

#include "LPC55S69_cm33_core0.h"
#include "fsl_gpio.h"

#define TZDS_RW __attribute__((annotate("tzds")))
#define TZDS_R __attribute__((annotate("tzds.read")))
#define TZDS_W __attribute__((annotate("tzds.write")))
#define TZDS_DM_R __attribute__((annotate("tzds.dm.r")))
#define TZDS_DM_W __attribute__((annotate("tzds.dm.w")))
#define TZDS_DM_RW(base, size) __attribute__((annotate("tzds.dm.rw")))

GPIO_Type* gpio TZDS_DM_RW(GPIO_BASE, 0x1000) = (GPIO_Type*)GPIO_BASE;

#endif /* _TZDS_H_ */
