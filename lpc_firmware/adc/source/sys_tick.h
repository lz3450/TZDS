#ifndef _SYS_TICK_H_
#define _SYS_TICK_H_

#include "fsl_common.h"

void SysTick_Init(void);
void SysTick_DelayTicks(uint32_t n);

#endif /* _SYS_TICK_H_ */
