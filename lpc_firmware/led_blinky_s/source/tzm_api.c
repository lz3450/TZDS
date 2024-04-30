#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif

#include "fsl_device_registers.h"
#include "tzm_api.h"

/* typedef for non-secure callback functions */
typedef void (*funcptr_ns)(void) __attribute__((cmse_nonsecure_call));

/*!
 * @brief This function jumps to normal world.
 */
void TZM_JumpToNormalWorld(uint32_t nonsecVTORAddress)
{
    funcptr_ns ResetHandler_ns;

    /* Set non-secure main stack (MSP_NS) */
    __TZ_set_MSP_NS(*((uint32_t *)(nonsecVTORAddress)));

    /* Set non-secure vector table */
    SCB_NS->VTOR = nonsecVTORAddress;

    /* Get non-secure reset handler */
    ResetHandler_ns = (funcptr_ns)(*((uint32_t *)(nonsecVTORAddress + 4U)));

    /* Call non-secure application - jump to normal world */
    ResetHandler_ns();
}
