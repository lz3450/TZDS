
#include "pin_mux.h"

#include "fsl_component_mem_manager.h"
#include "fsl_gpio.h"
#include "fsl_shell.h"

#include "loader.h"
#include "tzds.h"
#include "fsl_shell.h"
#include <stdint.h>
#include <stdlib.h>

extern serial_handle_t g_serialHandle;
static shell_handle_t  s_shellHandle;

SDK_ALIGN(static uint8_t s_shellHandleBuffer[SHELL_HANDLE_SIZE], 4);

/* Shell Command definition */

#if 1
/* led */
static shell_status_t LedControl(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    char *kLedCommand = argv[1];

    if (strcmp(kLedCommand, "on") == 0) {
        GPIO_PortClear(GPIO, LED_BLUE_PORT, LED_BLUE_PIN_MASK);
    } else if (strcmp(kLedCommand, "off") == 0) {
        GPIO_PortSet(GPIO, LED_BLUE_PORT, LED_BLUE_PIN_MASK);
    } else {
        SHELL_Printf(shellHandle, "Control command is wrong!\r\n");
    }
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(led, "\r\n\"led\": LED Control\r\nUsage: led [ on | off ]\r\n", LedControl, 1);

/* malloc: allocate <number_bytes> memory */
static void *m_head TZDS_RW = NULL;
static shell_status_t MemoryAlloc(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    uint32_t number_bytes = atoi(argv[1]);
    if (number_bytes <= 0) {
        SHELL_Printf(shellHandle, "<size> should be greater than 0.\r\n");
        return kStatus_SHELL_Error;
    }
    m_head = MEM_BufferAllocWithId(number_bytes, 0);
    if (NULL == m_head) {
        SHELL_Printf(shellHandle, "Memory allocation failed.\r\n");
        return kStatus_SHELL_Error;
    }
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(malloc, "\r\n\"malloc\": Allocates size bytes of uninitialized storage\r\nUsage: malloc <size>\r\n", MemoryAlloc, 1);

/* mfree: free last allocated buffer. */
static shell_status_t MemoryFree(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    mem_status_t status;
    status = MEMORY_Free(m_head);
    if (kStatus_MemSuccess != status) {
        SHELL_Printf(shellHandle, "Memory free failed.\r\n");
        return kStatus_SHELL_Error;
    }
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(mfree, "\r\n\"mfree\": Free the last allocated buffer\r\nUsage: mfree\r\n", MemoryFree, 0);
#endif

static shell_status_t MemoryTrace(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    SHELL_Printf(shellHandle, "\r\n");
    MEM_Trace(shellHandle);
    SHELL_Printf(shellHandle, "\r\n");
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(mtrace, "\r\n\"mtrace\": Print statistics of memory manager\r\n", MemoryTrace, 0);

#if 0
static shell_status_t ListFunctionObjects(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    LOADER_ListFunction(shellHandle);
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(lsfo, "\r\n\"lsfo\": Print registered function objects\r\n", ListFunctionObjects, 0);
#endif

#if 0
#define BW                    (0xb800f000U)
#define BW_DISTANCE_MASK      (0x000007ffU)
#define BW_DISTANCE_PART_SIZE (11U)
static shell_status_t CalculateBW(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    uint32_t src      = strtol(argv[1], NULL, 0x10);
    uint32_t des      = strtol(argv[2], NULL, 0x10);
    uint32_t distance = (des - src - 0x4) >> 1;
    uint32_t bw = BW | (((distance & BW_DISTANCE_MASK) << 0x10) | (((distance >> BW_DISTANCE_PART_SIZE) & BW_DISTANCE_MASK)));
    SHELL_Printf(shellHandle, "(0x%08x - 0x%08x - 0x4) / 2 = 0x%08x\r\n", des, src, distance);
    SHELL_Printf(shellHandle, "0x%08x\r\n", bw);
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(bw, "\r\n\"bw\": Calculate `b.w\' instruction.\r\nUsage: bw <src> <des>\r\n", CalculateBW, 2);
#endif

static shell_status_t ReadTemperature(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    extern float *g_pCurrentTemperature;
    if (NULL == g_pCurrentTemperature) {
        SHELL_Printf(shellHandle, "Pointer to temperature is not set!\r\n");
        return kStatus_SHELL_Error;
    }
    SHELL_Printf(shellHandle, "Current temperature: \033[33;40m%6.2f\033[37;40m\r\n", *g_pCurrentTemperature);
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(temp, "\r\n\"temp\": Print current temperature\r\n", ReadTemperature, 0);

static shell_status_t CycleCounter(shell_handle_t shellHandle, int32_t argc, char **argv)
{
    SHELL_Printf(shellHandle, "0x%08x\r\n", DWT->CYCCNT);
    return kStatus_SHELL_Success;
}
SHELL_COMMAND_DEFINE(dwt, "\r\n\"dwt\": Print current cycle counter value\r\n", CycleCounter, 0);

void BOARD_Shell_Init(void)
{
    s_shellHandle = &s_shellHandleBuffer[0];

    assert(kStatus_SHELL_Success == SHELL_Init(s_shellHandle, g_serialHandle, "LOADER> "));

#if 0
    status = SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(led));
    assert(kStatus_SHELL_Success == status);

    status = SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(malloc));
    assert(kStatus_SHELL_Success == status);

    status = SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(mfree));
    assert(kStatus_SHELL_Success == status);

    status = SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(bw));
    assert(kStatus_SHELL_Success == status);
#endif

    assert(kStatus_SHELL_Success == SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(mtrace)));

    // assert(kStatus_SHELL_Success == SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(lsfo)));

    assert(kStatus_SHELL_Success == SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(temp)));

    assert(kStatus_SHELL_Success == SHELL_RegisterCommand(s_shellHandle, SHELL_COMMAND(dwt)));
}

void BOARD_Shell_Task(void)
{
#if !(defined(SHELL_NON_BLOCKING_MODE) && (SHELL_NON_BLOCKING_MODE > 0U))
    SHELL_Task(s_shellHandle);
#endif
}
