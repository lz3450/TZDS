#include "loader.h"
#include "fsl_component_generic_list.h"
#include "fsl_component_mem_manager.h"

#define FUNCTION_NAME_MAX_LENGTH (24)
#define MAX_FUNCTION_COUNT       (0x10U)
#define MAX_FUNCTION_SIZE        (0x100U)

#define SG                    (0xe97fe97fU)
#define BW                    (0xb800f000U)
#define BW_DISTANCE_MASK      (0x000007ffU)
#define BW_DISTANCE_PART_SIZE (11U)
#define SECURE_ADDRESS_MASK   ((uint32_t)(1 << 28))

typedef struct _secure_gateway {
    uint32_t sg;
    uint32_t bw;
} secure_gateway_t;

typedef struct _function_object {
    uint32_t          index;
    uint32_t          start;  // original start address
    uint32_t          length; // length
    secure_gateway_t *entry;  // secure gateway address
    void             *f;      // in-cache start address
    list_element_t    node;
} function_object_t;

#define FUNCTION_OBJECT_POINTER(node) ((function_object_t *)(((uint32_t)(node)) - (sizeof(function_object_t) - sizeof(list_element_t))))

static list_label_t        s_functionObjectList;
static const list_handle_t s_listHandle_FunctionObject = &s_functionObjectList;
secure_gateway_t           s_secureGatewayEntries[MAX_FUNCTION_COUNT] __attribute__((section(".function_entry")));
uint32_t                   s_functionCount = 0;

/* Memory */
#define FUNCTION_OBJECT_BUFFER_ID (0U)
#define FUNCTION_CACHE_BUFFER_ID  (1U)
MEM_BLOCK_BUFFER_DEFINE(FunctionObject, MAX_FUNCTION_COUNT, sizeof(function_object_t), FUNCTION_OBJECT_BUFFER_ID);
MEM_BLOCK_BUFFER_DEFINE(FunctionCache, MAX_FUNCTION_COUNT, MAX_FUNCTION_SIZE, FUNCTION_CACHE_BUFFER_ID);

// #define FUNCTION_OBJECT_DEFINE(function_name)     \
//     function_object_t s_functionObject_##function_name = { \
//         .name = #function_name,                   \
//     }
// #define FUNCTION_OBJECT(function_name) &s_functionObject_##function_name

// FUNCTION_OBJECT_DEFINE(MeasureTemperature);

loader_status_t LOADER_Init(void)
{
    assert(kStatus_MemSuccess == MEM_AddBuffer(MEM_BLOCK_BUFFER(FunctionObject)));
    assert(kStatus_MemSuccess == MEM_AddBuffer(MEM_BLOCK_BUFFER(FunctionCache)));
    LIST_Init(s_listHandle_FunctionObject, MAX_FUNCTION_COUNT);

    return kStatus_LoaderSuccess;
}

loader_status_t LOADER_RegisterFunction(function_object_t *functionObject)
{
    if (NULL == functionObject) {
        return kStatus_LoaderRegError;
    }

    (void)memset(&functionObject->node, 0, sizeof(functionObject->node));

    if (kLIST_Ok != LIST_AddTail(s_listHandle_FunctionObject, &functionObject->node)) {
        return kStatus_LoaderRegError;
    }

    s_functionCount++;

    return kStatus_LoaderSuccess;
}

void LOADER_ListFunction(shell_handle_t shellHandle)
{
    list_element_handle_t p = LIST_GetHead(s_listHandle_FunctionObject);
    function_object_t    *functionObjectHandle;

    SHELL_Printf(shellHandle,
                 "\r\nRegistered Functions:\r\n"
                 "+---------+--------------+--------------+--------------+----------+\r\n"
                 "|  %s  |  %-10s  |  %-10s  |  %-10s  |  %s  |\r\n"
                 "+=========+==============+==============+==============+==========+\r\n",
                 "Index", "Start", "Entry", "Start*", "Length");

    while (p != NULL) {
        functionObjectHandle = FUNCTION_OBJECT_POINTER(p);
        SHELL_Printf(shellHandle,
                     "|  %5d  |  0x%08x  |  0x%08x  |  0x%08x  |  %6d  |\r\n",
                     functionObjectHandle->index,
                     functionObjectHandle->start,
                     functionObjectHandle->entry,
                     functionObjectHandle->f,
                     functionObjectHandle->length);
        p = LIST_GetNext(p);
    }
    SHELL_Printf(shellHandle,
                 "+---------+--------------+--------------+--------------+----------+\r\n"
                 "\r\n");
}

static inline function_object_t *LOADER_AllocFunctionObject(void)
{
    return (function_object_t *)MEM_BufferAllocWithId(sizeof(function_object_t), FUNCTION_OBJECT_BUFFER_ID);
}

static inline void *LOADER_AllocFunctionCache(uint32_t numBytes)
{
    assert(numBytes <= MAX_FUNCTION_SIZE);
    return MEM_BufferAllocWithId(numBytes, FUNCTION_CACHE_BUFFER_ID);
}

void *LOADER_Load(uint32_t start, uint32_t length)
{
    function_object_t *fo = LOADER_AllocFunctionObject();
    assert(fo);

    fo->index  = s_functionCount;
    fo->start  = start;
    fo->length = length;

#if 1
    void *f = (void *)LOADER_AllocFunctionCache(length);
    if (NULL == f) {
        // TODO: cache replacement
        return NULL;
    }
    fo->f = f;
    memcpy(f, (void *)start, length);

    /* 4770  bl ir -> 4774  blns ir */
    // ((uint16_t *)f)[length / 2 - 1] = 0x4774;
    for (int i = 0; i < length / 2; i++) {
        uint16_t *instruct = f;
        if (instruct[i] == 0x4770)
            instruct[i] = 0x4774;
    }
#endif

    /* (des - src - 4) / 2 */
    /* 22 bits */
#if 1
    int32_t distance = ((uint32_t)f - (uint32_t)&fo->entry->bw - 4) >> 1;

    fo->entry = &s_secureGatewayEntries[s_functionCount];

    fo->entry->sg = SG;
    fo->entry->bw = BW | (((distance & BW_DISTANCE_MASK) << 0x10) | (((distance >> BW_DISTANCE_PART_SIZE) & BW_DISTANCE_MASK)));

    assert(kStatus_LoaderSuccess == LOADER_RegisterFunction(fo));

    void *function_entry = (void *)(((uint32_t)fo->entry & ~SECURE_ADDRESS_MASK) + 1);
    return function_entry;
#else
    return NULL;
#endif
}
