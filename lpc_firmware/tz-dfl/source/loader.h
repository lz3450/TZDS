#ifndef _LOADER_H_
#define _LOADER_H_

#include "fsl_common.h"
#include "fsl_shell.h"

typedef enum _loader_status {
    kStatus_LoaderSuccess   = kStatus_Success,                     /* No error occurred */
    kStatus_LoaderInitError = MAKE_STATUS(kStatusGroup_microPI, 1), /* Loader initialization error */
    kStatus_LoaderRegError  = MAKE_STATUS(kStatusGroup_microPI, 2), /* Function registration error */
} loader_status_t;

#define LDRW (0xf000f85fU)

typedef struct _veneer {
    uint32_t ldrw;
    void    *sg;
} veneer_t;

loader_status_t LOADER_Init(void);
void           *LOADER_Load(uint32_t entry, uint32_t exit);
void            LOADER_ListFunction(shell_handle_t shellHandle);

#endif /* _LOADER_H_ */
