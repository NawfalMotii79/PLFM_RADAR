#ifndef PLFM_BOOTLOADER_H
#define PLFM_BOOTLOADER_H

#include <stdint.h>

typedef enum
{
    PLFM_BL_IDLE = 0,
    PLFM_BL_RECEIVING,
    PLFM_BL_VERIFYING,
    PLFM_BL_READY_TO_SWAP,
    PLFM_BL_FAILED
} plfm_bl_state_t;

typedef struct
{
    uint32_t image_size;
    uint32_t image_crc32;
    uint32_t chunk_size;
} plfm_bl_manifest_t;

int plfm_bl_begin_update(const plfm_bl_manifest_t *manifest);
int plfm_bl_push_chunk(uint32_t offset, const uint8_t *data, uint32_t len);
int plfm_bl_finalize(void);
plfm_bl_state_t plfm_bl_get_state(void);

#endif /* PLFM_BOOTLOADER_H */
