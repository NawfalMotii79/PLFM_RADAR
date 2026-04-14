#include "plfm_bootloader.h"

/*
 * Reference A/B bootloader state machine.
 * Platform-specific flash erase/write/read and reset hooks are intentionally
 * abstracted for portability and safety review.
 */

static plfm_bl_state_t g_state = PLFM_BL_IDLE;
static plfm_bl_manifest_t g_manifest;

int plfm_bl_begin_update(const plfm_bl_manifest_t *manifest)
{
    if (manifest == (const plfm_bl_manifest_t *)0) {
        return -1;
    }
    g_manifest = *manifest;
    g_state = PLFM_BL_RECEIVING;
    return 0;
}

int plfm_bl_push_chunk(uint32_t offset, const uint8_t *data, uint32_t len)
{
    (void)offset;
    (void)data;
    (void)len;

    if (g_state != PLFM_BL_RECEIVING) {
        return -1;
    }
    return 0;
}

int plfm_bl_finalize(void)
{
    if (g_state != PLFM_BL_RECEIVING) {
        return -1;
    }
    g_state = PLFM_BL_VERIFYING;

    /*
     * TODO:
     * 1) verify CRC/signature
     * 2) atomically mark next slot active
     * 3) reboot to new image
     */
    g_state = PLFM_BL_READY_TO_SWAP;
    return 0;
}

plfm_bl_state_t plfm_bl_get_state(void)
{
    return g_state;
}
