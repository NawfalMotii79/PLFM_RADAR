#include "plfm_protocol.h"

#include <string.h>

uint16_t plfm_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

plfm_status_t plfm_frame_encode(plfm_cmd_t cmd,
                                uint32_t seq,
                                const uint8_t *payload,
                                uint16_t payload_len,
                                uint8_t *out,
                                size_t *out_len)
{
    size_t idx = 0U;
    uint16_t crc;
    size_t total_needed;

    if ((out == NULL) || (out_len == NULL) || (payload_len > PLFM_PROTO_MAX_PAYLOAD)) {
        return PLFM_ERR_INVALID_ARG;
    }
    if ((payload_len > 0U) && (payload == NULL)) {
        return PLFM_ERR_INVALID_ARG;
    }

    total_needed = (size_t)1U + 1U + 2U + 4U + payload_len + 2U + 1U;
    if (*out_len < total_needed) {
        return PLFM_ERR_INVALID_ARG;
    }

    out[idx++] = PLFM_PROTO_SOF;
    out[idx++] = (uint8_t)cmd;
    out[idx++] = (uint8_t)(payload_len & 0xFFU);
    out[idx++] = (uint8_t)((payload_len >> 8U) & 0xFFU);
    out[idx++] = (uint8_t)(seq & 0xFFU);
    out[idx++] = (uint8_t)((seq >> 8U) & 0xFFU);
    out[idx++] = (uint8_t)((seq >> 16U) & 0xFFU);
    out[idx++] = (uint8_t)((seq >> 24U) & 0xFFU);

    if (payload_len > 0U) {
        (void)memcpy(&out[idx], payload, payload_len);
        idx += payload_len;
    }

    crc = plfm_crc16_ccitt(&out[1], (size_t)(1U + 2U + 4U + payload_len));
    out[idx++] = (uint8_t)(crc & 0xFFU);
    out[idx++] = (uint8_t)((crc >> 8U) & 0xFFU);
    out[idx++] = PLFM_PROTO_EOF;

    *out_len = idx;
    return PLFM_OK;
}

plfm_status_t plfm_frame_decode(const uint8_t *in, size_t in_len, plfm_frame_t *frame)
{
    uint16_t payload_len;
    uint16_t crc_rx;
    uint16_t crc_calc;
    size_t expected_len;

    if ((in == NULL) || (frame == NULL) || (in_len < (size_t)11U)) {
        return PLFM_ERR_INVALID_ARG;
    }
    if ((in[0] != PLFM_PROTO_SOF) || (in[in_len - 1U] != PLFM_PROTO_EOF)) {
        return PLFM_ERR_INVALID_ARG;
    }

    payload_len = (uint16_t)((uint16_t)in[2] | ((uint16_t)in[3] << 8U));
    if (payload_len > PLFM_PROTO_MAX_PAYLOAD) {
        return PLFM_ERR_INVALID_ARG;
    }

    expected_len = (size_t)1U + 1U + 2U + 4U + payload_len + 2U + 1U;
    if (in_len != expected_len) {
        return PLFM_ERR_INVALID_ARG;
    }

    crc_rx = (uint16_t)((uint16_t)in[8U + payload_len] |
                        ((uint16_t)in[9U + payload_len] << 8U));
    crc_calc = plfm_crc16_ccitt(&in[1], (size_t)(1U + 2U + 4U + payload_len));
    if (crc_rx != crc_calc) {
        return PLFM_ERR_CRC;
    }

    frame->hdr.sof = in[0];
    frame->hdr.cmd = in[1];
    frame->hdr.payload_len = payload_len;
    frame->hdr.seq = (uint32_t)in[4] |
                     ((uint32_t)in[5] << 8U) |
                     ((uint32_t)in[6] << 16U) |
                     ((uint32_t)in[7] << 24U);

    if (payload_len > 0U) {
        (void)memcpy(frame->payload, &in[8], payload_len);
    }
    frame->crc16 = crc_rx;
    frame->eof = in[in_len - 1U];

    return PLFM_OK;
}
