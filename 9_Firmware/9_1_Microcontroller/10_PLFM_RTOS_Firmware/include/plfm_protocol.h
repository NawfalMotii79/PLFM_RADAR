#ifndef PLFM_PROTOCOL_H
#define PLFM_PROTOCOL_H

#include "plfm_types.h"

#define PLFM_PROTO_SOF         (0xA5U)
#define PLFM_PROTO_EOF         (0x5AU)
#define PLFM_PROTO_MAX_PAYLOAD (512U)

typedef enum
{
    PLFM_CMD_GET_CONFIG = 0x01,
    PLFM_CMD_SET_CONFIG = 0x02,
    PLFM_CMD_START_STREAM = 0x03,
    PLFM_CMD_STOP_STREAM = 0x04,
    PLFM_CMD_GET_DIAG = 0x05,
    PLFM_CMD_ACK = 0x80,
    PLFM_CMD_NACK = 0x81,
    PLFM_CMD_RANGE_PROFILE = 0x90
} plfm_cmd_t;

typedef struct
{
    uint8_t sof;
    uint8_t cmd;
    uint16_t payload_len;
    uint32_t seq;
} plfm_proto_hdr_t;

typedef struct
{
    plfm_proto_hdr_t hdr;
    uint8_t payload[PLFM_PROTO_MAX_PAYLOAD];
    uint16_t crc16;
    uint8_t eof;
} plfm_frame_t;

uint16_t plfm_crc16_ccitt(const uint8_t *data, size_t len);
plfm_status_t plfm_frame_encode(plfm_cmd_t cmd,
                                uint32_t seq,
                                const uint8_t *payload,
                                uint16_t payload_len,
                                uint8_t *out,
                                size_t *out_len);
plfm_status_t plfm_frame_decode(const uint8_t *in,
                                size_t in_len,
                                plfm_frame_t *frame);

#endif /* PLFM_PROTOCOL_H */
