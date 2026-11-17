#include "hudie_protocol.h"

#include <string.h>

uint16_t hudie_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void hudie_proto_finalize(void *frame, size_t frame_len, uint8_t type)
{
    uint8_t *p = (uint8_t *)frame;
    p[0] = PROTO_VER;
    p[1] = type;
    uint16_t crc = hudie_crc16_ccitt(p, frame_len - sizeof(uint16_t));
    memcpy(p + frame_len - sizeof(uint16_t), &crc, sizeof(crc)); /* 小端 */
}

bool hudie_proto_check(const void *frame, size_t len, size_t expect_len, uint8_t expect_type)
{
    const uint8_t *p = (const uint8_t *)frame;
    if (frame == NULL || len != expect_len || p[0] != PROTO_VER || p[1] != expect_type) {
        return false;
    }
    uint16_t crc;
    memcpy(&crc, p + len - sizeof(uint16_t), sizeof(crc));
    return crc == hudie_crc16_ccitt(p, len - sizeof(uint16_t));
}
