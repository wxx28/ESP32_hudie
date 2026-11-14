/**
 * @file    protocol.h
 * @brief   遥控器 <-> 飞控 共用协议（草案 v0.1，评审通过后冻结）
 *
 * 规则：
 *  - 两端必须使用同一份文件；
 *  - 修改字段 = 改版本号 + 双方同步 + 记入 docs/06；
 *  - 小端、packed、CRC16-CCITT 覆盖除 crc 外所有字节。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#define PROTO_VER      1u
#define TYPE_RC        0x01u
#define TYPE_TELEM     0x02u
#define TYPE_BIND      0x03u

/* ---------- 控制帧：遥控器 -> 机 50Hz ---------- */
typedef struct __attribute__((packed)) {
    uint8_t  ver;      /**< PROTO_VER */
    uint8_t  type;     /**< TYPE_RC */
    uint8_t  seq;      /**< 递增，统计丢包 */
    uint8_t  flags;    /**< bit0 armed, bit1 mode(0增稳/1手动), bit2 calibrate, bit3 bind */
    int16_t  ch[6];    /**< -1000..1000；ch0 油门 0..1000 */
    uint16_t crc;      /**< CRC16-CCITT */
} rc_frame_t;          /* 18 bytes */

/* ---------- 遥测帧：机 -> 遥控器 20Hz ---------- */
typedef struct __attribute__((packed)) {
    uint8_t  ver, type, seq, state;  /**< state: 0自检 1校准 2待解锁 3飞行 4迫降 5故障 */
    int16_t  roll, pitch, yaw;       /**< 0.1 deg */
    int16_t  gyro[3];                /**< dps */
    uint16_t vbat_mv;
    int16_t  alt_cm;
    uint8_t  rssi_dbm;
    uint8_t  loss_pct;
    uint16_t crc;
} tele_frame_t;        /* 24 bytes */

/* ---------- 绑定帧 ---------- */
typedef struct __attribute__((packed)) {
    uint8_t ver, type, cmd;          /**< cmd: 1 请求, 2 确认, 3 解绑 */
    uint8_t mac[6];
    uint8_t reserved;                /**< 补齐到 12 字节，保留 */
    uint16_t crc;
} bind_frame_t;        /* 12 bytes */

static_assert(sizeof(rc_frame_t) == 18, "rc_frame_t size");
static_assert(sizeof(tele_frame_t) == 24, "tele_frame_t size");
static_assert(sizeof(bind_frame_t) == 12, "bind_frame_t size");

/* ---------- CRC ---------- */
static inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

/* ---------- 打包 ---------- */
template <typename T>
static inline void proto_finalize(T& f) {
    f.crc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&f), sizeof(T) - sizeof(f.crc));
}
template <typename T>
static inline bool proto_check(const T& f) {
    if (f.ver != PROTO_VER) return false;
    return f.crc == crc16_ccitt(reinterpret_cast<const uint8_t*>(&f), sizeof(T) - sizeof(f.crc));
}
