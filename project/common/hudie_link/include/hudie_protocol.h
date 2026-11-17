#pragma once

/*
 * 遥控器 <-> 飞控 共用协议（C 版本）
 *
 * 帧格式与 项目计划/firmware/shared/protocol.h（v0.1 草案）逐字节一致：
 *   - 小端、packed；
 *   - CRC16-CCITT（初值 0xFFFF，多项式 0x1021）覆盖除 crc 外所有字节；
 *   - 修改任何字段 = 升 PROTO_VER + 两端同步。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_VER   1u
#define TYPE_RC     0x01u
#define TYPE_TELEM  0x02u
#define TYPE_BIND   0x03u

/* rc_frame_t.flags */
#define RC_FLAG_ARMED      (1u << 0) /* 解锁请求 */
#define RC_FLAG_MANUAL     (1u << 1) /* 0 = 增稳，1 = 手动 */
#define RC_FLAG_CALIBRATE  (1u << 2) /* 请求重新校准陀螺零偏（仅上锁时生效） */
#define RC_FLAG_BIND       (1u << 3) /* 保留：绑定流程用 TYPE_BIND 帧 */

/* rc_frame_t.ch[] 下标与范围 */
enum {
    RC_CH_THROTTLE = 0, /* 0..1000 */
    RC_CH_PITCH    = 1, /* -1000..1000，正 = 抬头 */
    RC_CH_ROLL     = 2, /* -1000..1000，正 = 右滚 */
    RC_CH_YAW      = 3, /* -1000..1000，正 = 右偏 */
    RC_CH_MODE     = 4, /* -1000 / 0 / 1000 */
    RC_CH_ARM      = 5, /* 0 / 1000 */
    RC_CH_COUNT    = 6
};

/* 控制帧：遥控器 -> 飞控，50 Hz */
typedef struct __attribute__((packed)) {
    uint8_t  ver;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  flags;
    int16_t  ch[RC_CH_COUNT];
    uint16_t crc;
} rc_frame_t; /* 18 字节 */

/* tele_frame_t.state */
enum {
    TELE_STATE_SELFTEST = 0,
    TELE_STATE_CALIBRATE = 1,
    TELE_STATE_STANDBY = 2,
    TELE_STATE_FLYING = 3,
    TELE_STATE_FAILSAFE = 4,
    TELE_STATE_FAULT = 5
};

/* 遥测帧：飞控 -> 遥控器，20 Hz */
typedef struct __attribute__((packed)) {
    uint8_t  ver, type, seq, state;
    int16_t  roll, pitch, yaw;   /* 0.1 deg */
    int16_t  gyro[3];            /* dps */
    uint16_t vbat_mv;
    int16_t  alt_cm;             /* 第一版不读气压计，恒为 0 */
    uint8_t  rssi_dbm;           /* 取绝对值，例如 55 表示 -55 dBm */
    uint8_t  loss_pct;
    uint16_t crc;
} tele_frame_t; /* 24 字节 */

/* bind_frame_t.cmd */
enum {
    BIND_CMD_REQUEST = 1, /* 遥控器广播：请求绑定 */
    BIND_CMD_ACCEPT  = 2, /* 飞控单播：确认绑定 */
    BIND_CMD_UNBIND  = 3
};

typedef struct __attribute__((packed)) {
    uint8_t  ver, type, cmd;
    uint8_t  mac[6];             /* 发起方 STA MAC */
    uint8_t  reserved;
    uint16_t crc;
} bind_frame_t; /* 12 字节 */

_Static_assert(sizeof(rc_frame_t) == 18, "rc_frame_t size");
_Static_assert(sizeof(tele_frame_t) == 24, "tele_frame_t size");
_Static_assert(sizeof(bind_frame_t) == 12, "bind_frame_t size");

uint16_t hudie_crc16_ccitt(const uint8_t *data, size_t len);

/* 填写 ver/type 并计算 CRC。frame_len 为整个结构体长度。 */
void hudie_proto_finalize(void *frame, size_t frame_len, uint8_t type);

/* 校验长度、版本、类型和 CRC。 */
bool hudie_proto_check(const void *frame, size_t len, size_t expect_len, uint8_t expect_type);

#ifdef __cplusplus
}
#endif
