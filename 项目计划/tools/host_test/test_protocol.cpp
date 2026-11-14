// 协议自测骨架（先跑这个，最简单）
#include <cstdio>
#include "protocol.h"

int main() {
    rc_frame_t f{};
    f.ver = PROTO_VER; f.type = TYPE_RC; f.seq = 1;
    for (int i = 0; i < 6; ++i) f.ch[i] = (int16_t)(i * 100 - 300);
    proto_finalize(f);

    printf("sizeof rc=%zu tele=%zu bind=%zu\n",
           sizeof(rc_frame_t), sizeof(tele_frame_t), sizeof(bind_frame_t));
    printf("crc=%04X check=%d\n", f.crc, (int)proto_check(f));

    f.ch[0] ^= 1;
    printf("tamper check=%d (expect 0)\n", (int)proto_check(f));
    return 0;
}
