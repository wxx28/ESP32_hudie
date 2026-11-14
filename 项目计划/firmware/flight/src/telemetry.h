/**
 * @file  telemetry.h
 * @brief 20Hz 遥测组帧发送（接口契约，实现待补）
 * 负责人：链路组
 */
#pragma once

struct TelemetryState {
    float roll_deg, pitch_deg, yaw_deg;
    float gyro_dps[3];
    float vbat_v;
    float alt_cm;
    uint8_t flight_state;
};

void telemetry_init();
void telemetry_send(const TelemetryState& s);
