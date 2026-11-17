#include "flight_control.h"

#include <string.h>

#include "LED.h"
#include "esp_log.h"
#include "fc_input.h"
#include "fc_link.h"
#include "fc_output.h"
#include "fc_scheduler.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "flight";

#define TELEM_DIV   (FLIGHT_CONTROL_FAST_FREQUENCY_HZ / 20u)  /* 20 Hz 遥测 */
#define LOG_DIV     (FLIGHT_CONTROL_FAST_FREQUENCY_HZ * 2u)   /* 2 s 打印一次 */

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static flight_control_snapshot_t s_snap;
static algo_config_t s_config;
static volatile bool s_estop_request;
static volatile bool s_clear_request;
static bool s_skip_dt;   /* 阻塞操作后的第一拍用名义周期，避免 dt 超限 */

static bool is_flying(safety_state_t s)
{
    return s == ALGO_SAFETY_ARMED || s == ALGO_SAFETY_DEGRADED || s == ALGO_SAFETY_FAILSAFE;
}

static led_pattern_t pattern_for(const algo_output_t *o, bool bound, bool rc_ok)
{
    if (s_snap.emergency_stop_latched) {
        return LED_PATTERN_SOS;
    }
    switch (o->safety.state) {
    case ALGO_SAFETY_BOOT:
    case ALGO_SAFETY_SELFTEST:
        return LED_PATTERN_RAPID;
    case ALGO_SAFETY_STANDBY:
        if (!bound || !rc_ok) {
            return LED_PATTERN_DOUBLE;
        }
        return (o->fault_flags & ALGO_FAULT_POWER) ? LED_PATTERN_FAST : LED_PATTERN_SLOW;
    case ALGO_SAFETY_ARMED:
        return LED_PATTERN_ON;
    case ALGO_SAFETY_DEGRADED:
    case ALGO_SAFETY_FAILSAFE:
        return LED_PATTERN_FAST;
    default:
        return LED_PATTERN_SOS;
    }
}

static const char *state_name(safety_state_t s)
{
    static const char *n[] = {"BOOT", "SELFTEST", "STANDBY", "ARMED", "DEGRADED", "FAILSAFE", "ESTOP"};
    return ((unsigned)s < 7u) ? n[s] : "?";
}

static void tick(uint64_t now_us, float dt_s)
{
    if (s_skip_dt) {
        dt_s = s_config.scheduler.fast_dt_s;
        s_skip_dt = false;
    }
    if (dt_s > s_config.scheduler.max_dt_s) {
        /* 调度被长时间打断：本拍按上限计算，避免算法拒绝 */
        dt_s = s_config.scheduler.max_dt_s;
    }

    algo_input_t in;
    fc_input_collect(now_us, &in);

    algo_output_t out;
    algo_status_t st = algo_api_step(&in, dt_s, &out);

    if (s_estop_request && !s_snap.emergency_stop_latched) {
        portENTER_CRITICAL(&s_lock);
        s_snap.emergency_stop_latched = true;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGE(TAG, "飞控急停已锁存");
    }
    if (s_clear_request) {
        s_clear_request = false;
        algo_api_clear_latched_faults();
    }

    if (st != ALGO_STATUS_OK || s_snap.emergency_stop_latched) {
        fc_output_write_safe();
    } else {
        fc_output_write(&out);
    }

    const bool flying = is_flying(out.safety.state);
    fc_link_allow_bind(!flying);

    portENTER_CRITICAL(&s_lock);
    s_snap.fast_cycle_count++;
    s_snap.last_algo_status = st;
    if (st != ALGO_STATUS_OK) {
        s_snap.algorithm_error_count++;
    }
    s_snap.output = out;
    const uint32_t n = s_snap.fast_cycle_count;
    portEXIT_CRITICAL(&s_lock);

    if ((n % TELEM_DIV) == 0u) {
        fc_link_send_telemetry(&out, fc_input_battery_mv());
        led_set_pattern(pattern_for(&out, fc_link_is_bound(), in.rc.valid));
    }

    if ((n % LOG_DIV) == 0u) {
        ESP_LOGI(TAG, "%s rc=%s(%ums) thr=%.2f roll=%+.1f° pitch=%+.1f° f=%.1fHz servo=%.0f/%.0f vbat=%umV faults=0x%03x overrun=%u",
                 state_name(out.safety.state), in.rc.valid ? "ok" : "--",
                 (unsigned)(in.rc.age_ms > 9999u ? 9999u : in.rc.age_ms), in.rc.throttle,
                 out.attitude.roll_rad * 57.2958f, out.attitude.pitch_rad * 57.2958f, out.phase.f_hz,
                 out.servo.angle_deg[0], out.servo.angle_deg[1], (unsigned)fc_input_battery_mv(),
                 (unsigned)out.fault_flags, (unsigned)fc_scheduler_overrun_count());
    }

    /* 遥控请求重新校准陀螺：只在没飞的时候做，做完整个算法重新自检 */
    if (fc_input_take_calibrate_request() && !flying) {
        ESP_LOGI(TAG, "遥控请求陀螺校准，请保持静止 1 s");
        led_set_pattern(LED_PATTERN_RAPID);
        fc_output_write_safe();
        fc_input_calibrate();
        algo_api_reset();
        s_skip_dt = true;
    }
}

fc_status_t flight_control_init(const algo_config_t *algo_config)
{
    if (algo_config == NULL) {
        return FC_STATUS_INVALID_ARG;
    }
    if (fc_scheduler_is_running()) {
        return FC_STATUS_NOT_READY;
    }
    if (algo_api_config_validate(algo_config) != ALGO_STATUS_OK) {
        return FC_STATUS_INVALID_CONFIG;
    }
    s_config = *algo_config;

    led_init();
    led_set_pattern(LED_PATTERN_RAPID);

    /* 先让舵机停在中位，再初始化其它硬件 */
    if (fc_output_init(&s_config.linkage) != FC_OUTPUT_STATUS_OK) {
        ESP_LOGE(TAG, "舵机输出初始化失败");
        return FC_STATUS_HARDWARE_ERROR;
    }
    if (!fc_link_init()) {
        ESP_LOGE(TAG, "ESP-NOW 初始化失败");
        return FC_STATUS_HARDWARE_ERROR;
    }
    /* IMU 失败不返回错误：飞控照常运行，安全状态机会停在自检并闪 SOS */
    fc_input_init();

    if (algo_api_init(&s_config) != ALGO_STATUS_OK) {
        return FC_STATUS_ALGORITHM_ERROR;
    }
    if (!fc_scheduler_init(FLIGHT_CONTROL_FAST_PERIOD_US, tick)) {
        return FC_STATUS_SCHEDULER_ERROR;
    }

    portENTER_CRITICAL(&s_lock);
    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.initialized = true;
    portEXIT_CRITICAL(&s_lock);

    if (!s_config.waveform.mechanical_limits_valid && s_config.safety.require_valid_mechanical_limits) {
        ESP_LOGW(TAG, "机构参数未确认（menuconfig: FC_MECH_CONFIRMED），可以连遥控看姿态，但不会解锁扑翼");
    }
    return FC_STATUS_OK;
}

fc_status_t flight_control_start(void)
{
    if (!s_snap.initialized) {
        return FC_STATUS_NOT_INITIALIZED;
    }
    if (s_snap.emergency_stop_latched) {
        return FC_STATUS_NOT_READY;
    }
    if (!fc_scheduler_start()) {
        return FC_STATUS_SCHEDULER_ERROR;
    }
    portENTER_CRITICAL(&s_lock);
    s_snap.running = true;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "飞控调度已启动 (%u Hz)", (unsigned)FLIGHT_CONTROL_FAST_FREQUENCY_HZ);
    return FC_STATUS_OK;
}

fc_status_t flight_control_stop(void)
{
    if (!s_snap.initialized) {
        return FC_STATUS_NOT_INITIALIZED;
    }
    fc_scheduler_stop();
    fc_output_write_safe();
    algo_api_reset();
    portENTER_CRITICAL(&s_lock);
    s_snap.running = false;
    portEXIT_CRITICAL(&s_lock);
    return FC_STATUS_OK;
}

fc_status_t flight_control_emergency_stop(void)
{
    if (!s_snap.initialized) {
        return FC_STATUS_NOT_INITIALIZED;
    }
    s_estop_request = true;
    fc_output_write_safe();
    return FC_STATUS_OK;
}

fc_status_t flight_control_clear_faults(void)
{
    if (!s_snap.initialized) {
        return FC_STATUS_NOT_INITIALIZED;
    }
    s_clear_request = true;
    return FC_STATUS_OK;
}

fc_status_t flight_control_get_snapshot(flight_control_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return FC_STATUS_INVALID_ARG;
    }
    if (!s_snap.initialized) {
        return FC_STATUS_NOT_INITIALIZED;
    }
    portENTER_CRITICAL(&s_lock);
    *snapshot = s_snap;
    portEXIT_CRITICAL(&s_lock);
    return FC_STATUS_OK;
}
