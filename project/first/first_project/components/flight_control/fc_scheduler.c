#include "fc_scheduler.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 飞控任务优先级高于 Wi-Fi 以外的一切应用任务 */
#define FC_TASK_PRIORITY   (configMAX_PRIORITIES - 3)
#define FC_TASK_STACK      6144

static esp_timer_handle_t s_timer;
static TaskHandle_t s_task;
static fc_scheduler_tick_cb_t s_cb;
static uint32_t s_period_us;
static volatile bool s_running;
static volatile uint32_t s_overrun;

static void timer_cb(void *arg)
{
    (void)arg;
    if (s_task != NULL) {
        /* 通知值累加；任务一次取出若大于 1 说明错过节拍 */
        xTaskNotifyGive(s_task);
    }
}

static void fc_task(void *arg)
{
    (void)arg;
    uint64_t last_us = 0;
    while (true) {
        const uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (n > 1u) {
            s_overrun += n - 1u;
        }
        if (!s_running) {
            last_us = 0;
            continue;
        }
        const uint64_t now = (uint64_t)esp_timer_get_time();
        float dt = (last_us == 0u) ? (float)s_period_us * 1e-6f : (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (s_cb != NULL) {
            s_cb(now, dt);
        }
    }
}

bool fc_scheduler_init(uint32_t period_us, fc_scheduler_tick_cb_t cb)
{
    if (s_task != NULL) {
        s_cb = cb;
        s_period_us = period_us;
        return true;
    }
    s_cb = cb;
    s_period_us = period_us;
    if (xTaskCreate(fc_task, "flight", FC_TASK_STACK, NULL, FC_TASK_PRIORITY, &s_task) != pdPASS) {
        return false;
    }
    const esp_timer_create_args_t args = {
        .callback = timer_cb,
        .name = "fc_tick",
        .dispatch_method = ESP_TIMER_TASK,
    };
    return esp_timer_create(&args, &s_timer) == ESP_OK;
}

bool fc_scheduler_start(void)
{
    if (s_timer == NULL) {
        return false;
    }
    if (s_running) {
        return true;
    }
    s_running = true;
    if (esp_timer_start_periodic(s_timer, s_period_us) != ESP_OK) {
        s_running = false;
        return false;
    }
    return true;
}

void fc_scheduler_stop(void)
{
    if (s_timer != NULL && s_running) {
        esp_timer_stop(s_timer);
    }
    s_running = false;
}

bool fc_scheduler_is_running(void)
{
    return s_running;
}

uint32_t fc_scheduler_overrun_count(void)
{
    return s_overrun;
}
