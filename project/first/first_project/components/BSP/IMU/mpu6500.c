#include "mpu6500.h"

#include <math.h>
#include <string.h>

#include "board_v1.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "mpu6500";

/* 寄存器 */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H  0x3B
#define REG_USER_CTRL     0x6A
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75

#define WHO_AM_I_MPU6500  0x70
#define WHO_AM_I_MPU9250  0x71

/* ±1000 dps -> 32.8 LSB/dps；±8 g -> 4096 LSB/g */
#define GYRO_LSB_PER_DPS  32.8f
#define ACCEL_LSB_PER_G   4096.0f
#define GRAVITY           9.80665f
#define DEG2RAD           0.01745329252f

/* 寄存器配置都在 1 MHz 以内完成，MPU6500 的 SPI 读写上限即为 1 MHz（传感器寄存器可到 20 MHz）。 */
#define SPI_CLOCK_HZ      1000000

static spi_device_handle_t s_dev;
static float s_gyro_bias[3];
static bool s_ready;

/* 传感器轴 -> 机体轴：body[i] = sign[i] * sensor[index[i]] */
static int s_axis_index[3];
static float s_axis_sign[3];

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,
        .tx_data = {reg & 0x7F, val},
    };
    return spi_device_polling_transmit(s_dev, &t);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    if (len + 1 > sizeof(tx)) {
        return ESP_ERR_INVALID_SIZE;
    }
    tx[0] = reg | 0x80;
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    if (err == ESP_OK) {
        memcpy(buf, rx + 1, len);
    }
    return err;
}

static esp_err_t setup_axes(void)
{
#if CONFIG_BSP_IMU_BODY_X_POS_X
    s_axis_index[0] = 0; s_axis_sign[0] = 1.0f;
#elif CONFIG_BSP_IMU_BODY_X_NEG_X
    s_axis_index[0] = 0; s_axis_sign[0] = -1.0f;
#elif CONFIG_BSP_IMU_BODY_X_POS_Y
    s_axis_index[0] = 1; s_axis_sign[0] = 1.0f;
#else
    s_axis_index[0] = 1; s_axis_sign[0] = -1.0f;
#endif
#if CONFIG_BSP_IMU_BODY_Y_POS_X
    s_axis_index[1] = 0; s_axis_sign[1] = 1.0f;
#elif CONFIG_BSP_IMU_BODY_Y_NEG_X
    s_axis_index[1] = 0; s_axis_sign[1] = -1.0f;
#elif CONFIG_BSP_IMU_BODY_Y_POS_Y
    s_axis_index[1] = 1; s_axis_sign[1] = 1.0f;
#else
    s_axis_index[1] = 1; s_axis_sign[1] = -1.0f;
#endif
    s_axis_index[2] = 2;
#if CONFIG_BSP_IMU_BOARD_TOP_UP
    s_axis_sign[2] = -1.0f;
#else
    s_axis_sign[2] = 1.0f;
#endif

    /* 右手系检查：(body X) × (body Y) 必须等于 body Z */
    float ex[3] = {0}, ey[3] = {0}, ez[3] = {0};
    ex[s_axis_index[0]] = s_axis_sign[0];
    ey[s_axis_index[1]] = s_axis_sign[1];
    ez[s_axis_index[2]] = s_axis_sign[2];
    float cx = ex[1] * ey[2] - ex[2] * ey[1];
    float cy = ex[2] * ey[0] - ex[0] * ey[2];
    float cz = ex[0] * ey[1] - ex[1] * ey[0];
    if (s_axis_index[0] == s_axis_index[1] || fabsf(cx - ez[0]) > 0.1f || fabsf(cy - ez[1]) > 0.1f ||
        fabsf(cz - ez[2]) > 0.1f) {
        ESP_LOGE(TAG, "IMU 安装方向不是右手系，请在 menuconfig 里修正");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t mpu6500_init(void)
{
    ESP_RETURN_ON_ERROR(setup_axes(), TAG, "axis config");

    spi_bus_config_t bus = {
        .miso_io_num = BOARD_PIN_IMU_MISO,
        .mosi_io_num = BOARD_PIN_IMU_MOSI,
        .sclk_io_num = BOARD_PIN_IMU_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED), TAG, "spi bus");

    spi_device_interface_config_t dev = {
        .mode = 3,
        .clock_speed_hz = SPI_CLOCK_HZ,
        .spics_io_num = BOARD_PIN_IMU_CS,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev, &s_dev), TAG, "spi device");

    /* 复位，选 PLL 时钟，关 I2C 接口（只用 SPI） */
    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, 0x80), TAG, "reset");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, 0x01), TAG, "clock");
    ESP_RETURN_ON_ERROR(write_reg(REG_USER_CTRL, 0x10), TAG, "i2c disable");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(read_regs(REG_WHO_AM_I, &who, 1), TAG, "who am i");
    if (who != WHO_AM_I_MPU6500 && who != WHO_AM_I_MPU9250) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02x，不是 MPU6500（检查 IO4~IO7 焊接）", who);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_2, 0x00), TAG, "enable axes");
    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, 0x03), TAG, "dlpf");         /* 陀螺 41 Hz */
    ESP_RETURN_ON_ERROR(write_reg(REG_SMPLRT_DIV, 0x00), TAG, "smplrt");   /* 1 kHz */
    ESP_RETURN_ON_ERROR(write_reg(REG_GYRO_CONFIG, 0x10), TAG, "gyro fs"); /* ±1000 dps */
    ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG, 0x10), TAG, "acc fs"); /* ±8 g */
    ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG2, 0x03), TAG, "acc dlpf"); /* 41 Hz */
    vTaskDelay(pdMS_TO_TICKS(20));

    s_ready = true;
    ESP_LOGI(TAG, "MPU6500 ok (WHO_AM_I 0x%02x)", who);
    return ESP_OK;
}

static esp_err_t read_raw(float gyro_dps[3], float accel_g[3], float *temp_c)
{
    uint8_t b[14];
    esp_err_t err = read_regs(REG_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) {
        return err;
    }
    int16_t v[7];
    for (int i = 0; i < 7; ++i) {
        v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    }
    for (int i = 0; i < 3; ++i) {
        accel_g[i] = (float)v[i] / ACCEL_LSB_PER_G;
        gyro_dps[i] = (float)v[4 + i] / GYRO_LSB_PER_DPS;
    }
    if (temp_c != NULL) {
        *temp_c = (float)v[3] / 333.87f + 21.0f;
    }
    return ESP_OK;
}

esp_err_t mpu6500_read(mpu6500_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    float g[3], a[3];
    esp_err_t err = read_raw(g, a, &out->temp_c);
    if (err != ESP_OK) {
        return err;
    }
    out->timestamp_us = (uint64_t)esp_timer_get_time();
    for (int i = 0; i < 3; ++i) {
        const int k = s_axis_index[i];
        out->gyro_rad_s[i] = s_axis_sign[i] * (g[k] - s_gyro_bias[k]) * DEG2RAD;
        out->accel_m_s2[i] = s_axis_sign[i] * a[k] * GRAVITY;
    }
    /* 芯片 Z 轴朝上时，静止读数为 +1 g；机体 Z 向下，比力应为 -g，上面的符号映射已处理 */
    return ESP_OK;
}

esp_err_t mpu6500_calibrate_gyro(uint32_t duration_ms)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t n = duration_ms / 2u;
    double sum[3] = {0}, sq[3] = {0};
    uint32_t got = 0;
    for (uint32_t i = 0; i < n; ++i) {
        float g[3], a[3];
        if (read_raw(g, a, NULL) == ESP_OK) {
            for (int k = 0; k < 3; ++k) {
                sum[k] += g[k];
                sq[k] += (double)g[k] * g[k];
            }
            got++;
        }
        esp_rom_delay_us(2000);
    }
    if (got < n / 2u) {
        return ESP_FAIL;
    }
    for (int k = 0; k < 3; ++k) {
        const double mean = sum[k] / got;
        const double var = sq[k] / got - mean * mean;
        /* 静止时噪声标准差约 0.05~0.1 dps，超过 1 dps 视为在动 */
        if (var > 1.0) {
            ESP_LOGW(TAG, "校准时板子在动（轴 %d 方差 %.2f），请放平静止", k, var);
            return ESP_ERR_INVALID_STATE;
        }
    }
    for (int k = 0; k < 3; ++k) {
        s_gyro_bias[k] = (float)(sum[k] / got);
    }
    ESP_LOGI(TAG, "陀螺零偏 %.2f %.2f %.2f dps", s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
    return ESP_OK;
}

bool mpu6500_is_ready(void)
{
    return s_ready;
}
