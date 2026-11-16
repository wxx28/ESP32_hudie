#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_http_server.h"
#include "servo_motion.h"
#include "servo_web_server.h"

#define CONTROL_REQUEST_MAX_SIZE 256

extern const uint8_t servo_web_html_start[] asm("_binary_servo_web_html_start");

static esp_err_t root_get_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, (const char *)servo_web_html_start,
                           HTTPD_RESP_USE_STRLEN);
}

static esp_err_t state_get_handler(httpd_req_t *request)
{
    const servo_motion_params_t state = servo_motion_get_params();
    char response[256];

    snprintf(response, sizeof(response),
             "{\"amplitude_deg\":%.2f,\"differential_deg\":%.2f,"
             "\"frequency_hz\":%.2f,\"phase_deg\":%.2f,\"center_deg\":%.2f,"
             "\"running\":%s,\"joystick_active\":%s}",
             state.amplitude_deg, state.differential_deg, state.frequency_hz,
             state.phase_deg, state.center_deg, state.running ? "true" : "false",
             state.joystick_active ? "true" : "false");
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, response);
}

static bool read_float_parameter(char *body, const char *key, float *value)
{
    char encoded_value[24];
    if (httpd_query_key_value(body, key, encoded_value, sizeof(encoded_value)) != ESP_OK) {
        return false;
    }

    char *end = NULL;
    const float parsed = strtof(encoded_value, &end);
    if (end == encoded_value || *end != '\0' || !isfinite(parsed)) {
        return false;
    }

    *value = parsed;
    return true;
}

static bool read_bool_parameter(char *body, const char *key, bool *value)
{
    char encoded_value[6];
    if (httpd_query_key_value(body, key, encoded_value, sizeof(encoded_value)) != ESP_OK) {
        return false;
    }

    if (strcmp(encoded_value, "true") == 0) {
        *value = true;
        return true;
    }
    if (strcmp(encoded_value, "false") == 0) {
        *value = false;
        return true;
    }
    return false;
}

static bool receive_body(httpd_req_t *request, char *body, size_t capacity)
{
    size_t received = 0;

    if (request->content_len == 0 || request->content_len >= capacity) {
        return false;
    }

    while (received < request->content_len) {
        const int count = httpd_req_recv(request, body + received,
                                         request->content_len - received);
        if (count <= 0) {
            return false;
        }
        received += (size_t)count;
    }
    body[received] = '\0';
    return true;
}

static esp_err_t send_ok(httpd_req_t *request)
{
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_sendstr(request, "{\"ok\":true}");
}

static esp_err_t control_post_handler(httpd_req_t *request)
{
    char body[CONTROL_REQUEST_MAX_SIZE + 1];
    servo_motion_params_t next = {0};

    if (!receive_body(request, body, sizeof(body))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid request size");
    }

    const bool fields_valid =
        read_float_parameter(body, "amplitude_deg", &next.amplitude_deg) &&
        read_float_parameter(body, "differential_deg", &next.differential_deg) &&
        read_float_parameter(body, "frequency_hz", &next.frequency_hz) &&
        read_float_parameter(body, "phase_deg", &next.phase_deg) &&
        read_float_parameter(body, "center_deg", &next.center_deg) &&
        read_bool_parameter(body, "running", &next.running);

    if (!fields_valid || servo_motion_set_params(&next) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Parameter out of range");
    }

    return send_ok(request);
}

static esp_err_t config_post_handler(httpd_req_t *request)
{
    char body[CONTROL_REQUEST_MAX_SIZE + 1];
    float frequency_hz = 0.0f;
    float phase_deg = 0.0f;
    float center_deg = 0.0f;

    if (!receive_body(request, body, sizeof(body))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid request size");
    }

    if (!read_float_parameter(body, "frequency_hz", &frequency_hz) ||
        !read_float_parameter(body, "phase_deg", &phase_deg) ||
        !read_float_parameter(body, "center_deg", &center_deg) ||
        servo_motion_set_config(frequency_hz, phase_deg, center_deg) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Parameter out of range");
    }

    return send_ok(request);
}

static esp_err_t joystick_post_handler(httpd_req_t *request)
{
    char body[CONTROL_REQUEST_MAX_SIZE + 1];
    float x = 0.0f;
    float y = 0.0f;
    bool active = false;

    if (!receive_body(request, body, sizeof(body))) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Invalid request size");
    }

    if (!read_float_parameter(body, "x", &x) ||
        !read_float_parameter(body, "y", &y) ||
        !read_bool_parameter(body, "active", &active) ||
        servo_motion_set_joystick(x, y, active) != ESP_OK) {
        return httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Parameter out of range");
    }

    return send_ok(request);
}

void servo_web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    config.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t state_uri = {
        .uri = "/api/state",
        .method = HTTP_GET,
        .handler = state_get_handler,
    };
    const httpd_uri_t control_uri = {
        .uri = "/api/control",
        .method = HTTP_POST,
        .handler = control_post_handler,
    };
    const httpd_uri_t config_uri = {
        .uri = "/api/config",
        .method = HTTP_POST,
        .handler = config_post_handler,
    };
    const httpd_uri_t joystick_uri = {
        .uri = "/api/joystick",
        .method = HTTP_POST,
        .handler = joystick_post_handler,
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &state_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &control_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &joystick_uri));
}
