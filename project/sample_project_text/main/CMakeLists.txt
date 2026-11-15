idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES Middlewares esp_event esp_http_server esp_netif esp_wifi nvs_flash
    EMBED_TXTFILES "servo_web.html"
)
