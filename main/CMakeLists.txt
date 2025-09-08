idf_component_register(SRCS "elt_servo_control.c" "elt_wifi_comm.c" "elt_graphic_ui.c" "elt_shared_data.c" "main.c"
                    INCLUDE_DIRS "." REQUIRES mdns nvs_flash esp_netif esp_wifi ads111x ina219 esp_http_server json)

lvgl_port_create_c_image("images/esp_logo.png" "images/" "ARGB8888" "NONE")
lvgl_port_add_images(${COMPONENT_LIB} "images/")