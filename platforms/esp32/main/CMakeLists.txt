set(MOTO_REPOSITORY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")
set(MOTO_SHARED_UI_SOURCE
    "${MOTO_REPOSITORY_ROOT}/shared/nav_ui/src/moto_nav_ui.cpp")
set(MOTO_SHARED_UI_FONT
    "${MOTO_REPOSITORY_ROOT}/shared/nav_ui/assets/moto_font_nav_16.c")
set(MOTO_SHARED_UI_CLOCK_FONT
    "${MOTO_REPOSITORY_ROOT}/shared/nav_ui/assets/moto_font_clock_96.c")
set(MOTO_SHARED_UI_NOTIFY_FONTS
    "${MOTO_REPOSITORY_ROOT}/shared/nav_ui/assets/moto_font_notify_16.c"
    "${MOTO_REPOSITORY_ROOT}/shared/nav_ui/assets/moto_font_notify_20.c")
set(MOTO_SHARED_UI_INCLUDE
    "${MOTO_REPOSITORY_ROOT}/shared/nav_ui/include")
set(MOTO_SHARED_NAV_PRESENTER_SOURCE
    "${MOTO_REPOSITORY_ROOT}/shared/nav_presenter/src/moto_nav_presenter.cpp")
set(MOTO_SHARED_BLE_PROTOCOL_SOURCE
    "${MOTO_REPOSITORY_ROOT}/shared/ble_protocol/src/ble_protocol.cpp")
set(MOTO_SHARED_AUDIO_SOURCE
    "${MOTO_REPOSITORY_ROOT}/shared/audio/src/chime_policy.cpp"
    "${MOTO_REPOSITORY_ROOT}/shared/audio/src/moto_voice_phrases.c")
set(MOTO_SHARED_AUDIO_INCLUDE
    "${MOTO_REPOSITORY_ROOT}/shared/audio/include")

# The shared UI source is compiled directly into this ESP-IDF component. Do not
# copy it into platforms/esp32: Web and firmware must execute the same code.
idf_component_register(
    SRCS
        "ancs_client.cpp"
        "app_main.cpp"
        "ble_nav_transport_nimble.cpp"
        "board_audio.cpp"
        "board_port_waveshare_2_16.cpp"
        "motion_heading_sensor.cpp"
        "phone_nav_bridge.cpp"
        "rtc_clock_driver.cpp"
        "${MOTO_SHARED_AUDIO_SOURCE}"
        "${MOTO_SHARED_BLE_PROTOCOL_SOURCE}"
        "${MOTO_SHARED_NAV_PRESENTER_SOURCE}"
        "${MOTO_SHARED_UI_CLOCK_FONT}"
        "${MOTO_SHARED_UI_NOTIFY_FONTS}"
        "${MOTO_SHARED_UI_FONT}"
        "${MOTO_SHARED_UI_SOURCE}"
    INCLUDE_DIRS
        "."
        "${MOTO_REPOSITORY_ROOT}/shared"
        "${MOTO_SHARED_AUDIO_INCLUDE}"
        "${MOTO_REPOSITORY_ROOT}/shared/ble_protocol/include"
        "${MOTO_REPOSITORY_ROOT}/shared/nav_presenter/include"
        "${MOTO_SHARED_UI_INCLUDE}"
    REQUIRES
        esp_timer
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_i2s
        esp_hw_support
        esp_codec_dev
        freertos
        log
        lvgl
        esp_lcd_co5300
        esp_lcd_touch_cst9220
        nvs_flash
        bt
        esp32_s3_touch_amoled_2_16
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)
