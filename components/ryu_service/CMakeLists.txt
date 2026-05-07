# 현재 폴더의 모든 .cpp 파일을 찾아서 SOURCES 변수에 담음
file(GLOB SOURCES "*.cpp")
idf_component_register(
     SRCS ${SOURCES}
    INCLUDE_DIRS 
        "."
    REQUIRES
        esp_timer
        esp_event
        esp_driver_gpio
        esp_driver_mcpwm
        esp_pm
        esp_wifi
        nvs_flash
        driver
        freertos
        mavlink
        ryu_config
        ryu_controller
        ryu_driver 
        ryu_sensor
        ryu_event
)
