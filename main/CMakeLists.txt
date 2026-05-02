# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

idf_component_register(
    SRCS ${app_sources}
        
    INCLUDE_DIRS 
        "." 

    REQUIRES                   
                esp_adc             esp_wifi            nvs_flash 
                esp_driver_uart     esp_driver_gpio     esp_driver_mcpwm 
                esp_driver_i2c      esp_driver_ledc     esp_timer 
                bootloader_support  esp_pm              freertos
                esp_system          lwip                mavlink
                ryu_sensor          ryu_config          ryu_driver
                ryu_controller
                
    PRIV_REQUIRES espressif__mavlink       # 이 부분을 반드시 확인하세요!
)

# 3. 컴파일 옵션 설정
# 모든 옵션을 target_compile_options 내부에 포함시켜야 합니다.
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Wno-deprecated-declarations
    -Wno-address-of-packed-member
    -O3
    -ffast-math
    -fno-finite-math-only
    -std=gnu++26
)
