# 현재 폴더의 모든 .cpp 파일을 찾아서 SOURCES 변수에 담음
file(GLOB SOURCES "*.cpp")
idf_component_register(
     SRCS ${SOURCES}
    INCLUDE_DIRS    "."
                    "../../main"                  
    REQUIRES    
        esp_driver_gpio esp_driver_i2c  esp_driver_ledc esp_adc
        ryu_config
)
