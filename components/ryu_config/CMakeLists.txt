#file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS "ryu_config.cpp"
    INCLUDE_DIRS "."          # "현재 폴더(.)를 헤더 경로로 쓰겠다"는 뜻
    REQUIRES    
            esp_driver_i2c esp_driver_uart esp_adc lwip log  mavlink
)
