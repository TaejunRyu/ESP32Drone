# 현재 폴더의 모든 .cpp 파일을 찾아서 SOURCES 변수에 담음
file(GLOB SOURCES "*.cpp")
idf_component_register(
     SRCS ${SOURCES}
    INCLUDE_DIRS "."             # "현재 폴더(.)를 헤더 경로로 쓰겠다"는 뜻
                 "../ryu_driver"
    REQUIRES    
        esp_driver_gpio 
        esp_driver_i2c  
        esp_timer
        
)
