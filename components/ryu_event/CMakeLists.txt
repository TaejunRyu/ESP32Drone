#file(GLOB_RECURSE SOURCES "*.cpp") # 폴더 내 모든 cpp 파일을 SOURCES 변수에 담음

idf_component_register(
    SRCS 
    INCLUDE_DIRS "."          # "현재 폴더(.)를 헤더 경로로 쓰겠다"는 뜻
    REQUIRES    
        esp_system
        esp_event
)
