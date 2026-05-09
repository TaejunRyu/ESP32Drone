# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS 
        "." 
    REQUIRES                   
            ryu_driver          
            ryu_config
            ryu_controller      
            ryu_service     
)
 
# 프로젝트 전체(모든 컴포넌트)에 적용


# 3. 컴파일 옵션 설정
# 모든 옵션을 target_compile_options 내부에 포함시켜야 합니다.
#target_compile_options(${COMPONENT_LIB} PRIVATE 
#    -Wno-deprecated-declarations
#    -Wno-address-of-packed-member
#    -O3
#    -ffast-math
#    -fno-finite-math-only
#    -std=gnu++26
#)
