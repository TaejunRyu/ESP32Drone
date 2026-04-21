cmake_minimum_required(VERSION 3.16.0)

set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/components")

# 제외할 하위 프로젝트 폴더명을 리스트에 추가 (예: other_project_folder)
set(EXCLUDE_COMPONENTS "ground_station" "document")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32Drone)
