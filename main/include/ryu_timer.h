#pragma once

#include <freertos/FreeRTOS.h>  // FreeRTOS 기본 설정 및 정의
#include <freertos/timers.h>    // 소프트웨어 타이머 API 전용 헤더
#include <freertos/queue.h>
#include <lwip/sockets.h>
#include <c_library_v2/common/mavlink.h>
//#include <common/mavlink.h>


namespace TIMER{
#define DIVIDE_COUNT  10
#define DIVIDE_TIME   1000/ DIVIDE_COUNT
extern void setupTimer();

}