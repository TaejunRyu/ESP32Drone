/**
 * @file error_proc.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_sensor_event.h"

// 에러 비트 정의 (Bitmask 방식)
namespace Service
{
class  FailSafe{
    private:
        FailSafe();
    public:
        // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        FailSafe(const FailSafe&) = delete;
        FailSafe& operator=(const FailSafe&) = delete;
        ~FailSafe();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static FailSafe& get_instance() {
            static FailSafe* instance = new FailSafe(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  

        // 시스템 상태 비트 정의
        static inline constexpr uint32_t SYS_HEALTH_IMU_OK     =  (1 << 0);
        static inline constexpr uint32_t SYS_HEALTH_MAG_OK     =  (1 << 1);
        static inline constexpr uint32_t SYS_HEALTH_BARO_OK    =  (1 << 2);
        static inline constexpr uint32_t SYS_HEALTH_GPS_OK     =  (1 << 3);
        static inline constexpr uint32_t SYS_HEALTH_RC_OK      =  (1 << 4);
        static inline constexpr uint32_t SYS_HEALTH_BATTERY_OK =  (1 << 5);
        static inline constexpr uint32_t SYS_HEALTH_ALL_OK     =  ( SYS_HEALTH_IMU_OK   | 
                                                                    SYS_HEALTH_MAG_OK   | 
                                                                    SYS_HEALTH_BARO_OK  | 
                                                                    SYS_HEALTH_GPS_OK   | 
                                                                    SYS_HEALTH_RC_OK    | 
                                                                    SYS_HEALTH_BATTERY_OK);


        // 모든 필수 센서가 정상인 상태 (예: 위치 제어 모드용)
        static inline constexpr uint32_t MASK_REQUIRED_LOITER  = (  SYS_HEALTH_IMU_OK   | 
                                                                    SYS_HEALTH_GPS_OK   | 
                                                                    SYS_HEALTH_RC_OK    | 
                                                                    SYS_HEALTH_BARO_OK);
        // 최소 비행 가능 상태 (예: 수동 자세 제어용)
        static inline constexpr uint32_t MASK_REQUIRED_MANUAL  = (  SYS_HEALTH_IMU_OK | 
                                                                    SYS_HEALTH_RC_OK);

        // 에러 유형 정의 (비트마스크 방식)
        // I2C 버스 고착      (FATAL)     ==> I2C LINE 전체 복구처리
        static inline constexpr uint32_t  ERR_I2C_BUS_HANG     =  (1 << 0);  

        // 문제는 IMU,MAG,BARO가 문제 발생할경우 I2C 전체를 복구하는것이 이점이 있다.
        // 일단 문제가 발생하면 IMU만 복구 하면 일단 성공이다...
        static inline constexpr uint32_t  ERR_IMU_DEV_INVALID  =  (1 << 1);  // IMU DEVICE 이상    (FATAL)     ==> 복구처리
        static inline constexpr uint32_t  ERR_MAG_DEV_INVALID  =  (1 << 2);  // MAG DEVICE 이상    (WARNNG)    ==> HOLD_MODE 전환  
        static inline constexpr uint32_t  ERR_BARO_DEV_INVALID =  (1 << 3);  // BARO DEVICE 이상   (WARNNG)    ==> HOLD_MODE 전환

        static inline constexpr uint32_t  ERR_RC_LOST          =  (1 << 4);  // 조종기 신호 끊김    (FATAL)     ==> 자동복귀 (RTL)
        static inline constexpr uint32_t  ERR_BATTERY_LOW      =  (1 << 5);  // 배터리 저전압       (CRITICAL)  ==> 자동복귀 (RTL)
        static inline constexpr uint32_t  ERR_GPS_TIMEOUT      =  (1 << 6);  // GPS 2초이상 신호(X) (CRITICAL)  ==> HOLD_MODE 전환
        static inline constexpr uint32_t  ERR_GPS_SATS_LOW     =  (1 << 7);  // GPS 위성수 부족     (WARNNG)    ==> HOLD_MODE 전환

        // 태스크 핸들 (Core 0의 에러 태스크를 지칭)
        TaskHandle_t _task_handle = nullptr;
        // 시스템 통합 상태 (비트마스크)
        volatile uint32_t system_health = 0xFFFFFFFF; // 초기값은 모두 OK 

        esp_err_t initialize();
        esp_err_t reinit_all_sensors();
        static void event_handler_relay(void *arg, esp_event_base_t base, int32_t id, void *data);
        void update_health(fault_event_data_t *fault);
        static void failsafe_manager_task(void *pvParameters);
        void start_task();

    private:
        bool _initialized = false;
        static const char* TAG;

};



}
