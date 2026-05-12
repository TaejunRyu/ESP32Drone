/**
 * @file ryu_failsafe.h
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

namespace Service
{
class  FailSafe{
    private:
        FailSafe() = default;  
        ~FailSafe() = default;
        static constexpr const char* TAG = "FailSafe";
    public:
        static FailSafe& get_instance() {
            static FailSafe instance; 
            return instance;
        }
        FailSafe(const FailSafe&) = delete;
        FailSafe& operator=(const FailSafe&) = delete;

        enum sys_health_type{
            SYS_HEALTH_IMU_OK       = (1 << 0),
            SYS_HEALTH_MAG_OK       = (1 << 1),
            SYS_HEALTH_BARO_OK      = (1 << 2),
            SYS_HEALTH_GPS_OK       = (1 << 3),
            SYS_HEALTH_RC_OK        = (1 << 4),
            SYS_HEALTH_BATTERY_OK   = (1 << 5)
        };

        // 0000 0000 0000 0000 0000 0000 0011 1111
        static inline constexpr uint32_t SYS_HEALTH_ALL_OK     =  ( SYS_HEALTH_IMU_OK   | 
                                                                    SYS_HEALTH_MAG_OK   | 
                                                                    SYS_HEALTH_BARO_OK  | 
                                                                    SYS_HEALTH_GPS_OK   | 
                                                                    SYS_HEALTH_RC_OK    | 
                                                                    SYS_HEALTH_BATTERY_OK);


        // 모든 필수 센서가 정상인 상태 (예: 위치 제어 모드용)
        // 0000 0000 0000 0000 0000 0000 0001 1101
        static inline constexpr uint32_t MASK_REQUIRED_LOITER  = (  SYS_HEALTH_IMU_OK   | 
                                                                    SYS_HEALTH_GPS_OK   | 
                                                                    SYS_HEALTH_RC_OK    | 
                                                                    SYS_HEALTH_BARO_OK);
        // 최소 비행 가능 상태 (예: 수동 자세 제어용)
        // 0000 0000 0000 0000 0000 0000 0001 0001
        static inline constexpr uint32_t MASK_REQUIRED_MANUAL  = (  SYS_HEALTH_IMU_OK | 
                                                                    SYS_HEALTH_RC_OK);

        // 태스크 핸들 (Core 0의 에러 태스크를 지칭)
        TaskHandle_t _task_handle = nullptr;
        // 시스템 통합 상태 (비트마스크)
        volatile uint32_t system_health = SYS_HEALTH_ALL_OK; // 초기값은 모두 OK 

        esp_err_t initialize();
        bool is_initialized(){return _initialized;};
        esp_err_t reinit_all_sensors();
        static void event_handler_relay(void *arg, esp_event_base_t base, int32_t id, void *data);
        void update_health(Event::fault_event_data_t *fault);
        static void failsafe_manager_task(void *pvParameters);
        BaseType_t start_task();

    private:
        bool _initialized = false;

};



}
