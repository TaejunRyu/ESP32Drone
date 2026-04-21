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
#include <driver/i2c_master.h>
#include <esp_log.h>

// 에러 비트 정의 (Bitmask 방식)
namespace ERR
{

// 시스템 상태 비트 정의
inline constexpr uint32_t SYS_HEALTH_IMU_OK     =  (1 << 0);
inline constexpr uint32_t SYS_HEALTH_MAG_OK     =  (1 << 1);
inline constexpr uint32_t SYS_HEALTH_BARO_OK    =  (1 << 2);
inline constexpr uint32_t SYS_HEALTH_GPS_OK     =  (1 << 3);
inline constexpr uint32_t SYS_HEALTH_RC_OK      =  (1 << 4);
inline constexpr uint32_t SYS_HEALTH_BATTERY_OK =  (1 << 5);
inline constexpr uint32_t SYS_HEALTH_ALL_OK     =  (SYS_HEALTH_IMU_OK   | 
                                                    SYS_HEALTH_MAG_OK   | 
                                                    SYS_HEALTH_BARO_OK  | 
                                                    SYS_HEALTH_GPS_OK   | 
                                                    SYS_HEALTH_RC_OK    | 
                                                    SYS_HEALTH_BATTERY_OK);


// 모든 필수 센서가 정상인 상태 (예: 위치 제어 모드용)
inline constexpr uint32_t MASK_REQUIRED_LOITER  = ( SYS_HEALTH_IMU_OK   | 
                                                    SYS_HEALTH_GPS_OK   | 
                                                    SYS_HEALTH_RC_OK    | 
                                                    SYS_HEALTH_BARO_OK);
// 최소 비행 가능 상태 (예: 수동 자세 제어용)
inline constexpr uint32_t MASK_REQUIRED_MANUAL  = ( SYS_HEALTH_IMU_OK | 
                                                    SYS_HEALTH_RC_OK);

// 에러 유형 정의 (비트마스크 방식)
inline constexpr uint32_t  ERR_I2C_BUS_HANG     =  (1 << 0);  // I2C 버스 고착      (FATAL)     ==> I2C LINE 전체 복구처리

// 문제는 IMU,MAG,BARO가 문제 발생할경우 I2C 전체를 복구하는것이 이점이 있다.
// 일단 문제가 발생하면 IMU만 복구 하면 일단 성공이다...
inline constexpr uint32_t  ERR_IMU_DEV_INVALID  =  (1 << 1);  // IMU DEVICE 이상    (FATAL)     ==> 복구처리
inline constexpr uint32_t  ERR_MAG_DEV_INVALID  =  (1 << 2);  // MAG DEVICE 이상    (WARNNG)    ==> HOLD_MODE 전환  
inline constexpr uint32_t  ERR_BARO_DEV_INVALID =  (1 << 3);  // BARO DEVICE 이상   (WARNNG)    ==> HOLD_MODE 전환

inline constexpr uint32_t  ERR_RC_LOST          =  (1 << 4);  // 조종기 신호 끊김    (FATAL)     ==> 자동복귀 (RTL)
inline constexpr uint32_t  ERR_BATTERY_LOW      =  (1 << 5);  // 배터리 저전압       (CRITICAL)  ==> 자동복귀 (RTL)
inline constexpr uint32_t  ERR_GPS_TIMEOUT      =  (1 << 6);  // GPS 2초이상 신호(X) (CRITICAL)  ==> HOLD_MODE 전환
inline constexpr uint32_t  ERR_GPS_SATS_LOW     =  (1 << 7);  // GPS 위성수 부족     (WARNNG)    ==> HOLD_MODE 전환

// 태스크 핸들 (Core 0의 에러 태스크를 지칭)
extern TaskHandle_t xErrorHandle;
// 시스템 통합 상태 (비트마스크)
extern volatile uint32_t g_system_health; // 초기값은 모두 OK

extern void error_manager_task(void *pvParameters);
extern esp_err_t reinit_all_sensors(i2c_master_bus_handle_t handle);
}
