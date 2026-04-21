/**
 * @file servo.h
 * @author ryutaejun (ryuwillow@gmail.com)
 * @brief   드론의 ESC에 신호를 보내는 작업을 하기 위한 초기화.
 *          
 * @version 0.1
 * @date 2026-03-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <math.h>
#include <driver/mcpwm_prelude.h> // 신형 MCPWM

#include "ryu_buzzer.h"

namespace SERVO
{
constexpr int MOTOR_PINS[4] = {MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT}; // FR, FL, RL, RR

/**
 * @brief 드론의 4개의 모터 출력
 * 
 */
extern mcpwm_cmpr_handle_t comparators[4];      // 모터 듀티 제어용
/**
 * @brief 낚시 미끼 드랍용(servo motor)
 * 
 */
extern mcpwm_cmpr_handle_t servo_comparator;    // 서보 각도 조절용

extern void initialize();

// 모든 모터의 중지...
inline esp_err_t stop_all_motors(){
    esp_err_t ret = ESP_OK;
    for(auto& comp : comparators){ 
        ret = mcpwm_comparator_set_compare_value(comp, 1000);
        if (ret != ESP_OK){
            return ret;
        }
    }
    return ESP_OK;
}

}