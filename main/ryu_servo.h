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

#include "ryu_config.h"

namespace SERVO 
{

// FC가 공중 운반물이 있을경우 떨어뜨리는 SERVO MOTOR 
//inline constexpr  gpio_num_t   SERVO_MOTOR_PIN   = GPIO_NUM_25;
inline constexpr  gpio_num_t   MOTOR_FRONT_LEFT  = GPIO_NUM_26;
inline constexpr  gpio_num_t   MOTOR_FRONT_RIGHT = GPIO_NUM_27;
inline constexpr  gpio_num_t   MOTOR_REAR_LEFT   = GPIO_NUM_32;
inline constexpr  gpio_num_t   MOTOR_REAR_RIGHT  = GPIO_NUM_33;


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