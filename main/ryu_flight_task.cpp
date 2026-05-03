#include "ryu_flight_task.h"

#include <driver/i2c_master.h>
#include <driver/mcpwm_prelude.h> // 신형 MCPWM
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ryu_config.h"
#include "ryu_flysky.h"
#include "ryu_MahonyFilter.h"
#include "ryu_pid.h"
#include "ryu_error_proc.h"

// icm20948, ak09916, bmp388 위의 센서 교체.
#include "ryu_icm20948.h"
#include "ryu_ak09916.h"
#include "ryu_bmp388.h"   

// gps에 있는 지자계센서.
#include "ryu_ist8310.h"
// motor
#include "ryu_servo.h"


namespace FLIGHT
{

class PerfMonitor part_timer;

uint8_t imu_error_cnt = 0;
uint8_t mag_error_cnt = 0;
uint8_t baro_error_cnt =0;
uint8_t imu_active_index = 0;
uint8_t mag_active_index = 0;
uint8_t baro_active_index = 0;

uint64_t  total_us  =0;
float calculated_dt = 0.0f;
    
void flight_task(void *pv) {
    
    uint32_t loop_cnt = 0;
    int64_t  last_time = esp_timer_get_time();

    //Watch Dog 등록.  
    esp_task_wdt_add(NULL);     

    while(true) {
        int64_t now = esp_timer_get_time();
        calculated_dt = (now- last_time);
        last_time = now; 
        if (++loop_cnt >= 400) loop_cnt = 0; // 1초 주기로 초기화
 
        //Watch Dog에게 "나 살아 있어!"" 라고 알린다.  
        esp_task_wdt_reset(); 
        
        // 연속적인 데이터 읽기 실패를 체크한다.
        esp_err_t ret_code = ESP_FAIL;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&    
// imu_active_index = 1;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
 
        static float   calculation_acc_x  = 0.0f,  calculation_acc_y  = 0.0f,  calculation_acc_z  = 0.0f;
        static float   calculation_gyro_x = 0.0f,  calculation_gyro_y = 0.0f,  calculation_gyro_z = 0.0f;
        if (imu_error_cnt < ERROR_MAX_NUM+1){
            switch(imu_active_index){
                case 0:{
                    auto [ret,macc,mgyro] = Sensor::ICM20948::Main().read_with_offset();
                    g_imu.acc   = macc;
                    g_imu.gyro  = mgyro;
                    ret_code    = ret;
                    }
                    break;
                
                case 1:{
                    auto [ret,macc,mgyro] = Sensor::ICM20948::Sub().read_with_offset();
                    g_imu.acc   = macc;
                    g_imu.gyro  = mgyro;
                    ret_code    = ret;
                    }
                    break;
            }
            if(ret_code == ESP_OK)[[likely]]{
                imu_error_cnt = 0;
                calculation_acc_x  = g_imu.acc[0] ;
                calculation_acc_y  = g_imu.acc[1] ;
                calculation_acc_z  = g_imu.acc[2] ;
                calculation_gyro_x = g_imu.gyro[0] ;
                calculation_gyro_y = g_imu.gyro[1] ;
                calculation_gyro_z = g_imu.gyro[2] ;
            }else{
                imu_error_cnt++;
                if( imu_error_cnt > ERROR_CNT_NUM ){
                    imu_active_index = (imu_active_index == 0) ? 1 : 0;
                    ESP_LOGW("IMU", "Primary IMU failed %d times, trying Backup (IMU %d)",imu_error_cnt, imu_active_index);
                    imu_error_cnt = 0;
                }
            }     
            if (imu_error_cnt ==ERROR_MAX_NUM){
                xTaskNotify(ERR::xErrorHandle, ERR::ERR_I2C_BUS_HANG, eSetBits);
                g_sys.error_hold_mode = true;
                imu_error_cnt = ERROR_MAX_NUM+1;  // overflow나지 않도록 잡아둔다.
            }
        }  

        // 두 지자계의 기본 차이값을 저장하여 main을 기준으로 sub의 값을 변경.
        // 고정 상태에서 측정하여 차이만큼 보정
        const float diff_x =  0.2784f;
        const float diff_y = -0.1175f;
        const float diff_z = -0.1285;

        static float calulation_mag_x=0.0f, calulation_mag_y=0.0f, calulation_mag_z=0.0f;

        //20 hz단위로 처리. (전체루프는 400hz이다)
        if ((loop_cnt % 20 == 0) && (mag_error_cnt < ERROR_MAX_NUM + 1)) [[likely]] { // 0. 치명적 에러 시 읽기 시도 방지 (11 이상이면 스킵)
 
            // AK09916를 테스트하기 위하여 강제로 인덱스를 1로 고정 (완료되면 삭제할것)                
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//            mag_active_index = 1;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            // 1.인덱스별로  main과 sub별 실행을 분리한다.     
            switch(mag_active_index){
                case 0:{
                    auto [ret, mag_main] = Sensor::IST8310::get_instance().read_with_offset();
                    g_imu.mag = mag_main;
                    ret_code = ret;
                    }
                    break;
                case 1:{
                    auto [ret, mag_sub] = Sensor::AK09916::get_instance().read_with_offset();
                    g_imu.mag = mag_sub;
                    ret_code = ret;
                    }
                    break;
            }

            if(ret_code == ESP_OK)[[likely]]{
                mag_error_cnt = 0;
                // main과 격차를 줄이기위하여 보정한다.
                calulation_mag_x = g_imu.mag[0] + ((mag_active_index == 1) ?  diff_x : 0.0f);
                calulation_mag_y = g_imu.mag[1] + ((mag_active_index == 1) ?  diff_y : 0.0f);
                calulation_mag_z = g_imu.mag[2] + ((mag_active_index == 1) ?  diff_z : 0.0f);
            }else{
                mag_error_cnt++;
                if (mag_error_cnt > ERROR_CNT_NUM) {
                    mag_active_index = (mag_active_index == 0) ? 1 : 0;
                    ESP_LOGW("MAG", "Primary MAG failed %d times, trying Backup (MAG %d)",mag_error_cnt, mag_active_index);
                    mag_error_cnt = 0;
                }
            }                           
            // 3.치명적 에러 발생 (10회 연속 실패)
            if (mag_error_cnt == ERROR_MAX_NUM) {
                ESP_LOGW("MAG", "xTaskNotify()-> sending to signal(ERR_MAG_DEV_INVALID)");
                xTaskNotify(ERR::xErrorHandle, ERR::ERR_I2C_BUS_HANG, eSetBits);
                g_sys.error_hold_mode = true;
                mag_error_cnt = ERROR_MAX_NUM+1; // 차단
            }             
        }

    
        // if (loop_cnt % 32 == 0)  
        //     printf("\n%8.4f \t %8.4f \t %8.4f \t %8.4f \t %8.4f \t %8.4f \t %8.4f \t %8.4f \t %8.4f \t %8.6f",
        //         calculation_acc_x,
        //         calculation_acc_y,
        //         calculation_acc_z,
        //         calculation_gyro_x,
        //         calculation_gyro_y,
        //         calculation_gyro_z,
        //         calulation_mag_x,
        //         calulation_mag_y,
        //         calulation_mag_z,calculated_dt );  

        { // qgc로 보내는 데이터
            g_attitude.rollspeed    = calculation_gyro_x ;
            g_attitude.pitchspeed   = calculation_gyro_y ;
            g_attitude.yawspeed     = calculation_gyro_z ;
        }
        AHRS::MahonyAHRSupdate(   
                            calculation_gyro_x * DEG_TO_RAD,
                            calculation_gyro_y * DEG_TO_RAD, 
                            calculation_gyro_z * DEG_TO_RAD, 
                            calculation_acc_x, 
                            calculation_acc_y, 
                            calculation_acc_z, 
                            calulation_mag_x,
                            calulation_mag_y,
                            calulation_mag_z,
                            dt
                        );

        // 각도 추출 ( 단위 DEG)
        // QGroundControl에 보내기 위하여 (-)부호를 처리해야하는데 
        // telemetry의  mavlink_msg_attitude_pack에서 (-) 부호 처리하여 보낸다.
        // 수정사항 qgc에 보낼정보는 따로 담아서 보관하도록 해야할것 같다. roll정보를 pid에서 사용하기때문에 변하면 안되다.
        float sinP =0.0f,actual_compass_heading=0.0f;
        {
            using namespace AHRS;      
            const float q0q0 = q0 * q0;
            const float q1q1 = q1 * q1;
            const float q2q2 = q2 * q2;
            const float q3q3 = q3 * q3;             

            g_attitude.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0q0 - q1q1 - q2q2 + q3q3) * RAD_TO_DEG;
            sinP      = std::clamp(2.0f * (q0 * q2 - q1 * q3), -1.0f, 1.0f);
            g_attitude.pitch= asinf(sinP) * RAD_TO_DEG;    
            actual_compass_heading   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0q0 + q1q1 - q2q2 - q3q3);
        }
        // 2. 편각 보정 (-7.7도 적용) 하여 '진북' 기준으로 업데이트
        // 진북에서 -7.7도정도에 자북이 존재하므로 현재 자북을 구한상태에 +7.7도를 더해야만 진북이된다.
        float declinationAngle = 7.7f * DEG_TO_RAD;
        actual_compass_heading += declinationAngle;

// 변수 변화확인용
//if (loop_cnt % 16 == 0) ESP_LOGI("MAIN", "yaw_rad: %8.3f", yaw_rad);

        // 3. 각도 범위 정규화 (-PI ~ +PI) -> PID 제어에 유리함
        // 3. 각도 범위 정규화 (-PI ~ +PI)
        if (actual_compass_heading >  M_PI)         actual_compass_heading -= 2.0f * M_PI;
        else if (actual_compass_heading < -M_PI)    actual_compass_heading += 2.0f * M_PI;

        // 4. 이 yaw_rad를 기반으로 최종 yaw(degree)와 heading_deg 생성
        g_attitude.yaw = actual_compass_heading * RAD_TO_DEG; // 이제 이 yaw는 '진북' 기준입니다.

        // 5. QGC 나침반용 (0 ~ 360도)
        float heading_deg = g_attitude.yaw;
        while (heading_deg < 0)    heading_deg += 360.0f;
        while (heading_deg >= 360) heading_deg -= 360.0f;

        g_attitude.heading = heading_deg;

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@        
//        g_sys.is_armed =true;
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        if(!g_sys.is_armed) [[unlikely]]{                
            // 시동 안 걸렸을 때는 모터 정지 및 PID 적분항 초기화
            for(auto& comp : SERVO::comparators) mcpwm_comparator_set_compare_value(comp, 1000);
            // 시동을 켜는 순간 '튀는' 현상을 방지합니다.
            auto& pid = Controller::PID::get_instance();
            pid.reset_pid(&pid.pid_roll_angle);
            pid.reset_pid(&pid.pid_pitch_angle);
            pid.reset_pid(&pid.pid_yaw_angle);
            pid.reset_pid(&pid.pid_roll_rate);
            pid.reset_pid(&pid.pid_pitch_rate);
            pid.reset_pid(&pid.pid_yaw_rate);
            pid.reset_pid(&pid.pid_alt_pos);  
        }else{
            // 조종기 입력값 계산  (실제 조종기에서 들어오는 값들을 scale 작업을 하여 감도를 조절한다.)
            // 감도를 높이려면 값을 키우면 된다.              
            float tg_throttle = g_rc.throttle * 10.0f; 
            float tg_roll     = g_rc.roll     * 0.3f; 
            float tg_pitch    = g_rc.pitch    * 0.3f; 
            float tg_yaw_rate = g_rc.yaw      * 1.5f; 
            
            static float target_alt = 0.0f;
            static float alt_throttle_offset = 0.0f;   
            static bool last_alt_hold_state = false;
            // 고도 PID 제어를 위해 현재 고도와 상승률을 계산한다.
            static float filtered_alt =0.0f, filtered_climb_rate=0.0f;
            // 3. 고도 유지 모드 스위치 처리                               
            if (g_sys.manual_hold_mode  && !last_alt_hold_state) 
            {
                target_alt = g_baro.filtered_altitude; // 모드가 켜지는 순간의 고도를 목표로 고정
                alt_throttle_offset = 0.0f; // PID 보정값 초기화
                filtered_alt = 0.0f; // 고도 초기화
                filtered_climb_rate = 0.0f; // 상승률 초기화
            }
            last_alt_hold_state = g_sys.manual_hold_mode ;

            // 1초에 한번 파라미터 테이블에서 최신 PID 계수를 읽어옵니다
            //if(loop_cnt==100) sync_pid_from_params();
      
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// barro_active_index = 1;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      

            // 고도 PID 및 자세 PID (기존 로직 유지)
            if ((loop_cnt % 8 ==0) && (baro_error_cnt < ERROR_MAX_NUM + 1)){  //50hz단위로 처리. (전체루프는 400hz이다)
                float temp_alt = 0.0f,temp_rate = 0.0f;
                //esp_err_t ret_code;
                switch(baro_active_index){
                    case 0:{
                            std::tie(ret_code,temp_alt)   = Sensor::BMP388::Main().get_relative_altitude();
                            temp_rate  = Sensor::BMP388::Main().update_climb_rate();
                            //ret_code   = bmp388_main.get_last_error();
                        }    
                        break;
                    case 1:{
                            std::tie(ret_code,temp_alt)   = Sensor::BMP388::Sub().get_relative_altitude();
                            temp_rate  = Sensor::BMP388::Sub().update_climb_rate();
                            //ret_code   = bmp388_sub.get_last_error();
                        }
                        break;
                }
                if (ret_code ==ESP_OK)[[likely]]{
                    baro_error_cnt =0;
                    filtered_alt        = temp_alt;
                    filtered_climb_rate = temp_rate;
                }else{
                    baro_error_cnt++;
                    if(baro_error_cnt > ERROR_CNT_NUM) {
                        baro_active_index = (baro_active_index == 0) ? 1 : 0;
                        ESP_LOGW("BARO", "Primary BARO failed %d times, trying Backup (BARO %d)",baro_error_cnt, baro_active_index);                                         
                        baro_error_cnt =0;
                    }
                }
                if (baro_error_cnt == ERROR_MAX_NUM) {
                    ESP_LOGW("BARO", "xTaskNotify()-> sending to signal(ERR_BUS_HANG)");
                    xTaskNotify(ERR::xErrorHandle, ERR::ERR_I2C_BUS_HANG, eSetBits);
                    g_sys.error_hold_mode = true;
                    baro_error_cnt = ERROR_MAX_NUM + 1; // 차단
                }
              
                if (filtered_alt > 500.0f) filtered_alt = 0.0f; // 비정상적인 고도 차단
                if (filtered_alt <= 0.0f) filtered_alt = 0.0f; // 음수 고도 방지
                if (fabsf(filtered_climb_rate) > 10.0f) filtered_climb_rate = 0.0f; // 비정상적 상승률 방지
                if (g_sys.error_hold_mode) {
                    filtered_alt = g_baro.filtered_altitude; // 에러 모드에서는 => 마지막 데이터로 안정된 고도 유지
                    filtered_climb_rate = 0.0f;                      // 상승률은 0으로 고정
                }

                g_baro.filtered_altitude = filtered_alt;
                
                auto& pid = Controller::PID::get_instance();
                if(g_sys.manual_hold_mode) {
                    // Outer Loop: 고도 유지 (P 제어 위주)
                    float target_climb_rate = pid.run_pid_angle(&pid.pid_alt_pos, target_alt, filtered_alt, 0.025f, false);
                    target_climb_rate       = std::clamp(target_climb_rate, -1.5f, 1.5f);
                    // Inner Loop: 수직 속도 유지 (PI 제어 위주)
                    alt_throttle_offset = pid.run_pid_rate(&pid.pid_alt_rate, target_climb_rate, filtered_climb_rate, 0.025f);
                    alt_throttle_offset = std::clamp(alt_throttle_offset, -150.0f, 150.0f);
                } else {
                    filtered_alt = 0.0f;
                    filtered_climb_rate = 0.0f;
                    alt_throttle_offset = 0.0f;
                    pid.reset_pid(&pid.pid_alt_pos);
                    pid.reset_pid(&pid.pid_alt_rate);
                }

            }
            
            auto& pid = Controller::PID::get_instance();                
            // --- [1단계: Outer Loop - 각도 제어] ---
            // 조종기 스틱(tg_roll) -> 목표 각도 -> 목표 각속도(deg/s) 출력
            float target_rate_roll  = pid.run_pid_angle(&pid.pid_roll_angle,  tg_roll,  g_attitude.roll,  dt, false);
            float target_rate_pitch = pid.run_pid_angle(&pid.pid_pitch_angle, tg_pitch, g_attitude.pitch, dt, false);
            //float target_rate_yaw = pid.run_pid_angle(&pid.pid_yaw_angle,   tg_yaw_rate, g_attitude.yaw, dt, true);

            // Yaw는 사용자의 스틱 입력(tg_yaw_rate)을 목표 각속도로 직접 사용하거나, 
            // 현재처럼 Heading Hold를 원하시면 아래처럼 목표 각도를 유지하게 합니다.
            static float target_yaw_angle = 0.0f;
            target_yaw_angle += tg_yaw_rate * dt; 
            if (target_yaw_angle > 180.0f) target_yaw_angle -= 360.0f;
            if (target_yaw_angle < -180.0f) target_yaw_angle += 360.0f;
            float target_rate_yaw = pid.run_pid_angle(&pid.pid_yaw_angle, target_yaw_angle, g_attitude.yaw, dt, true);

            // --- [2단계: Inner Loop - 각속도 제어] ---
            // 목표 각속도 -> 현재 자이로 값(g_imu.gyro)과 비교 -> 최종 모터 출력(PWM 변위)
            // g_imu.gyro[0]: Roll속도, [1]: Pitch속도, [2]: Yaw속도
            float out_roll  = pid.run_pid_rate(&pid.pid_roll_rate,  target_rate_roll,  g_imu.gyro[0], dt);
            float out_pitch = pid.run_pid_rate(&pid.pid_pitch_rate, target_rate_pitch, g_imu.gyro[1], dt);
            float out_yaw   = pid.run_pid_rate(&pid.pid_yaw_rate,   target_rate_yaw,   g_imu.gyro[2], dt);

            // throttle이 거의 0일 때는 yaw 제어를 억제하여
            // 하한 클램프와 충돌하는 현상을 방지한다.
            // 적분/이전 오차도 같이 초기화.
            if (tg_throttle < 5.0f) {
                out_yaw = 0.0f;
                pid.pid_yaw_angle.integral = 0.0f;
                pid.pid_yaw_angle.err_prev = 0.0f;
            }
            // 작은 값은 dead‑band 처리
            if (fabsf(out_yaw) < 1.0f) {
                out_yaw = 0.0f;
            }

            //QGC 캘리브레이션 테스트 목적=====================================================
            // qgc_roll_pid.current    = g_attitude.roll;
            // qgc_roll_pid.target     = tg_roll;
            // qgc_roll_pid.output     = out_roll;
            //===============================================================================
            float base_pwm = 1000.0f + std::max(tg_throttle + alt_throttle_offset, 50.0f);
            float motor[4];
            motor[0] = std::clamp(base_pwm - out_pitch - out_roll - out_yaw, 1050.0f, 2000.0f);
            motor[1] = std::clamp(base_pwm - out_pitch + out_roll + out_yaw, 1050.0f, 2000.0f);
            motor[2] = std::clamp(base_pwm + out_pitch - out_roll + out_yaw, 1050.0f, 2000.0f);
            motor[3] = std::clamp(base_pwm + out_pitch + out_roll - out_yaw, 1050.0f, 2000.0f);

            // static_cast는 유지하되, 타입 추론은 auto에게 맡깁니다.
            for (size_t i = 0; auto comp : SERVO::comparators) {
                mcpwm_comparator_set_compare_value(comp, static_cast<uint32_t>(motor[i++]));
            }
        }           
        total_us = esp_timer_get_time() - last_time;
        int64_t current_time;
        while ((current_time = esp_timer_get_time()) - last_time < INTERVAL_US) {
            if (INTERVAL_US - (current_time - last_time) > 1200) {
                vTaskDelay(1); 
            }
        }
    }
}
}//namespace FLIGHT
