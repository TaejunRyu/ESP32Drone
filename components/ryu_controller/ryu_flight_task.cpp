#include "ryu_flight_task.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ryu_flysky.h"
#include "ryu_MahonyFilter.h"
#include "ryu_pid.h"
#include "ryu_wifi.h"
#include "ryu_failsafe.h"
#include "ryu_mavlink.h"
#include "ryu_telemetry.h"
#include "ryu_timer.h"
#include "ryu_buzzer.h"
#include "ryu_gps.h"
#include "ryu_icm20948.h"
#include "ryu_ak09916.h"
#include "ryu_bmp388.h"   
#include "ryu_ist8310.h"
#include "ryu_motor.h"
#include "ryu_battery.h"
#include "ryu_i2c.h"
#include "ryu_mag.h"

namespace Controller
{

/**
 * @brief 
 *      1. flight task를 실행하려면 초기화를 반드시 처리해야한다.
 * @return esp_err_t 
 */
esp_err_t Flight::initialize()
{
    if(_initialized){ 
        return ESP_OK;
    }
    
    esp_err_t err;
    auto& buzzer        = Driver::Buzzer::get_instance();
        err = buzzer.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Buzzer module initialize failed.");
            return err;
        }        
        vTaskDelay(pdMS_TO_TICKS(50));
        //buzzer.sound_system_start();
    
    auto& i2c           = Driver::I2C::get_instance();
        err = i2c.initialize();
        if (err != ESP_OK){
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& icm20948_main = Sensor::ICM20948::Main();
        err = icm20948_main.initialize();
        if (err != ESP_OK){
            return err;
        }
        err = icm20948_main.enable_mag_bypass();        
        if (err != ESP_OK){
            ESP_LOGE(TAG, "ICM20948 main module Bypass mode setting failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& icm20948_sub  = Sensor::ICM20948::Sub();
        err = icm20948_sub.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "ICM20948 sub module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& ist8310       = Sensor::IST8310::get_instance();
        err = ist8310.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "IST8310 module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& ak09916       = Sensor::AK09916::get_instance();
        err = ak09916.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "AK09916 module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& bmp388_main   = Sensor::BMP388::Main();
        err = bmp388_main.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "BMP338 Main module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& bmp388_sub    = Sensor::BMP388::Sub();
        err = bmp388_sub.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "BMP338 Sub module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));

    auto& espnow        = Service::EspNow::get_instance();
        err = espnow.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Espnow module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& battery       = Driver::Battery::get_instance();
        err = battery.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Baterry module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& mahony        = Service::Mahony::get_instance();
        err = mahony.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Mahony module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& motor         = Driver::Motor::get_instance();
        err = motor.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Motor module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& gps           = Sensor::Gps::get_instance();
        err = gps.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Gps module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& flysky        = Service::Flysky::get_instance();
        err = flysky.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Flysky module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& mavlink       = Service::Mavlink::get_instance();
        err = mavlink.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Mavlink module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& telemetry     = Service::Telemetry::get_instance();
        err = telemetry.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Telemetry module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& pid           = Controller::PID::get_instance();
        err = pid.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Pid module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& failsafe      = Service::FailSafe::get_instance();
        err = failsafe.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "FailSafe module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    auto& timer         = Service::Timer::get_instance();
        err = timer.intiallize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Timer module initialize failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));  

    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return err;
}

/**
 * @brief 
 *      1. 실제 비행 로직이 처리되는 task.
 *      2. class들은 initialize에서 초기화.
 * @param pvParameters 
 */
void Flight::flight_task(void *pvParameters)
{
    auto flight = static_cast<Flight*>(pvParameters);
    auto& motor         = Driver::Motor::get_instance();
    auto& icm20948_main = Sensor::ICM20948::Main();
    //auto& icm20948_sub  = Sensor::ICM20948::Sub();
    //auto& ak09916       = Sensor::AK09916::get_instance();
    auto& bmp388_main   = Sensor::BMP388::Main();
    //auto& bmp388_sub    = Sensor::BMP388::Sub();
    //auto& ist8310       = Sensor::IST8310::get_instance();
    auto& pid           = Controller::PID::get_instance();
    auto& mahony        = Service::Mahony::get_instance();
    //auto& failsafe      = Service::FailSafe::get_instance();
    
    // ist8310,ak09916을 한 class에 묶어서 내부에서 일기로직을 처리.
    auto& managed_mag  = Service::ManageMag::get_instance();
    if(!managed_mag.is_initialized())
        managed_mag.initialize();

    uint32_t loop_cnt = 0;
    int64_t  last_time = esp_timer_get_time();

    //Watch Dog 등록.  
    esp_task_wdt_add(flight->_task_handle);     
     
    while(true) {
        int64_t now = esp_timer_get_time();
        flight->calculated_dt = (now- last_time);
        last_time = now; 
        if (++loop_cnt >= 400) loop_cnt = 0; // 1초 주기로 초기화
 
        //Watch Dog에게 "나 살아 있어!"" 라고 알린다.  
        esp_task_wdt_reset(); 
        
        // 연속적인 데이터 읽기 실패를 체크한다.
        esp_err_t ret_code = ESP_FAIL;
 
        static float    calculation_acc_x  = 0.0f,  
                        calculation_acc_y  = 0.0f,  
                        calculation_acc_z  = 0.0f;
        static float    calculation_gyro_x = 0.0f,  
                        calculation_gyro_y = 0.0f,  
                        calculation_gyro_z = 0.0f;

        auto [ret,macc,mgyro] = icm20948_main.Managed_read_with_offset();
        if (ret == ESP_OK){
            calculation_acc_x  = macc[0] ;
            calculation_acc_y  = macc[1] ;
            calculation_acc_z  = macc[2] ;
            calculation_gyro_x = mgyro[0] ;
            calculation_gyro_y = mgyro[1] ;
            calculation_gyro_z = mgyro[2] ;
        }

        static float    calulation_mag_x=0.0f, 
                        calulation_mag_y=0.0f, 
                        calulation_mag_z=0.0f;
        if (loop_cnt % 8 == 0){ // 50HZ
            auto [ret_mag, mag] = managed_mag.Managed_read_with_offset();
            if(ret_mag == ESP_OK){
                calulation_mag_x=mag[0]; 
                calulation_mag_y=mag[1]; 
                calulation_mag_z=mag[2];
            } 
            //ESP_LOGI(TAG,"mx:%f , my:%f , mz:%f",calulation_mag_x,calulation_mag_y,calulation_mag_z);
            ret_code = ret;
        }
        mahony.MahonyAHRSupdate(   
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

        { // qgc로 보내는 데이터
            g_attitude.rollspeed    = calculation_gyro_x ;
            g_attitude.pitchspeed   = calculation_gyro_y ;
            g_attitude.yawspeed     = calculation_gyro_z ;
        }
 
        // 각도 추출 ( 단위 DEG)
        // QGroundControl에 보내기 위하여 (-)부호를 처리해야하는데 
        // telemetry의  mavlink_msg_attitude_pack에서 (-) 부호 처리하여 보낸다.
        // 수정사항 qgc에 보낼정보는 따로 담아서 보관하도록 해야할것 같다. roll정보를 pid에서 사용하기때문에 변하면 안되다.
        float sinP =0.0f,actual_compass_heading=0.0f;

        const float q0q0 = mahony.q0 * mahony.q0;
        const float q1q1 = mahony.q1 * mahony.q1;
        const float q2q2 = mahony.q2 * mahony.q2;
        const float q3q3 = mahony.q3 * mahony.q3;             

        g_attitude.roll = atan2f(2.0f * (mahony.q0 * mahony.q1 + mahony.q2 * mahony.q3), q0q0 - q1q1 - q2q2 + q3q3) * RAD_TO_DEG;
        sinP      = std::clamp(2.0f * (mahony.q0 * mahony.q2 - mahony.q1 * mahony.q3), -1.0f, 1.0f);
        g_attitude.pitch= asinf(sinP) * RAD_TO_DEG;    
        actual_compass_heading   = atan2f(2.0f * (mahony.q1 * mahony.q2 + mahony.q0 * mahony.q3), q0q0 + q1q1 - q2q2 - q3q3);

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

        if(!g_sys.is_armed) [[unlikely]]{                
            // 시동 안 걸렸을 때는 모터 정지 및 PID 적분항 초기화
            motor.stop_all_motors();
            // 시동을 켜는 순간 '튀는' 현상을 방지합니다.
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
      
            if (loop_cnt % 20 == 2){ //20HZ
                float temp_alt = 0.0f,temp_rate = 0.0f;
                std::tie(ret_code,temp_alt,temp_rate)   = bmp388_main.Managed_get_relative_altitude();       
                if (ret_code == ESP_OK){
                    g_baro.filtered_altitude = temp_alt;
                    filtered_alt        = temp_alt;                    
                    filtered_climb_rate = temp_rate;
                }

                { // 비정상적인 요인 제한
                    if (filtered_alt > 500.0f) filtered_alt = 0.0f;                     // 비정상적인 고도 차단
                    if (filtered_alt <= 0.0f) filtered_alt = 0.0f;                      // 음수 고도 방지
                    if (fabsf(filtered_climb_rate) > 10.0f) filtered_climb_rate = 0.0f; // 비정상적 상승률 방지
                }

                if (g_sys.error_hold_mode) {
                    filtered_alt = g_baro.filtered_altitude;        // 에러 모드에서는 => 마지막 데이터로 안정된 고도 유지
                    filtered_climb_rate = 0.0f;                     // 상승률은 0으로 고정
                }else if(g_sys.manual_hold_mode) {
                    // Outer Loop: 고도 유지 (P 제어 위주)
                    float target_climb_rate = pid.run_pid_angle(&pid.pid_alt_pos, target_alt, filtered_alt, 0.025f, false);
                    target_climb_rate       = std::clamp(target_climb_rate, -1.5f, 1.5f);

                    // Inner Loop: 수직 속도 유지 (PI 제어 위주)
                    alt_throttle_offset = pid.run_pid_rate(&pid.pid_alt_rate, target_climb_rate, filtered_climb_rate, 0.025f);
                    alt_throttle_offset = std::clamp(alt_throttle_offset, -150.0f, 150.0f);
                } else { // 정상모드 (?)
                    filtered_alt = 0.0f;
                    filtered_climb_rate = 0.0f;
                    alt_throttle_offset = 0.0f;

                    pid.reset_pid(&pid.pid_alt_pos);
                    pid.reset_pid(&pid.pid_alt_rate);
                }
            }
            
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
            float out_roll  = pid.run_pid_rate(&pid.pid_roll_rate,  target_rate_roll,  calculation_gyro_x, dt);
            float out_pitch = pid.run_pid_rate(&pid.pid_pitch_rate, target_rate_pitch, calculation_gyro_y, dt);
            float out_yaw   = pid.run_pid_rate(&pid.pid_yaw_rate,   target_rate_yaw,   calculation_gyro_z, dt);

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
            float motor_v[4];
            motor_v[0] = std::clamp(base_pwm - out_pitch - out_roll - out_yaw, 1050.0f, 2000.0f);
            motor_v[1] = std::clamp(base_pwm - out_pitch + out_roll + out_yaw, 1050.0f, 2000.0f);
            motor_v[2] = std::clamp(base_pwm + out_pitch - out_roll + out_yaw, 1050.0f, 2000.0f);
            motor_v[3] = std::clamp(base_pwm + out_pitch + out_roll - out_yaw, 1050.0f, 2000.0f);
            motor.update_compare_value({motor_v[0],motor_v[1],motor_v[2],motor_v[3]});
        }           
        // loop check 
        flight->loop_check();
        flight->total_us = esp_timer_get_time() - last_time;
        int64_t current_time;
        while ((current_time = esp_timer_get_time()) - last_time < INTERVAL_US) {
            if (INTERVAL_US - (current_time - last_time) > 1200) {
                vTaskDelay(1); 
            }
        }
    }
}


// ========== 비행 제어 태스크 생성  ==========
BaseType_t Flight::start_task()
{
    auto& espnow        = Service::EspNow::get_instance();
    auto& battery       = Driver::Battery::get_instance();
    //auto& i2c           = Driver::I2C::get_instance();
    auto& icm20948_main = Sensor::ICM20948::Main();
    auto& icm20948_sub  = Sensor::ICM20948::Sub();
    auto& ist8310       = Sensor::IST8310::get_instance();
    auto& ak09916       = Sensor::AK09916::get_instance();
    auto& bmp388_main   = Sensor::BMP388::Main();
    auto& bmp388_sub    = Sensor::BMP388::Sub();
    auto& mahony        = Service::Mahony::get_instance();
    auto& motor         = Driver::Motor::get_instance();
    auto& gps           = Sensor::Gps::get_instance();
    auto& flysky        = Service::Flysky::get_instance();
    auto& buzzer        = Driver::Buzzer::get_instance();
    //auto& mavlink       = Service::Mavlink::get_instance();
    auto& telemetry     = Service::Telemetry::get_instance();
    //auto& pid           = Controller::PID::get_instance();
    auto& failsafe      = Service::FailSafe::get_instance();
    auto& timer         = Service::Timer::get_instance();
    
    esp_err_t ret;
    auto mac_addr = espnow.get_my_mac_address();
    ESP_LOGI(TAG, "My MAC address: %02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0], mac_addr[1], mac_addr[2],mac_addr[3], mac_addr[4], mac_addr[5]);
    
    // 각각의 오프셋을 구한다.
    icm20948_main.calibrate();  
    vTaskDelay(pdMS_TO_TICKS(50));
    icm20948_sub.calibrate();		

    auto [ret_bmp0,mgp] = bmp388_main.calibrate_ground_pressure();
    vTaskDelay(pdMS_TO_TICKS(50));
    auto [ret_bmp1,sgp] = bmp388_sub.calibrate_ground_pressure();
    g_baro.ground_pressure = (mgp+sgp) * 0.5;


    {// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (시작)==========	        
		auto [ret,acc,gyro]     = icm20948_main.read_with_offset();
		acc[1]    *=  -1.0f;  // 오른손 법칙에 적용 2가지 모두 (-)부호를 해야한다 (여기는 gyro는 사용하지 않지만 알아두라는 알림의 표시로...)
        gyro[0]   *=  -1.0f;

        // 지자계 데이터를 읽는다. 		
        auto [ist_ret,ist_mag]  = ist8310.read_with_offset();
		auto [ ak_ret, ak_mag]  = ak09916.read_with_offset();    

        auto  magx = (ist_mag[0]+ak_mag[0])*0.5;
        auto  magy = (ist_mag[1]+ak_mag[1])*0.5;
        auto  magz = (ist_mag[2]+ak_mag[2])*0.5;

		// 융합된 데이터를 적용처리.
        mahony.calibrate_mahony_initial_attitude(acc[0],acc[1], acc[2],magx,magy,magz);
		ESP_LOGI(TAG, "✓ Mahony attitude initialization completeed");
		// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (끝)==========
	}


    // ========== [3단계] 센서 연결 상태 검증 (critical check) ==========
    if (
        icm20948_main.get_dev_handle()  == nullptr ||
        icm20948_sub.get_dev_handle()   == nullptr ||
        ist8310.get_dev_handle()        == nullptr ||
        ak09916.get_dev_handle()        == nullptr ||
        bmp388_main.get_dev_handle()    == nullptr ||
        bmp388_sub.get_dev_handle()     == nullptr) 
        {
        if (icm20948_main.get_dev_handle() == nullptr) 
            ESP_LOGE(TAG, "%s MAIN에 연결할 수 없습니다!",   "ICM20948");
        if (icm20948_sub.get_dev_handle() == nullptr) 
            ESP_LOGE(TAG, "%s SUB에 연결할 수 없습니다!",    "ICM20948");
        if( ist8310.get_dev_handle() == nullptr) 
            ESP_LOGE(TAG, "%s MAIN에 연결할 수 없습니다!",   "IST8310");
        if( ak09916.get_dev_handle()  == nullptr) 
            ESP_LOGE(TAG, "%s SUB에 연결할 수 없습니다!",    "AK09916");
        if (bmp388_main.get_dev_handle()   == nullptr) 
            ESP_LOGE(TAG, "%s MAIN에 연결할 수 없습니다!",   "BMP388");
        if (bmp388_sub.get_dev_handle()    == nullptr) 
            ESP_LOGE(TAG, "%s SUB에 연결할 수 없습니다!",    "BMP388");

        ESP_LOGE(TAG, "필수 센서 미연결! 시스템 중단");

        motor.stop_all_motors();    

        // 무한 대기 (리부팅 필요)
        while (true) {
            static bool blink_led = false;
            gpio_set_level(GPIO_NUM_2,(blink_led = !blink_led));
            vTaskDelay(pdMS_TO_TICKS(100));
            buzzer.sound_error();
        }
    }

    // CORE 0
    BaseType_t res = pdPASS;
    res = espnow.start_task();        // 1. espnow tx task
    if (res != pdPASS) return res;
    res = flysky.start_task();        // 2. flysky task
    if (res != pdPASS) return res;
    res = gps.start_task();           // 3. gps task
    if (res != pdPASS) return res;
    res = telemetry.start_task();     // 4. telemetry task
    if (res != pdPASS) return res;
    res = battery.start_task();       // 5. battery check task
    if (res != pdPASS) return res;
    res = failsafe.start_task();      // 6. failsafe task
    if (res != pdPASS) return res;
    
    // CORE 1
    // 7. flight task
    res = xTaskCreatePinnedToCore(
                        flight_task, 
                        "flight", 
                        8192,
                        this, 
                        24, 
                        &_task_handle, 
                        1);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "6.Flight Task is Failed! code: %d", res);
        // 모터 안전 정지
        motor.stop_all_motors();
        return res;
    } else {
        ESP_LOGI(TAG, "✓ 6.Flight Task is passed...");
    }
    
    ESP_LOGI(TAG, "✅ All Processes is passed... Flight ready!");

    // 콜백이 등록되어야지 데이터가 들어온다.
    ret = espnow.connect_callback();
    if (ret != ESP_OK){
        res = pdFAIL;
    }
    // 10hz,1hz...일정하게 qgc로 보내는 mavlink message처리를 한다.
    ret = timer.Start();
    if (ret != ESP_OK){
        res = pdFAIL;
    }
//   buzzer.sound_success();
    return res;
}

void Flight::loop_check()
{
    // static uint16_t ii =  0;
    // ii++;
    // if(ii > 1000){
    //     ESP_LOGI(TAG,"calculated_dt : %10.4f ",calculated_dt);
    //     ii =0;
    // }
}


} // namespace FLIGHT
