#include "ryu_flight_task.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ryu_flysky.h"
#include "ryu_MahonyFilter.h"
#include "ryu_pid.h"
#include "ryu_espnow.h"
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
#include "ryu_magsensor.h"
#include "ryu_icm20948.h"
#include "ryu_mavlink.h"
#include "ryu_businterface.h"
#include "ryu_KalmanFilter.h"

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

    auto& kalman = Controller::KalmanFilter::get_instance();
    kalman.reset(); // 공분산 및 상태 초기화

    auto& buzzer        = Driver::Buzzer::get_instance();
        err = buzzer.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Buzzer Module Initialize Failed.");
            return err;
        }        
        vTaskDelay(pdMS_TO_TICKS(10));
        //buzzer.sound_system_start();

    // 이 부분에서 SPI Interface 를 할용하면 SPI로 처리된다
    
    auto& i2c           = Driver::I2C::get_instance();
    err = i2c.initialize();
    if (err != ESP_OK){
        ESP_LOGI(TAG, "I2C Module Initialize Failed.");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    auto bus_handle = i2c.get_bus_handle();

    { //ICM20948의 main 초기화
        // 1. Main IMU 설정 (주소 전달 -> 내부에서 장치추가/Bus객체생성/set_bus/init까지 한방에)
        err = Sensor::ICM20948::Main().setup_i2c_interface(bus_handle, Sensor::ICM20948::ADDR_VCC);
        if (err != ESP_OK){
            ESP_LOGI(TAG, "ICM20948 Main Module Setup Failed.");
            return err;
        }
        err = Sensor::ICM20948::Main().initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "ICM20948 Main Module Initialize Failed.");
            return err;
        }
        err = Sensor::ICM20948::Main().enable_mag_bypass();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "ICM20948 Main Module enable_mag_bypass Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    {// 2. Sub IMU 설정
        err = Sensor::ICM20948::Sub().setup_i2c_interface(bus_handle, Sensor::ICM20948::ADDR_GND);
        if (err != ESP_OK){
            ESP_LOGI(TAG, "ICM20948 Sub Module Setup Failed.");
            return err;
        }
        err = Sensor::ICM20948::Sub().initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "ICM20948 Sub Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    { //AK09916 INITIALIZE
        err = Sensor::AK09916::get_instance().setup_i2c_interface(bus_handle,Sensor::AK09916::ADDR);
        if (err != ESP_OK){
            ESP_LOGI(TAG, "AK09916 Module Setup Failed.");
            return err;
        }
        err = Sensor::AK09916::get_instance().initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "AK09916 Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    {
        err = Sensor::IST8310::get_instance().setup_i2c_interface(bus_handle,Sensor::IST8310::ADDR);
        if (err != ESP_OK){
            ESP_LOGI(TAG, "IST8310 Module Setup Failed.");
            return err;
        }
        err = Sensor::IST8310::get_instance().initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "IST8310 Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    {    
        Sensor::BMP388::Main().setup_i2c_interface(bus_handle,Sensor::BMP388::ADDR_VCC);
        if (err != ESP_OK){
            ESP_LOGI(TAG, "BMP338 Main Module setup Failed.");
            return err;
        }
        Sensor::BMP388::Main().initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "BMP338 Main Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    {
        Sensor::BMP388::Sub().setup_i2c_interface(bus_handle,Sensor::BMP388::ADDR_GND);
        if (err != ESP_OK){
            ESP_LOGI(TAG, "BMP338 Sub Module Setup Failed.");
            return err;
        }
        Sensor::BMP388::Sub().initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "BMP338 Sub Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    auto& espnow        = Service::EspNow::get_instance();
        err = espnow.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Espnow Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& battery       = Driver::Battery::get_instance();
        err = battery.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Baterry Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& mahony        = Service::Mahony::get_instance();
        err = mahony.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Mahony Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& motor         = Driver::Motor::get_instance();
        err = motor.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Motor Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& gps           = Sensor::Gps::get_instance();
        err = gps.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Gps Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& flysky        = Service::Flysky::get_instance();
        err = flysky.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Flysky Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& mavlink       = Service::Mavlink::get_instance();
        err = mavlink.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Mavlink Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& telemetry     = Service::Telemetry::get_instance();
        err = telemetry.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Telemetry Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    auto& pid           = Controller::PID::get_instance();
        err = pid.initialize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Pid Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    // main.cpp 로 넘김..... 제일먼저 실행되어져 감시자 역활을 해야한다.
    // auto& failsafe      = Service::FailSafe::get_instance();
    //     err = failsafe.initialize();
    //     if (err != ESP_OK){
    //         ESP_LOGI(TAG, "FailSafe Module Initialize Failed.");
    //         return err;
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(10));
    auto& timer         = Service::Timer::get_instance();
        err = timer.intiallize();
        if (err != ESP_OK){
            ESP_LOGI(TAG, "Timer Module Initialize Failed.");
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  

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
    Flight& flight = Flight::get_instance(); 

    auto& motor         = Driver::Motor::get_instance();
    auto& icm20948_main = Sensor::ICM20948::Main();
    auto& bmp388_main   = Sensor::BMP388::Main();
    auto& pid           = Controller::PID::get_instance();
    auto& mahony        = Service::Mahony::get_instance();
    auto& flysky        = Service::Flysky::get_instance();
    auto& managed_mag  = Sensor::ManageMag::get_instance();
    if(!managed_mag.is_initialized())
        managed_mag.initialize();

    auto& kalman = Controller::KalmanFilter::get_instance();

    uint32_t loop_cnt = 0;
    int64_t  last_time = esp_timer_get_time();

    //Watch Dog 등록.  
    esp_task_wdt_add(flight._task_handle);     
     
    while(true) {
        int64_t now = esp_timer_get_time();
        flight.calculated_dt = (now- last_time);
        last_time = now; 
        if (++loop_cnt >= 400) loop_cnt = 0; // 1초 주기로 초기화
 
        //Watch Dog에게 "나 살아 있어!"" 라고 알린다.  
        esp_task_wdt_reset(); 
        
        // 연속적인 데이터 읽기 실패를 체크한다. 
        static float    calculation_acc_x  = 0.0f,  
                        calculation_acc_y  = 0.0f,  
                        calculation_acc_z  = 0.0f;
        static float    calculation_gyro_x = 0.0f,  
                        calculation_gyro_y = 0.0f,  
                        calculation_gyro_z = 0.0f;

        float macc[3]={},mgyro[3]={};
        esp_err_t ret_code = icm20948_main.Managed_read_with_offset( macc, mgyro,sizeof(macc));
        if (ret_code == ESP_OK){
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

        if (managed_mag.get_bus_type() == Interface::BusType::SPI){
            auto mag = icm20948_main.get_mag();
            calulation_mag_x=mag[0]; 
            calulation_mag_y=mag[1]; 
            calulation_mag_z=mag[2];
        }else{ //Interface::BusType::I2C
            if (loop_cnt % 8 == 0){ // 50HZ
                auto [ret_mag, mag] = managed_mag.Managed_read_with_offset();
                if(ret_mag == ESP_OK){
                    calulation_mag_x=mag[0]; 
                    calulation_mag_y=mag[1]; 
                    calulation_mag_z=mag[2];
                } 
                //ESP_LOGI(TAG,"mx:%f , my:%f , mz:%f",calulation_mag_x,calulation_mag_y,calulation_mag_z);
                ret_code = ret_mag;
            }
        }

        kalman.update(
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
              
        // mahony.MahonyAHRSupdate(   
        //                     calculation_gyro_x * DEG_TO_RAD,
        //                     calculation_gyro_y * DEG_TO_RAD, 
        //                     calculation_gyro_z * DEG_TO_RAD, 
        //                     calculation_acc_x, 
        //                     calculation_acc_y, 
        //                     calculation_acc_z, 
        //                     calulation_mag_x,
        //                     calulation_mag_y,
        //                     calulation_mag_z,
        //                     dt
        //                 );
        
        attitude_data_t m_attitude ={};               
        sys_t m_sys = g_sys;

        { // qgc로 보내는 데이터
            m_attitude.rollspeed    = calculation_gyro_x ;
            m_attitude.pitchspeed   = calculation_gyro_y ;
            m_attitude.yawspeed     = calculation_gyro_z ;
        }
 
        float roll_deg, pitch_deg, yaw_deg;
        //mahony.get_euler(&roll_deg,&pitch_deg,&yaw_deg);
        kalman.get_euler(&roll_deg,&pitch_deg,&yaw_deg);
        
        m_attitude.roll  = roll_deg;
        m_attitude.pitch = pitch_deg;

        float actual_compass_heading = yaw_deg * DEG_TO_RAD;
        // 2. 편각 보정 (-7.7도 적용) 하여 '진북' 기준으로 업데이트
        // 진북에서 -7.7도정도에 자북이 존재하므로 현재 자북을 구한상태에 +7.7도를 더해야만 진북이된다.
        float declinationAngle = 7.7f * DEG_TO_RAD;
        actual_compass_heading += declinationAngle;

        // 3. 각도 범위 정규화 (-PI ~ +PI) -> PID 제어에 유리함
        // 3. 각도 범위 정규화 (-PI ~ +PI)
        if (actual_compass_heading >  M_PI)         
            actual_compass_heading -= 2.0f * M_PI;
        else if (actual_compass_heading < -M_PI)    
            actual_compass_heading += 2.0f * M_PI;

        // 4. 이 yaw_rad를 기반으로 최종 yaw(degree)와 heading_deg 생성
        m_attitude.yaw = actual_compass_heading * RAD_TO_DEG; // 이제 이 yaw는 '진북' 기준입니다.

        // 5. QGC 나침반용 (0 ~ 360도)
        float heading_deg = m_attitude.yaw;
        while (heading_deg < 0)    heading_deg += 360.0f;
        while (heading_deg >= 360) heading_deg -= 360.0f;

        m_attitude.heading = heading_deg;

        //m_attitide에저장되어진 정보를 g_attitude에 넘긴다.
        portENTER_CRITICAL(&g_attitude_mux);
        g_attitude = m_attitude;
        portEXIT_CRITICAL(&g_attitude_mux);

        if(m_sys.is_armed) [[unlikely]]{                
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

            Service::rc_data_t flysky_rc, qgc_rc, final_rc;
            Service::Flysky::get_instance().get_latest_rc(&flysky_rc);
            Service::Mavlink::get_instance().get_qgc_rc(&qgc_rc);
  
            // 조종기가 켜져 있으면(스로틀이 1000 이상이면) 조종기 우선, 아니면 QGC
            if (flysky_rc.type == Service::RC_FLYSKY && flysky_rc.throttle > 5.0f) {
                final_rc = flysky_rc;
            } else if (qgc_rc.type == Service::RC_QGC && qgc_rc.throttle > 5.0f) {
                final_rc = qgc_rc;
            }
            //                                          민감도    
            float tg_roll     = final_rc.roll     ;  //* 0.3f;
            float tg_pitch    = final_rc.pitch    ;  //* 0.3f;
            float tg_yaw_rate = final_rc.yaw      ;  //* 1.5f;
            float tg_throttle = final_rc.throttle * 10.0f;

            // 1초에 한번 파라미터 테이블에서 최신 PID 계수를 읽어옵니다
            //if(loop_cnt==100) sync_pid_from_params();
            
            // 수직속도 (Inner Loop: 수직 속도 유지 (PI 제어 위주))
            static float alt_throttle_offset = 0.0f;         

            if (loop_cnt % 20 == 2){ //20HZ
                static float target_alt = 0.0f;
                static bool last_alt_hold_state = false;

                // bmp388에서 읽어오는 변수 (현재 고도와 상승률)
                static float    filtered_alt =0.0f, 
                                filtered_climb_rate=0.0f;
                
                float temp_alt =0.0f,temp_rate =0.0f;
                auto err = bmp388_main.Managed_get_relative_altitude(&temp_alt,&temp_rate);
                if (err == ESP_OK){
                    filtered_alt        = temp_alt;                    
                    filtered_climb_rate = temp_rate;
                }

                // 3. 고도 유지 모드 스위치 처리                               
                if (m_sys.manual_hold_mode){
                    if(!last_alt_hold_state){
                        target_alt = filtered_alt;  // 모드가 켜지는 순간의 고도를 목표로 고정
                        alt_throttle_offset = 0.0f; // PID 보정값 초기화
                        filtered_alt = 0.0f;        // 고도 초기화
                        filtered_climb_rate = 0.0f; // 상승률 초기화
                        last_alt_hold_state = true;
                    }
                }                 
                last_alt_hold_state = m_sys.manual_hold_mode ;

                { // 비정상적인 요인 제한
                    if (filtered_alt > 500.0f) filtered_alt = 0.0f;                     // 비정상적인 고도 차단
                    if (filtered_alt <= 0.0f) filtered_alt = 0.0f;                      // 음수 고도 방지
                    if (fabsf(filtered_climb_rate) > 10.0f) filtered_climb_rate = 0.0f; // 비정상적 상승률 방지
                }

                // 에러발생으로 인한 hold mode
                if (m_sys.error_hold_mode) {
                    filtered_climb_rate = 0.0f;                     // 상승률은 0으로 고정
                }

                // 사용자 지정 hold mode 
                if(m_sys.manual_hold_mode) {
                    // Outer Loop: 고도 유지 (P 제어 위주)
                    float target_climb_rate = pid.run_pid_angle(&pid.pid_alt_pos, target_alt, filtered_alt, 0.025f, false);
                    target_climb_rate       = std::clamp(target_climb_rate, -1.5f, 1.5f);

                    // Inner Loop: 수직 속도 유지 (PI 제어 위주)
                    alt_throttle_offset = pid.run_pid_rate(&pid.pid_alt_rate, target_climb_rate, filtered_climb_rate, 0.025f);
                    alt_throttle_offset = std::clamp(alt_throttle_offset, -150.0f, 150.0f);
                }

                // 정상모드
                if(!m_sys.error_hold_mode && !m_sys.manual_hold_mode ){
                    filtered_alt = 0.0f;
                    filtered_climb_rate = 0.0f;
                    alt_throttle_offset = 0.0f;

                    pid.reset_pid(&pid.pid_alt_pos);
                    pid.reset_pid(&pid.pid_alt_rate);
                }
            }
            

            //정지 상태에서 출력값이 누적되는 문제를 해결하기 위해,
            // 이 코드에 I-term만 초기화하는 기능을 추가하고 적용하는 방법을 제안해 드립니다.
            if (tg_throttle < 10.0f) { // 스로틀이 매우 낮을 때 (바닥에 있을 때)
                pid.reset_pid_iterm(&pid.pid_roll_angle);
                pid.reset_pid_iterm(&pid.pid_pitch_angle);
                pid.reset_pid_iterm(&pid.pid_yaw_angle);
                
                pid.reset_pid_iterm(&pid.pid_roll_rate);
                pid.reset_pid_iterm(&pid.pid_pitch_rate);
                pid.reset_pid_iterm(&pid.pid_yaw_rate);
            }

            // --- [1단계: Outer Loop - 각도 제어] ---
            // 조종기 스틱(tg_roll) -> 목표 각도 -> 목표 각속도(deg/s) 출력
            float target_rate_roll  = pid.run_pid_angle(&pid.pid_roll_angle,  tg_roll,  m_attitude.roll,  dt, false);
            float target_rate_pitch = pid.run_pid_angle(&pid.pid_pitch_angle, tg_pitch, m_attitude.pitch, dt, false);
            //float target_rate_yaw = pid.run_pid_angle(&pid.pid_yaw_angle,   tg_yaw_rate, m_attitude.yaw, dt, true);

            // Yaw는 사용자의 스틱 입력(tg_yaw_rate)을 목표 각속도로 직접 사용하거나, 
            // 현재처럼 Heading Hold를 원하시면 아래처럼 목표 각도를 유지하게 합니다.
            // static float target_yaw_angle = 0.0f;
            // target_yaw_angle += tg_yaw_rate * dt; 
            // if (target_yaw_angle > 180.0f) target_yaw_angle -= 360.0f;
            // if (target_yaw_angle < -180.0f) target_yaw_angle += 360.0f;
            // float target_rate_yaw = pid.run_pid_angle(&pid.pid_yaw_angle, target_yaw_angle, m_attitude.yaw, dt, true);

            // --- [2단계: Inner Loop - 각속도 제어] ---
            // 목표 각속도 -> 현재 자이로 값(g_imu.gyro)과 비교 -> 최종 모터 출력(PWM 변위)
            // g_imu.gyro[0]: Roll속도, [1]: Pitch속도, [2]: Yaw속도
            

            // Yaw 제어 방식 변경: 조종기 스틱 값을 '목표 각도'가 아니라 '목표 각속도'로 직접 사용해 보세요.
            // Outer Loop를 건너뛰고 스틱 값을 바로 Inner Loop의 목표로 전달(throttle이 )
            float target_rate_yaw  = tg_yaw_rate; 
            float out_roll  = pid.run_pid_rate(&pid.pid_roll_rate,  target_rate_roll,  calculation_gyro_x, dt);
            float out_pitch = pid.run_pid_rate(&pid.pid_pitch_rate, target_rate_pitch, calculation_gyro_y, dt);
            float out_yaw   = pid.run_pid_rate(&pid.pid_yaw_rate,   target_rate_yaw,   calculation_gyro_z, dt);

//if (loop_cnt % 16 == 0) ESP_LOGI(TAG, "out_roll: %8.3f out_pitch: %8.3f out_yaw: %8.3f", out_roll,out_pitch,out_yaw);


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
            float base_pwm = 1000.0f + std::max(tg_throttle + alt_throttle_offset, 50.0f);

            // 1. 우선 클램프 없이 믹싱 계산 (임시 변수)
            float m0 = base_pwm + out_roll - out_pitch + out_yaw;
            float m1 = base_pwm - out_roll - out_pitch - out_yaw;
            float m2 = base_pwm + out_roll + out_pitch - out_yaw;
            float m3 = base_pwm - out_roll + out_pitch + out_yaw;

            // 2. 가장 많이 튀어나온(최대값) 모터 찾기
            float max_motor = m0;
            if (m1 > max_motor) max_motor = m1;
            if (m2 > max_motor) max_motor = m2;
            if (m3 > max_motor) max_motor = m3;

            // 3. 만약 최대값이 2000을 넘는다면, 넘는 만큼 모든 모터에서 공통으로 차감
            if (max_motor > 2000.0f) {
                float diff = max_motor - 2000.0f;
                m0 -= diff;
                m1 -= diff;
                m2 -= diff;
                m3 -= diff;
            }

            //m0 : FL (CW) ,m1 : FR(CCW), m3 : RL (CCW) ,m4 : RR (CW) 
            //float motor_v[4];
            m0 = std::clamp(m0, 1050.0f, 2000.0f);
            m1 = std::clamp(m1, 1050.0f, 2000.0f);
            m2 = std::clamp(m2, 1050.0f, 2000.0f);
            m3 = std::clamp(m3, 1050.0f, 2000.0f);
// 변수 변화확인용
// if (loop_cnt % 16 == 0) ESP_LOGI(TAG, "tg_throttle : %8.3f  m1: %8.3f m2: %8.3f m3: %8.3f m4: %8.3f",
//                                                 tg_throttle,
//                                                 m0,
//                                                 m1,
//                                                 m2,
//                                                 m3);
            motor.update_compare_value({m0,m1,m2,m3});
        }           

        // loop check 
        flight.loop_check();
        flight.total_us = esp_timer_get_time() - last_time;
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
    auto& icm20948_sub = Sensor::ICM20948::Sub();    
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
    //auto& failsafe      = Service::FailSafe::get_instance();
    auto& timer         = Service::Timer::get_instance();
    
    esp_err_t ret;
    auto mac_addr = espnow.get_my_mac_address();
    ESP_LOGI(TAG, "My MAC address: %02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0], mac_addr[1], mac_addr[2],mac_addr[3], mac_addr[4], mac_addr[5]);
    
    // // 각각의 오프셋을 구한다.
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

    bool is_all_ok = true;
    // // ========== [3단계] 센서 연결 상태 검증 (critical check) ==========
    if (!icm20948_main.is_initialized()){ ESP_LOGE(TAG, "ICM20948 MAIN 연결 실패!");is_all_ok = false; }
    if (!icm20948_sub.is_initialized()) { ESP_LOGE(TAG, "ICM20948 SUB 연결 실패!"); is_all_ok = false; }
    if (!ist8310.is_initialized())       { ESP_LOGE(TAG, "IST8310 MAIN 연결 실패!"); is_all_ok = false; }
    if (!ak09916.is_initialized())       { ESP_LOGE(TAG, "AK09916 SUB 연결 실패!");  is_all_ok = false; }
    if (!bmp388_main.is_initialized())   { ESP_LOGE(TAG, "IST8310 MAIN 연결 실패!"); is_all_ok = false; }
    if (!bmp388_sub.is_initialized())    { ESP_LOGE(TAG, "IST8310 MAIN 연결 실패!"); is_all_ok = false; }
    if (!is_all_ok) {
        ESP_LOGE(TAG, "필수 센서 미연결! 시스템 중단");
        motor.stop_all_motors();
        // 무한 루프 진입
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
    // res = failsafe.start_task();      // 6. failsafe task
    // if (res != pdPASS) return res;
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
