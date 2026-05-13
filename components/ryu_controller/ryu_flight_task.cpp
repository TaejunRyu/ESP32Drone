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

    auto& kalman = Filter::KalmanFilter::get_instance();
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
        err = Sensor::ICM20948::Main().init_bus(
                        Interface::createBIF(bus_handle, Sensor::ICM20948::ADDR_VCC));
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
        err = Sensor::ICM20948::Sub().init_bus(
                        Interface::createBIF(bus_handle, Sensor::ICM20948::ADDR_GND));
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
        err = Sensor::AK09916::get_instance().init_bus(
                        Interface::createBIF(bus_handle,Sensor::AK09916::ADDR));
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
        err = Sensor::IST8310::get_instance().init_bus(
                    Interface::createBIF(bus_handle,Sensor::IST8310::ADDR));
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
        Sensor::BMP388::Main().init_bus(
                Interface::createBIF(bus_handle,Sensor::BMP388::ADDR_VCC));
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
        Sensor::BMP388::Sub().init_bus(
                Interface::createBIF(bus_handle,Sensor::BMP388::ADDR_GND));
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
    auto& mahony        = Filter::Mahony::get_instance();
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
    auto& mahony        = Filter::Mahony::get_instance();
    //auto& flysky        = Service::Flysky::get_instance();
    auto& managed_mag  = Sensor::ManageMag::get_instance();
    if(!managed_mag.is_initialized())
        managed_mag.initialize();

    auto& kalman = Filter::KalmanFilter::get_instance();

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

        static ENV::sensor_data_t cur_acc{0.0f,};
        static ENV::sensor_data_t cur_gyro{0.0f,};
        static ENV::sensor_data_t cur_mag{0.0f,};
        
        
        float acc[3]={}, gyro[3]={},mag[3]={};
        esp_err_t ret_code = icm20948_main.Managed_read_with_offset( acc, gyro,sizeof(acc));
        if (ret_code == ESP_OK){
            for(size_t ii = 0 ; ii < 3 ; ++ii){ 
                cur_acc.data[ii] = acc[ii];
                cur_gyro.data[ii] = gyro[ii];
            }
            // memcpy(cur_acc.data,acc,sizeof(float)*3);
            // memcpy(cur_gyro.data,gyro,sizeof(float)*3);
        }


        if (loop_cnt % 8 == 0){// 50HZ
            if (managed_mag.get_bus_type() == Interface::BusType::SPI){ 
                icm20948_main.get_mag(mag);
                for(size_t ii = 0 ; ii < 3 ; ++ii){ 
                    cur_mag.data[ii] = mag[ii];
                }

                // memcpy(cur_mag.data,mag,sizeof(float)*3);

            } else if (managed_mag.get_bus_type() == Interface::BusType::I2C){ 
                auto [ret_mag, mag] = managed_mag.Managed_read_with_offset();
                if(ret_mag == ESP_OK){
                    for(size_t ii = 0 ; ii < 3 ; ++ii){ 
                        cur_mag.data[ii] = mag[ii];
                    }

                    // memcpy(cur_mag.data,mag.data(),sizeof(float)*3);
                } 
                ret_code = ret_mag;
            }
        }

        kalman.update(
                        cur_gyro.x * ENV::DEG_TO_RAD,
                        cur_gyro.y * ENV::DEG_TO_RAD,
                        cur_gyro.z * ENV::DEG_TO_RAD,
                        cur_acc.x,
                        cur_acc.y,
                        cur_acc.z,
                        cur_mag.x,
                        cur_mag.y,
                        cur_mag.z,
                        dt
                        );
              
        // mahony.MahonyAHRSupdate(   
        //                 cur_gyro.x * ENV::DEG_TO_RAD,
        //                 cur_gyro.y * ENV::DEG_TO_RAD,
        //                 cur_gyro.z * ENV::DEG_TO_RAD,
        //                 cur_acc.x,
        //                 cur_acc.y,
        //                 cur_acc.z,
        //                 cur_mag.x,
        //                 cur_mag.y,
        //                 cur_mag.z,
        //                 dt
        //                 );
        
        ENV::euler_data_t cur_euler_deg {0.0f,};

        float t_roll,t_pitch,t_yaw;
        kalman.get_euler(&t_roll,&t_pitch,&t_yaw);

        cur_euler_deg.roll  = t_roll;
        cur_euler_deg.pitch = t_pitch;
        cur_euler_deg.yaw   = t_yaw;
        
//if (loop_cnt % 16 == 0) ESP_LOGI(TAG, "roll_deg: %8.3f pitch_deg: %8.3f yaw_deg: %8.3f", cur_euler_deg.roll,cur_euler_deg.pitch,cur_euler_deg.yaw);
        
        constexpr float TARGET_TRUE_NORTH = 7.7f; 
        // 진북에서 -7.7도정도에 자북이 존재하므로 현재 자북을 구한상태에 +7.7도를 더해야만 진북이된다.
        cur_euler_deg.yaw = cur_euler_deg.yaw + TARGET_TRUE_NORTH;
        if (cur_euler_deg.yaw < 0.0f)           cur_euler_deg.yaw += 360.0f;
        else if (cur_euler_deg.yaw >= 360.0f)   cur_euler_deg.yaw -= 360.0f;
    
        ENV::attitude_data_t m_attitude ={};               

        // kalman의 x[4],x[5],x[6]성분을 가저와 현재 gyro데이터에서 제거한후 pid에 적용.
        cur_gyro.x = cur_gyro.x - kalman.q4 * ENV::RAD_TO_DEG;
        cur_gyro.y = cur_gyro.y - kalman.q5 * ENV::RAD_TO_DEG;
        cur_gyro.z = cur_gyro.z - kalman.q6 * ENV::RAD_TO_DEG;

// if (loop_cnt % 16 == 0) 
//         ESP_LOGI(TAG, "roll_speed_err : %8.5f pitch_speed_err: %8.5f yaw_speed_err: %8.5f cur_gyro x: %8.5f cur_gyro y: %8.5f cur_gyro z: %8.5f", 
//                         roll_speed_err,
//                         pitch_speed_err,
//                         yaw_speed_err,
//                         cur_gyro.x,
//                         cur_gyro.y,
//                         cur_gyro.z
//                     );

        m_attitude.rollspeed    = cur_gyro.x ;
        m_attitude.pitchspeed   = cur_gyro.y ;
        m_attitude.yawspeed     = cur_gyro.z ;
        m_attitude.roll         = cur_euler_deg.roll;
        m_attitude.pitch        = cur_euler_deg.pitch;        
        m_attitude.yaw          = cur_euler_deg.yaw;


        //m_attitide에 저장되어진 정보를 g_attitude에 넘긴다.
        portENTER_CRITICAL(&ENV::g_attitude_mux);
        ENV::g_attitude = m_attitude;
        portEXIT_CRITICAL(&ENV::g_attitude_mux);
        
        // 일시에 g_sys를 가져온다.
        ENV::sys_t m_sys = ENV::g_sys;

        // 시동이 걸렸을경우
        if(m_sys.is_armed) [[likely]]{                

            // rc데이터가 flysky에서 오는가 아니면 qgc에서 오는가?
            Service::rc_data_t flysky_rc, qgc_rc, final_rc;
            Service::Flysky::get_instance().get_latest_rc(&flysky_rc);
            Service::Mavlink::get_instance().get_qgc_rc(&qgc_rc);
  
            // 조종기가 켜져 있으면(스로틀이 1000 이상이면) 조종기 우선, 아니면 QGC
            if (flysky_rc.type == Service::RC_FLYSKY && flysky_rc.throttle > 5.0f) {
                final_rc = flysky_rc;
            } else if (qgc_rc.type == Service::RC_QGC && qgc_rc.throttle > 5.0f) {
                final_rc = qgc_rc;
            }
            static  float   target_rc_roll_deg{0.0f},
                            target_rc_pitch_deg{0.0f},
                            target_rc_yaw_deg{0.0f},
                            target_rc_throttle{0.0f};
            
            if (esp_timer_get_time() - final_rc.receive_time < 50'000){
                // 조종기 입력값 계산  (실제 조종기에서 들어오는 값들을 scale 작업을 하여 감도를 조절한다.)
                // 감도를 높이려면 값을 키우면 된다.                         
                target_rc_roll_deg        = final_rc.roll     ;  //* 0.3f;
                target_rc_pitch_deg       = final_rc.pitch    ;  //* 0.3f;
                target_rc_yaw_deg         = final_rc.yaw      ;  //* 1.5f;
                target_rc_throttle        = final_rc.throttle * 10.0f;
            }

            // 1초에 한번 파라미터 테이블에서 최신 PID 계수를 읽어옵니다
            //if(loop_cnt==100) sync_pid_from_params();
            
            // 수직속도 (Inner Loop: 수직 속도 유지 (PI 제어 위주))
            static float alt_throttle_offset = 0.0f;         
            static float target_alt{0.0f};
            static float    cur_alt{0.0f}, 
                            cur_climb_rate{0.0f};

            static float estimated_alt = 0.0f;        // 400Hz로 부드럽게 추정되는 현재 고도 (PID 입력용)
            static float estimated_climb_rate = 0.0f; // 400Hz로 부드럽게 추정되는 현재 상승률 (PID 입력용)
                            
            // [수정] 1. 중력가속도가 차감된 순수 수직 가속도 계산 (Z축 Up이 플러스인 시스템 기준)
            // m_attitude.roll과 pitch 각도(라디안)를 이용하여 물리적 틸트 보정을 가합니다.
            float cos_roll = cosf(cur_euler_deg.roll * ENV::DEG_TO_RAD);
            float cos_pitch = cosf(cur_euler_deg.pitch * ENV::DEG_TO_RAD);
            
            // 기체가 기울어지면 센서에 찍히는 중력 성분이 분산되므로 이를 복원하여 1.0G(9.8)를 뺍니다.
            // 만약 cur_acc.z 단위가 G단위라면 1.0f를 빼고, m/s^2 단위라면 9.80665f를 빼야 합니다.
            float pure_vertical_accel = (cur_acc.z / (cos_roll * cos_pitch)) - 1.0f; 
            
            // G단위를 m/s^2 스케일로 변환하여 적분 처리 (BMP388 고도 단위인 미터(m)와 스케일 일치)
            pure_vertical_accel *= 9.80665f; 

            // 가속도를 통한 고도/상승률 초고속 적분 추정 (Drift 방지를 위해 임계치 클램핑)
            if (fabsf(pure_vertical_accel) < 0.2f) pure_vertical_accel = 0.0f; // 진동 노이즈 데드밴드 처리
            estimated_alt += estimated_climb_rate * dt; //+ 0.5f * pure_vertical_accel * dt * dt;
            estimated_climb_rate += pure_vertical_accel * dt;


            if (loop_cnt % 20 == 2){ //20HZ
                // bmp388에서 읽어오는 변수 (현재 고도와 상승률)
 
                float temp_alt{0.0f},temp_rate{0.0f};
                auto err = bmp388_main.Managed_get_relative_altitude(&temp_alt,&temp_rate);
                if (err == ESP_OK){
                    cur_alt        = temp_alt;                    
                    ENV::g_altitude.current =temp_alt;
                    cur_climb_rate = temp_rate;
                    // [안전화] 가속도 적분치와 실제 기압계 데이터 상보필터(Complementary Filter) 융합
                    estimated_alt = (estimated_alt * 0.95f) + (temp_alt * 0.05f);
                    estimated_climb_rate = (estimated_climb_rate * 0.90f) + (temp_rate * 0.10f);
                }
 
                // 비정상적인 요인 제한( 고도제한 )
                static float saved_cur_alt={0.0f};
                if ( cur_alt < 500.0f && cur_alt > 0)
                    saved_cur_alt = cur_alt;
                else 
                    cur_alt = saved_cur_alt;

                // 비정상적인 요인 제한(하강/ 상승속도 제한)
                static float saved_cur_climb_reate={0.0f};
                if (fabsf(cur_climb_rate) <= 10 )
                    saved_cur_climb_reate = cur_climb_rate;
                else
                    cur_climb_rate = saved_cur_climb_reate;

                // 에러발생으로 인한 hold mode
                if (m_sys.hold_mode == ENV::flight_hold_mode::MODE_ERR_HOLD_MODE) {
                    cur_climb_rate = 0.0f;                     // 상승률은 0으로 고정
                }                
            } // loop_cnt %20 ==2 {}


            static ENV::flight_hold_mode last_alt_hold_state{ENV::flight_hold_mode::MODE_NORMAL};
            // 사용자 지정 hold mode 
            if(m_sys.hold_mode != ENV::flight_hold_mode::MODE_NORMAL) {
                // 3. 고도 유지 모드 스위치 처리                               
                if(last_alt_hold_state  != ENV::flight_hold_mode::MODE_USER_HOLD_MODE){
                    target_alt = cur_alt;      // 모드가 켜지는 순간의 고도를 목표로 고정
                    pid.reset_pid(&pid.pid_alt_pos);
                    pid.reset_pid(&pid.pid_alt_rate);
                    last_alt_hold_state = ENV::flight_hold_mode::MODE_USER_HOLD_MODE ;
                }

                // &&&&&&&&&&&&&&&메인 루프 내부 테스트용 코드&&&&&&&&&&&&&&&&(테스트후 삭제)
                static float mock_time = 0.0f;
                mock_time += 0.025f;
                // 고도가 0m -> 0.5m -> 0m -> -0.5m 로 부드럽게 출렁이도록 가짜 데이터 주입
                float cur_alt = target_alt + 0.5f * sinf(mock_time); 
                // &&&&&&77&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

                // Outer Loop: 고도 유지 (P 제어 위주)
                float target_climb_rate = pid.run_pid_angle(&pid.pid_alt_pos, target_alt,cur_alt, 0.05f, false);
                target_climb_rate       = std::clamp(target_climb_rate, -1.5f, 1.5f);

                // Inner Loop: 수직 속도 유지 (PI 제어 위주)
                alt_throttle_offset = pid.run_pid_rate(&pid.pid_alt_rate, target_climb_rate, cur_climb_rate, 0.05f);
                alt_throttle_offset = std::clamp(alt_throttle_offset, -150.0f, 150.0f);

            } else {
                last_alt_hold_state = ENV::flight_hold_mode::MODE_NORMAL ;
                alt_throttle_offset = 0.0f; // 기본 수동 스로틀 사용
                //pid.reset_pid(&pid.pid_alt_pos);
                //pid.reset_pid(&pid.pid_alt_rate);
            }


            //정지 상태에서 출력값이 누적되는 문제를 해결하기 위해,
            // 이 코드에 I-term만 초기화하는 기능을 추가하고 적용하는 방법을 제안해 드립니다.

            bool is_on_ground = target_rc_throttle < 10.0f;

            if (is_on_ground) { // 스로틀이 매우 낮을 때 (바닥에 있을 때)
                pid.reset_pid_iterm(&pid.pid_roll_deg);
                pid.reset_pid_iterm(&pid.pid_pitch_deg);
                pid.reset_pid_iterm(&pid.pid_yaw_deg);
                
                pid.reset_pid_iterm(&pid.pid_roll_rate);
                pid.reset_pid_iterm(&pid.pid_pitch_rate);
                pid.reset_pid_iterm(&pid.pid_yaw_rate);
            }

            // --- [1단계: Outer Loop - 각도 제어] ---
            // 조종기 스틱(target_roll) -> 목표 각도 -> 목표 각속도(deg/s) 출력
            float target_roll_rate  = pid.run_pid_angle(&pid.pid_roll_deg,  target_rc_roll_deg,  cur_euler_deg.roll,  dt, false);
            float target_pitch_rate = pid.run_pid_angle(&pid.pid_pitch_deg, target_rc_pitch_deg, cur_euler_deg.pitch, dt, false);

            // 컨트롤러에 의해서 입력되어지는 값.
            static float target_yaw_deg = 0.0f; // static 또는 전역 변수로 선언
            static bool was_controlling = false; // 직전 루프에서 스틱을 조종 중이었는지 기억
            // 1. 스틱 입력이 있으면 목표 각도를 변화시킴
            // 2. 스틱이력이 없으면 0도를 바라보게 된다.
            if (fabsf(target_rc_yaw_deg) > 1.0f) { // 데드밴드 설정
                 target_yaw_deg += target_rc_yaw_deg * dt;
                 was_controlling = true;
            } else {
                // [스틱 정지] 
                if (was_controlling) {
                    // ★ 중요: 스틱을 딱 놓은 '첫 번째 루프'에서만 현재 실제 각도를 목표각도로 고정!
                    target_yaw_deg = cur_euler_deg.yaw; 
                    was_controlling = false; // 다음 루프부터는 이 if문에 들어오지 않음
                }
                // 이제 스틱을 놓아도 target_yaw_deg가 현재 각도로 고정되므로 그 자리를 유지(Hold)합니다.
            }
            // 2. 각도 범위 정규화 (0~360도 기준인 m_attitude.yaw와 맞춤)
            if (target_yaw_deg >= 360.0f) target_yaw_deg -= 360.0f;
            if (target_yaw_deg < 0.0f)    target_yaw_deg += 360.0f;

            float target_yaw_rate = pid.run_pid_angle(&pid.pid_yaw_deg, target_yaw_deg, cur_euler_deg.yaw, dt,true);

            float out_roll  = pid.run_pid_rate(&pid.pid_roll_rate,  target_roll_rate,  cur_gyro.x, dt);
            float out_pitch = pid.run_pid_rate(&pid.pid_pitch_rate, target_pitch_rate, cur_gyro.y, dt);
            float out_yaw   = pid.run_pid_rate(&pid.pid_yaw_rate,   target_yaw_rate,   cur_gyro.z, dt);

//if (loop_cnt % 16 == 0) ESP_LOGI(TAG, "out_roll: %8.3f out_pitch: %8.3f out_yaw: %8.3f", out_roll,out_pitch,out_yaw);

            // throttle이 거의 0일 때는 yaw 제어를 억제하여
            // 하한 클램프와 충돌하는 현상을 방지한다.
            // 적분/이전 오차도 같이 초기화.
            if (is_on_ground){
                out_roll = 0.0f;
                out_pitch = 0.0f;
                out_yaw = 0.0f;
                //pid.pid_yaw_deg.integral = 0.0f;
                pid.pid_yaw_deg.err_prev = 0.0f;
            }

            // 작은 값은 dead‑band 처리
            if (fabsf(out_yaw) < 1.0f) {
                out_yaw = 0.0f;
            }
            // rc의 trottle의 값(target_rc_throttle)과 고도에 따른 출력기본값(alt_throttle_offset)
            // 최대값은 1050이 제한값이다 
            float base_pwm = 1000.0f + std::max(target_rc_throttle + alt_throttle_offset, 50.0f);

            // 1. 우선 클램프 없이 믹싱 계산 (임시 변수)

            // m1: Front-Left (CW)
            // m2: Front-Right (CCW)
            // m3: Rear-Left (CCW)
            // m4: Rear-Right (CW)
            float m1 = base_pwm + out_pitch + out_roll - out_yaw;
            float m2 = base_pwm + out_pitch - out_roll + out_yaw;
            float m3 = base_pwm - out_pitch + out_roll + out_yaw;
            float m4 = base_pwm - out_pitch - out_roll - out_yaw;


            // 2. 가장 많이 튀어나온(최대값) 모터 찾기
            float max_motor = m1;
            if (m2 > max_motor) max_motor = m2;
            if (m3 > max_motor) max_motor = m3;
            if (m4 > max_motor) max_motor = m4;

            // 3. 만약 최대값이 2000을 넘는다면, 넘는 만큼 모든 모터에서 공통으로 차감
            if (max_motor > 2000.0f) {
                float diff = max_motor - 2000.0f;
                m1 -= diff;
                m2 -= diff;
                m3 -= diff;
                m4 -= diff;
            }

            //float motor_v[4];
            m1 = std::clamp(m1, 1050.0f, 2000.0f);
            m2 = std::clamp(m2, 1050.0f, 2000.0f);
            m3 = std::clamp(m3, 1050.0f, 2000.0f);
            m4 = std::clamp(m4, 1050.0f, 2000.0f);

if (loop_cnt % 16 == 0) 
        ESP_LOGI(TAG, "| alt_throttle_offset : %8.3f | base: %8.3f| out_roll: %8.3f| out_pitch: %8.3f| out_yaw: %8.3f| m1: %8.3f| m2: %8.3f| m3: %8.3f| m4: %8.3f|", 
                                         alt_throttle_offset,base_pwm,out_roll,out_pitch,out_yaw,m1, m2, m3, m4);

            motor.update_compare_value({m1,m2,m3,m4});
        }else{
            // 시동 안 걸렸을 때는 모터 정지 및 PID 적분항 초기화
            motor.stop_all_motors();
            // 시동을 켜는 순간 '튀는' 현상을 방지합니다.
            pid.reset_pid(&pid.pid_roll_deg);
            pid.reset_pid(&pid.pid_pitch_deg);
            pid.reset_pid(&pid.pid_yaw_deg);
            pid.reset_pid(&pid.pid_roll_rate);
            pid.reset_pid(&pid.pid_pitch_rate);
            pid.reset_pid(&pid.pid_yaw_rate);
            pid.reset_pid(&pid.pid_alt_pos);  
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
    auto& mahony        = Filter::Mahony::get_instance();
    auto& motor         = Driver::Motor::get_instance();
    auto& gps           = Sensor::Gps::get_instance();
    auto& flysky        = Service::Flysky::get_instance();
    auto& buzzer        = Driver::Buzzer::get_instance();
    //auto& mavlink       = Service::Mavlink::get_instance();
    auto& telemetry     = Service::Telemetry::get_instance();
    //auto& pid           = Filetr::PID::get_instance();
    //auto& failsafe      = Service::FailSafe::get_instance();
    auto& timer         = Service::Timer::get_instance();
    
    esp_err_t err = ESP_FAIL;
    auto mac_addr = espnow.get_my_mac_address();
    ESP_LOGI(TAG, "My MAC address: %02x:%02x:%02x:%02x:%02x:%02x",mac_addr[0], mac_addr[1], mac_addr[2],mac_addr[3], mac_addr[4], mac_addr[5]);
    
    // // 각각의 오프셋을 구한다.
    icm20948_main.calibrate();  
    vTaskDelay(pdMS_TO_TICKS(50));
    icm20948_sub.calibrate();		
    vTaskDelay(pdMS_TO_TICKS(50));

    float mgp{};
    err = bmp388_main.calibrate_ground_pressure(&mgp);
    ENV::g_baro.ground_pressure = mgp;
    vTaskDelay(pdMS_TO_TICKS(50));
    err = bmp388_sub.calibrate_ground_pressure(&mgp);
    vTaskDelay(pdMS_TO_TICKS(50));    
    {// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (시작)==========	        
		auto [ret,acc,gyro]     = icm20948_main.read_with_offset();
        // 지자계 데이터를 읽는다. 		
        auto [ist_ret,ist_mag]  = ist8310.read_with_offset();
		// 융합된 데이터를 적용처리.
        mahony.calibrate_mahony_initial_attitude(acc[0],acc[1], acc[2],ist_mag[0],ist_mag[1],ist_mag[2]);
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
    err = espnow.connect_callback();
    if (err != ESP_OK){
        res = pdFAIL;
    }
    // 10hz,1hz...일정하게 qgc로 보내는 mavlink message처리를 한다.
    err = timer.Start();
    if (err != ESP_OK){
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
