
#include "ryu_main.h"

#include "ryu_i2c.h"
#include "ryu_icm20948.h"
#include "ryu_ist8310.h"
#include "ryu_ak09916.h"
#include "ryu_bmp388.h"
#include "ryu_gps.h"
#include "ryu_wifi.h"
#include "ryu_MahonyFilter.h"
#include "ryu_flysky.h"
#include "ryu_telemetry.h"
#include "ryu_battery.h"
#include "ryu_timer.h"
#include "ryu_mavlink.h"
#include "ryu_flight_task.h"
#include "ryu_failsafe.h"


static const char *MAINTAG = "MAIN";

// --- 메인 함수 ---
void app_main(void) {
    esp_err_t ret_code;

    // 시스템 시작시 여유를 준다.
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(MAINTAG, "시스템 부팅을 시작합니다...");

    { // 2번 포트 led 설정
        gpio_reset_pin(GPIO_NUM_2); // 핀 상태 초기화
        gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // 출력 모드로 설정       
        // buzzer 초기화및 fc의 시작을 알린다.
        gpio_set_level(GPIO_NUM_2,1);
    }

    // Driver::Buzzer::get_instance().initialize();
    // Driver::Buzzer::get_instance().sound_system_start();

     
    check_system_health_on_boot();

    log_information_control();
	
    watch_dog_initialize();
  
    // {
    //     // espnow 초기화
    //     auto& espnow = Service::EspNow::get_instance();
    //     espnow.initialize();
    //     auto mac_addr = espnow.get_my_mac_address();
    //     ESP_LOGI(MAINTAG, "현재 MAC 주소: %02x:%02x:%02x:%02x:%02x:%02x",
    //                 mac_addr[0], mac_addr[1], mac_addr[2],
    //                 mac_addr[3], mac_addr[4], mac_addr[5]);
    // }

    // {// 배터리 체크 ADC 초기화
    //     Driver::Battery::get_instance().initialize();
	// }
    
    // {// ========== [1단계] I2C 버스 생성 ==========
	// 	auto& i2c = Driver::I2C::get_instance();
    //     i2c.initialize(); 
	// 	if(i2c.get_bus_handle() == nullptr)
	// 		ESP_LOGI(MAINTAG, "I2C 초기화 실패!");
	// 	else
	// 	    ESP_LOGI(MAINTAG, "✓ I2C 초기화 완료");
	// 	vTaskDelay(pdMS_TO_TICKS(50));
	// }

	// { // 메인 imu sensor와  서브 imu  sensor을 초기화한다.
    //     auto& icm20948_main = Sensor::ICM20948::Main();
    //     auto& icm20948_sub  = Sensor::ICM20948::Sub();
    //     icm20948_main.initialize();
	// 	if(icm20948_main.get_dev_handle() ==nullptr)
	// 		ESP_LOGI(MAINTAG, "IMU MAIN MODULE 초기화 설정 실패!");
	// 	else 
	// 		ESP_LOGI(MAINTAG, "IMU MAIN MODULE 범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");

    //     // ak09916을 사용하기위하여 main sensor의 icm20948에서 bypass mode를 설정한다.
    //     ret_code = icm20948_main.enable_mag_bypass();
	// 	if (ret_code != ESP_OK)
	// 		ESP_LOGI(MAINTAG, "IMU MAIN MODULE Bypass 모드 설정 실패!");
	// 	else
	// 		ESP_LOGI(MAINTAG, "IMU MAIN MODULE Bypass 모드 설정 완료!");
	// 	vTaskDelay(pdMS_TO_TICKS(10));
	
	// 	icm20948_sub.initialize();
	// 	if(icm20948_sub.get_dev_handle() ==nullptr)
	// 		ESP_LOGI(MAINTAG, "IMU SUB MODULE 초기화 설정 실패!");
	// 	else 
	// 		ESP_LOGI(MAINTAG, "IMU SUB MODULE 범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");

    //     // 각각의 오프셋을 구한다.
	// 	icm20948_main.calibrate();
    //     icm20948_sub.calibrate();		

    // }
    
        
    // {// Gps에 있는 지자계 센서를 사용하기 위하여 초기화 한다.( 두 번째 지자계 센서로 ak09916을 등록 할지 생각해보자.)
    //     auto& ist8310 = Sensor::IST8310::get_instance();
    //     auto& ak09916 = Sensor::AK09916::get_instance();

    //     ist8310.initialize(); 
    //     if (ist8310.get_dev_handle() == nullptr) 
    //         ESP_LOGW("WARNING","IST8310 등록실패!");
	// 	ak09916.initialize();
    //     if (ak09916.get_dev_handle() == nullptr) 
    //         ESP_LOGW("WARNING","AK09916 등록실패!");
	// 	vTaskDelay(pdMS_TO_TICKS(50));
	// }
    

    // {// 기압계센서를 초기화하고 현재 위치의 기압을 체크한다.(나중에 상대 고도의 기준이 되는 값.)
    //     auto& bmp388_main = Sensor::BMP388::Main();
    //     auto& bmp388_sub  = Sensor::BMP388::Sub(); 
    //     ret_code = bmp388_main.initialize();
    //     if (ret_code !=ESP_OK){

    //     }
    //     ret_code = bmp388_sub.initialize(); 
    //     if (ret_code !=ESP_OK){

    //     }
    //     auto [ret_bmp0,mgp] = bmp388_main.calibrate_ground_pressure();
    //     vTaskDelay(pdMS_TO_TICKS(50));
    //     auto [ret_bmp1,sgp] = bmp388_sub.calibrate_ground_pressure();
    //     g_baro.ground_pressure = (mgp+sgp) * 0.5;
    // }
    
    
    // {// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (시작)==========	        
	// 	std::tie(ret_code,g_imu.acc,g_imu.gyro)     = Sensor::ICM20948::Main().read_with_offset();
	// 	g_imu.acc[1]    *=  -1.0f;  // 오른손 법칙에 적용 2가지 모두 (-)부호를 해야한다 (여기는 gyro는 사용하지 않지만 알아두라는 알림의 표시로...)
    //     g_imu.gyro[0]   *=  -1.0f;

    //     // 지자계 데이터를 읽는다. 		
    //     auto [ist_ret,ist_mag]  = Sensor::IST8310::get_instance().read_with_offset();
	// 	auto [ ak_ret, ak_mag]  = Sensor::AK09916::get_instance().read_with_offset();    

    //     g_imu.mag[0] = (ist_mag[0]+ak_mag[0])*0.5;
    //     g_imu.mag[1] = (ist_mag[1]+ak_mag[1])*0.5;
    //     g_imu.mag[2] = (ist_mag[2]+ak_mag[2])*0.5;

	// 	// 융합된 데이터를 적용처리.
    //     auto& mahony = Service::Mahony::get_instance();
    //     mahony.calibrate_mahony_initial_attitude(g_imu.acc[0],g_imu.acc[1], g_imu.acc[2],g_imu.mag[0],g_imu.mag[1],g_imu.mag[2]);
	// 	g_imu.acc   ={};
	// 	g_imu.gyro  ={};
	// 	g_imu.mag   ={};
	// 	ESP_LOGI(MAINTAG, "✓ Mahony AHRS 초기화 완료");
	// 	// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (끝)==========
	// }
    

	// {
	// 	// 모터 PWM 초기화
    //     Driver::Motor::get_instance().initialize();
	// 	vTaskDelay(pdMS_TO_TICKS(50));
	// 	// GPS 초기화
    //     Sensor::Gps::get_instance().initialize();
	// 	vTaskDelay(pdMS_TO_TICKS(50));
	// 	// FLYSKY  초기화
    //     Service::Flysky::get_instance().initialize();
	// 	vTaskDelay(pdMS_TO_TICKS(50));
	
	// }

    // // ========== [3단계] 센서 연결 상태 검증 (critical check) ==========
    // if (
    //     Sensor::ICM20948::Main().get_dev_handle() == nullptr ||
    //     Sensor::ICM20948::Sub().get_dev_handle() == nullptr ||
    //     Sensor::IST8310::get_instance().get_dev_handle() == nullptr ||
    //     Sensor::AK09916::get_instance().get_dev_handle() == nullptr ||
    //     Sensor::BMP388::Main().get_dev_handle()   == nullptr ||
    //     Sensor::BMP388::Sub().get_dev_handle()    == nullptr) 
    //     {
    //     if (Sensor::ICM20948::Main().get_dev_handle() == nullptr) 
    //         ESP_LOGE(MAINTAG, "❌ %s MAIN에 연결할 수 없습니다!", IMU_NAME_MAIN);
    //     if (Sensor::ICM20948::Sub().get_dev_handle() == nullptr) 
    //         ESP_LOGE(MAINTAG, "❌ %s SUB에 연결할 수 없습니다!", IMU_NAME_SUB);
    //     if( Sensor::IST8310::get_instance().get_dev_handle() == nullptr) 
    //         ESP_LOGE(MAINTAG, "❌ %s MAIN에 연결할 수 없습니다!", MAG_NAME_MAIN);
    //     if( Sensor::AK09916::get_instance().get_dev_handle()  == nullptr) 
    //         ESP_LOGE(MAINTAG, "❌ %s SUB에 연결할 수 없습니다!", MAG_NAME_SUB);
    //     if (Sensor::BMP388::Main().get_dev_handle()   == nullptr) 
    //         ESP_LOGE(MAINTAG, "❌ %s MAIN에 연결할 수 없습니다!", BARO_NAME_MAIN);
    //     if (Sensor::BMP388::Sub().get_dev_handle()    == nullptr) 
    //         ESP_LOGE(MAINTAG, "❌ %s SUB에 연결할 수 없습니다!", BARO_NAME_SUB);

    //     ESP_LOGE(MAINTAG, "❌ 필수 센서 미연결! 시스템 중단");

    //     Driver::Motor::get_instance().stop_all_motors();    

    //     // 무한 대기 (리부팅 필요)
    //     while (true) {
    //         static bool blink_led = false;
    //         gpio_set_level(GPIO_NUM_2,(blink_led = !blink_led));
    //         vTaskDelay(pdMS_TO_TICKS(100));
    //         Driver::Buzzer::get_instance().sound_error();
    //     }
    // }

    // //mavlink분석및 qgc와 communication.
    // auto& mavlink =  Service::Mavlink::get_instance();
    // mavlink.initialize();
    

    // // ========== [4단계] 보조 태스크 생성 ==========

    // // 0. espnow tx task
    // auto& espnow = Service::EspNow::get_instance(); 
    // espnow.start_task();

    // auto& flysky = Service::Flysky::get_instance();
    // flysky.initialize();
    // flysky.start_task();

    // auto& gps = Sensor::Gps::get_instance();
    // gps.initialize();
    // gps.start_task();
    
    // auto& telemetry = Service::Telemetry::get_instance();
    // telemetry.initialize();
    // telemetry.start_task();

    
    // auto& bat = Driver::Battery::get_instance();
    // bat.initialize();
    // bat.start_task();
    
    
    // auto& failsafe = Service::FailSafe::get_instance();
    // failsafe.initialize();
    // failsafe.start_task();
    

    auto& flight = Controller::Flight::get_instance();
    flight.initialize();
    flight.start_task();
    // // ========== [5단계] 비행 제어 태스크 생성 (모든 검증 완료 후) ==========
    // res = xTaskCreatePinnedToCore(FLIGHT::flight_task, "flight", 8192, NULL, 24, NULL, 1);
    // if (res != pdPASS) {
    //     ESP_LOGE(MAINTAG, "❌ 6.Flight Task is Failed! code: %d", res);
    //     // 모터 안전 정지
    //     Driver::Motor::get_instance().stop_all_motors();
    // } else {
    //     ESP_LOGI(MAINTAG, "✓ 6.Flight Task is passed...");
    // }

    // ESP_LOGI(MAINTAG, "✅ All Processes is passed... Flight ready!");
    
    // // 콜백이 등록되어야지 데이터가 들어온다.
    // espnow.connect_callback();
    
    // {// Timer service 초기화
    //     auto& timer = Service::Timer::get_instance();
    //     timer.intiallize();
    //     timer.Start();
    // }
    
    while (true) {
        static bool led_state = false;
        gpio_set_level(GPIO_NUM_2, (led_state = !led_state));
        vTaskDelay(pdMS_TO_TICKS(500)); // 메인 태스크가 종료되지 않게 붙잡음
    }
}
