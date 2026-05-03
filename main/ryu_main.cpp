
#include "ryu_main.h"

#include "ryu_i2c.h"
#include "ryu_icm20948.h"
#include "ryu_ist8310.h"
#include "ryu_ak09916.h"
#include "ryu_MahonyFilter.h"
#include "ryu_flysky.h"
#include "ryu_telemetry.h"
#include "ryu_gps.h"
#include "ryu_battery.h"
#include "ryu_error_proc.h"
#include "ryu_timer.h"
#include "ryu_flight_task.h"
#include "ryu_bmp388.h"
#include "ryu_mavlink.h"
#include "ryu_wifi.h"


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
        Driver::Buzzer::get_instance().initialize();
        Driver::Buzzer::get_instance().sound_system_start();
        gpio_set_level(GPIO_NUM_2,1);
    }
    
     
    check_system_health_on_boot();

    log_information_control();
	
    watch_dog_initialize();
  
    WIFI::init_wifi();
    WIFI::init_esp_now();         // ESP-NOW 초기화 (송수신)
        
    {// 배터리 체크 ADC 초기화
        Driver::Battery::get_instance().initialize();
	}
    
    {// ========== [1단계] I2C 버스 생성 ==========
		Driver::I2C::get_instance().initialize(); 
		if(Driver::I2C::get_instance().get_bus_handle() == nullptr)
			ESP_LOGI(MAINTAG, "I2C 초기화 실패!");
		else
		    ESP_LOGI(MAINTAG, "✓ I2C 초기화 완료");
		vTaskDelay(pdMS_TO_TICKS(50));
	}

	{ // 메인 imu sensor와  서브 imu  sensor을 초기화한다.
		Sensor::ICM20948::Main().initialize();
		if(Sensor::ICM20948::Main().get_dev_handle() ==nullptr)
			ESP_LOGI(MAINTAG, "IMU MAIN MODULE 초기화 설정 실패!");
		else 
			ESP_LOGI(MAINTAG, "IMU MAIN MODULE 범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");

		Sensor::ICM20948::Sub().initialize();
		if(Sensor::ICM20948::Sub().get_dev_handle() ==nullptr)
			ESP_LOGI(MAINTAG, "IMU SUB MODULE 초기화 설정 실패!");
		else 
			ESP_LOGI(MAINTAG, "IMU SUB MODULE 범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");

        // 각각의 오프셋을 구한다.
		Sensor::ICM20948::Main().calibrate();
        Sensor::ICM20948::Sub().calibrate();		

    }
    
	{ // ak09916을 사용하기위하여 main sensor의 icm20948에서 bypass mode를 설정한다.
		ret_code = Sensor::ICM20948::Main().enable_mag_bypass();
		if (ret_code != ESP_OK)
			ESP_LOGI(MAINTAG, "IMU MAIN MODULE Bypass 모드 설정 실패!");
		else
			ESP_LOGI(MAINTAG, "IMU MAIN MODULE Bypass 모드 설정 완료!");
		vTaskDelay(pdMS_TO_TICKS(10));
	}
    
    
    {// Gps에 있는 지자계 센서를 사용하기 위하여 초기화 한다.( 두 번째 지자계 센서로 ak09916을 등록 할지 생각해보자.)
        Sensor::IST8310::get_instance().initialize(); 
        if (Sensor::IST8310::get_instance().get_dev_handle() == nullptr) 
            ESP_LOGW("WARNING","IST8310 등록실패!");
		Sensor::AK09916::get_instance().initialize();
        if (Sensor::AK09916::get_instance().get_dev_handle() == nullptr) 
            ESP_LOGW("WARNING","AK09916 등록실패!");
		vTaskDelay(pdMS_TO_TICKS(50));
	}
    
    {// 기압계센서를 초기화하고 현재 위치의 기압을 체크한다.(나중에 상대 고도의 기준이 되는 값.)
        ret_code = Sensor::BMP388::Main().initialize();
        if (ret_code !=ESP_OK){

        }
        ret_code = Sensor::BMP388::Sub().initialize(); 
        if (ret_code !=ESP_OK){
            
        }
        auto [ret_bmp0,mgp] = Sensor::BMP388::Main().calibrate_ground_pressure();
        auto [ret_bmp1,sgp] = Sensor::BMP388::Sub().calibrate_ground_pressure();
        g_baro.ground_pressure = (mgp+sgp) * 0.5;
    }
    
    
    {// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (시작)==========	
		// IMU 데이터를 읽는다. 		
		std::tie(ret_code,g_imu.acc,g_imu.gyro)     = Sensor::ICM20948::Main().read_with_offset();
		g_imu.acc[1]    *=  -1.0f;  // 오른손 법칙에 적용 2가지 모두 (-)부호를 해야한다 (여기는 gyro는 사용하지 않지만 알아두라는 알림의 표시로...)
        g_imu.gyro[0]   *=  -1.0f;

        // 지자계 데이터를 읽는다. 		
        auto [ist_ret,ist_mag]  = Sensor::IST8310::get_instance().read_with_offset();
		auto [ ak_ret, ak_mag]  = Sensor::AK09916::get_instance().read_with_offset();    

        g_imu.mag[0] = (ist_mag[0]+ak_mag[0])*0.5;
        g_imu.mag[1] = (ist_mag[1]+ak_mag[1])*0.5;
        g_imu.mag[2] = (ist_mag[2]+ak_mag[2])*0.5;

		// 융합된 데이터를 적용처리.
        auto& mahony = Service::Mahony::get_instance();
        mahony.calibrate_mahony_initial_attitude(g_imu.acc[0],g_imu.acc[1], g_imu.acc[2],g_imu.mag[0],g_imu.mag[1],g_imu.mag[2]);
		g_imu.acc   ={};
		g_imu.gyro  ={};
		g_imu.mag   ={};
		ESP_LOGI(MAINTAG, "✓ Mahony AHRS 초기화 완료");
		// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (끝)==========
	}
    
	{
		// 모터 PWM 초기화
        Driver::Motor::get_instance().initialize();
		vTaskDelay(pdMS_TO_TICKS(50));
		// GPS 초기화
		GPS::initialize();
		vTaskDelay(pdMS_TO_TICKS(50));
		// FLYSKY  초기화
        Service::Flysky::get_instance().initialize();
		vTaskDelay(pdMS_TO_TICKS(50));
	
	}

    // ========== [3단계] 센서 연결 상태 검증 (critical check) ==========
    if (
        Sensor::ICM20948::Main().get_dev_handle() == nullptr ||
        Sensor::ICM20948::Sub().get_dev_handle() == nullptr ||
        Sensor::IST8310::get_instance().get_dev_handle() == nullptr ||
        Sensor::AK09916::get_instance().get_dev_handle() == nullptr ||
        Sensor::BMP388::Main().get_dev_handle()   == nullptr ||
        Sensor::BMP388::Sub().get_dev_handle()    == nullptr) 
        {
        if (Sensor::ICM20948::Main().get_dev_handle() == nullptr) 
            ESP_LOGE(MAINTAG, "❌ %s MAIN에 연결할 수 없습니다!", IMU_NAME_MAIN);
        if (Sensor::ICM20948::Sub().get_dev_handle() == nullptr) 
            ESP_LOGE(MAINTAG, "❌ %s SUB에 연결할 수 없습니다!", IMU_NAME_SUB);
        if( Sensor::IST8310::get_instance().get_dev_handle() == nullptr) 
            ESP_LOGE(MAINTAG, "❌ %s MAIN에 연결할 수 없습니다!", MAG_NAME_MAIN);
        if( Sensor::AK09916::get_instance().get_dev_handle()  == nullptr) 
            ESP_LOGE(MAINTAG, "❌ %s SUB에 연결할 수 없습니다!", MAG_NAME_SUB);
        if (Sensor::BMP388::Main().get_dev_handle()   == nullptr) 
            ESP_LOGE(MAINTAG, "❌ %s MAIN에 연결할 수 없습니다!", BARO_NAME_MAIN);
        if (Sensor::BMP388::Sub().get_dev_handle()    == nullptr) 
            ESP_LOGE(MAINTAG, "❌ %s SUB에 연결할 수 없습니다!", BARO_NAME_SUB);

        ESP_LOGE(MAINTAG, "❌ 필수 센서 미연결! 시스템 중단");

        
        Driver::Motor::get_instance().stop_all_motors();
        

        auto mac_addr = WIFI::get_my_mac_address();
        ESP_LOGE(MAINTAG, "현재 MAC 주소: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);

        // 무한 대기 (리부팅 필요)
        while (true) {
            static bool blink_led = false;
            gpio_set_level(GPIO_NUM_2,(blink_led = !blink_led));
            vTaskDelay(pdMS_TO_TICKS(100));
            Driver::Buzzer::get_instance().sound_error();
        }
    }

    

    // ========== [4단계] 보조 태스크 생성 ==========
    BaseType_t res;

    WIFI::mavlink_tx_queue      = xQueueCreate(WIFI::MAVLINK_TX_QUEUE_SIZE, sizeof(WIFI::mav_tx_packet_t));
    TELEM::mavlink_rx_queue     = xQueueCreate(WIFI::MAVLINK_TX_QUEUE_SIZE , sizeof(TELEM::esp_now_data_t));

    auto& mavlink =  Service::Mavlink::get_instance();
    mavlink.initialize();

    // 2. 송신 Task 생성 (우선순위를 높게 설정)
    xTaskCreatePinnedToCore(WIFI::mavlink_tx_task, "mavlink_tx_task", 4096, NULL, 15, NULL, 0);

    auto& flysky = Service::Flysky::get_instance();
    flysky.start_task();

    res = xTaskCreatePinnedToCore(GPS::gps_ubx_mode_task, "gps", 4096, NULL, 5, NULL, 0);
    if (res != pdPASS) ESP_LOGE(MAINTAG, "❌ 2.Gps Task is Failed! code: %d", res);
    else ESP_LOGI(MAINTAG, "✓ 2.Gps Task is passed... ");
    
    res = xTaskCreatePinnedToCore(TELEM::telemetry_task, "telemetry", 8192, NULL, 15, NULL, 0);
    if (res != pdPASS) ESP_LOGE(MAINTAG, "❌ 3.Telemetry Task is failed! code: %d", res);
    else ESP_LOGI(MAINTAG, "✓ 3.Telemetry task is passed...");
    
    Driver::Battery::get_instance().start_task();
    
    res= xTaskCreatePinnedToCore(ERR::error_manager_task, "ErrMgr", 4096, NULL, 10, &ERR::xErrorHandle, 0);
    if (res != pdPASS) ESP_LOGE(MAINTAG, "❌ 5.Error Check Task is failed! code: %d", res);
    else ESP_LOGI(MAINTAG, "✓ 5.Error Check Task is passed...");


    // ========== [5단계] 비행 제어 태스크 생성 (모든 검증 완료 후) ==========
    res = xTaskCreatePinnedToCore(FLIGHT::flight_task, "flight", 8192, NULL, 24, NULL, 1);
    if (res != pdPASS) {
        ESP_LOGE(MAINTAG, "❌ 6.Flight Task is Failed! code: %d", res);
        // 모터 안전 정지
        Driver::Motor::get_instance().stop_all_motors();
    } else {
        ESP_LOGI(MAINTAG, "✓ 6.Flight Task is passed...");
    }

    ESP_LOGI(MAINTAG, "✅ All Processes is passed... Flight ready!");
    
    // 콜백이 등록되어야지 데이터가 들어온다.
    WIFI::connect_callback();
    
    {// Timer service 초기화
        auto& timer = Service::Timer::get_instance();
        timer.intiallize();
        timer.Start();
    }
    
    while (true) {
        static bool led_state = false;
        gpio_set_level(GPIO_NUM_2, (led_state = !led_state));
        vTaskDelay(pdMS_TO_TICKS(500)); // 메인 태스크가 종료되지 않게 붙잡음
    }
}
