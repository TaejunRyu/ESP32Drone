
#include "ryu_main.h"

#ifndef UNIT_TEST  // 유닛 테스트 중이 아닐 때만 아래 코드를 포함

class CBMP388 cbmp388_main;
class CBMP388 cbmp388_sub;


// --- 메인 함수 ---
void app_main(void) {
    esp_err_t ret_code;
    static bool led_state = false;

    // 시스템 시작시 여유를 준다.
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI("MAIN", "시스템 부팅을 시작합니다...");

    { // 2번 포트 led 설정
        gpio_reset_pin(GPIO_NUM_2); // 핀 상태 초기화
        gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // 출력 모드로 설정
        // buzzer 초기화및 fc의 시작을 알린다.
        BUZZ::initialize();
        BUZZ::sound_system_start();
    }
    
    gpio_set_level(GPIO_NUM_2, (led_state = !led_state));
     
    check_system_health_on_boot();

    log_information_control();
	
    watch_dog_initialize();
  
    WIFI::init_wifi();
    WIFI::init_esp_now();         // ESP-NOW 초기화 (송수신)
        
    {// 배터리 체크 ADC 초기화
        BATT::initialize();
	}


    {// ========== [1단계] I2C 버스 생성 ==========
		i2c_handle      = I2C::initialize((i2c_port_num_t)0,I2C_SDA,I2C_SCL); 
		if(i2c_handle == nullptr)
			ESP_LOGI("MAIN", "I2C 초기화 실패!");
		else
		ESP_LOGI("MAIN", "✓ I2C 초기화 완료");
		vTaskDelay(pdMS_TO_TICKS(50));
	}

	{ // 메인 imu sensor와  서브 imu  sensor을 초기화한다.
		imu_handle[0]  = ICM20948::initialize(i2c_handle,ICM20948::ADDR_VCC);
		if(imu_handle[0] ==nullptr)
			ESP_LOGI("MAIN", "IMU MAIN MODULE 초기화 설정 실패!");
		else 
			ESP_LOGI("MAIN", "IMU MAIN MODULE 범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");

		imu_handle[1]      = ICM20948::initialize(i2c_handle,ICM20948::ADDR_GND);
		if(imu_handle[1] ==nullptr)
			ESP_LOGI("MAIN", "IMU SUB MODULE 초기화 설정 실패!");
		else 
			ESP_LOGI("MAIN", "IMU SUB MODULE 범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");

        // 각각의 오프셋을 구한다.
		std::tie(g_imu_offset[0].acc,g_imu_offset[0].gyro) = ICM20948::calibrate(imu_handle[0]);		
        std::tie(g_imu_offset[1].acc,g_imu_offset[1].gyro) = ICM20948::calibrate(imu_handle[1]);    

    }
    
	{ // ak09916을 사용하기위하여 main sensor의 icm20948에서 bypass mode를 설정한다.
		ret_code = ICM20948::enable_mag_bypass(imu_handle[0]);
		if (ret_code != ESP_OK)
			ESP_LOGI("MAIN", "IMU MAIN MODULE Bypass 모드 설정 실패!");
		else
			ESP_LOGI("MAIN", "IMU MAIN MODULE Bypass 모드 설정 완료!");
		vTaskDelay(pdMS_TO_TICKS(10));
	}
    gpio_set_level(GPIO_NUM_2, (led_state = !led_state));
    
    {// Gps에 있는 지자계 센서를 사용하기 위하여 초기화 한다.( 두 번째 지자계 센서로 ak09916을 등록 할지 생각해보자.)
		mag_handle[MAIN]      = IST8310::initialize(i2c_handle);
        if (mag_handle[MAIN] == NULL) ESP_LOGW("WARNING","IST8310 등록실패!");
		mag_handle[SUB]      = AK09916::initialize(i2c_handle);
        if (mag_handle[SUB] == NULL) ESP_LOGW("WARNING","AK09916 등록실패!");
		vTaskDelay(pdMS_TO_TICKS(50));
	}
    

    // 주소 체크 및 센서 연결 상태 확인을 위해 I2C 버스 스캔 함수를 호출합니다.
    //I2C::scan_bus(i2c_handle);                 

    // ========== TASK 시작전에 기타 확인할 정보 처리 구간 ==========
    // 지자계 센서의 하드아이언 보정 (중요)
    // BUZZ::sound_processing();
    // AK09916::calibrate_hard_iron(mag_handle[1]); 
    // BUZZ::sound_success();
    // while(true)(vTaskDelay(pdMS_TO_TICKS(1000)));


    {// 기압계센서를 초기화하고 현재 위치의 기압을 체크한다.(나중에 상대 고도의 기준이 되는 값.)
        ret_code = cbmp388_main.initialize(i2c_handle,CBMP388::ADDR_VCC);
        ret_code = cbmp388_sub.initialize(i2c_handle, CBMP388::ADDR_GND); 
        auto [ret_bmp0,mgp] = cbmp388_main.calibrate_ground_pressure();
        auto [ret_bmp1,sgp] = cbmp388_sub.calibrate_ground_pressure();
        g_baro.ground_pressure = (mgp+sgp) * 0.5;
    }
    
    
    {// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (시작)==========	
		// IMU 데이터를 읽는다. 		
		std::tie(ret_code,g_imu.acc,g_imu.gyro)     = ICM20948::read_with_offset(imu_handle[0],g_imu_offset[0].acc,g_imu_offset[0].gyro);
		g_imu.acc[Y]    *=  -1.0f;  // 오른손 법칙에 적용 2가지 모두 (-)부호를 해야한다 (여기는 gyro는 사용하지 않지만 알아두라는 알림의 표시로...)
        g_imu.gyro[X]   *=  -1.0f;

        // 지자계 데이터를 읽는다. 		
        auto [ist_ret,ist_mag]  = IST8310::read_with_offset(mag_handle[0]);
		auto [ ak_ret, ak_mag]  = AK09916::read_with_offset(mag_handle[1]);    

        g_imu.mag[X] = (ist_mag[X]+ak_mag[X])*0.5;
        g_imu.mag[Y] = (ist_mag[Y]+ak_mag[Y])*0.5;
        g_imu.mag[Z] = (ist_mag[Z]+ak_mag[Z])*0.5;

		// 융합된 데이터를 적용처리.
		AHRS::calibrate_mahony_initial_attitude(g_imu.acc[X],g_imu.acc[Y], g_imu.acc[Z],g_imu.mag[X],g_imu.mag[Y],g_imu.mag[Z]);
		g_imu.acc   ={};
		g_imu.gyro  ={};
		g_imu.mag   ={};
		ESP_LOGI("MAIN", "✓ Mahony AHRS 초기화 완료");
		// ========== Mahony AHRS 초기 롤/피치 캘리브레이션 (끝)==========
	}
    
	{
		// 모터 PWM 초기화
		SERVO::initialize();
		vTaskDelay(pdMS_TO_TICKS(50));
		// GPS 초기화
		GPS::initialize();
		vTaskDelay(pdMS_TO_TICKS(50));
		
		// FLYSKY  초기화
		FLYSKY::initialize();
		vTaskDelay(pdMS_TO_TICKS(50));
	
	}

    
    
    // ========== [3단계] 센서 연결 상태 검증 (critical check) ==========
    if (imu_handle[MAIN]    == NULL || 
        imu_handle[SUB]     == NULL || 
        mag_handle[MAIN]    == NULL || 
        mag_handle[SUB]     == NULL || 
        cbmp388_main.get_handle()   == NULL ||
        cbmp388_sub.get_handle()    == NULL) 
        {
        if (imu_handle[MAIN]            == NULL) ESP_LOGE("MAIN", "❌ %s MAIN에 연결할 수 없습니다!", IMU_NAME_MAIN);
        if (imu_handle[SUB]             == NULL) ESP_LOGE("MAIN", "❌ %s SUB에 연결할 수 없습니다!", IMU_NAME_SUB);
        if( mag_handle[MAIN]            == NULL) ESP_LOGE("MAIN", "❌ %s MAIN에 연결할 수 없습니다!", MAG_NAME_MAIN);
        if( mag_handle[SUB]             == NULL) ESP_LOGE("MAIN", "❌ %s SUB에 연결할 수 없습니다!", MAG_NAME_SUB);
        if (cbmp388_main.get_handle()   == NULL) ESP_LOGE("MAIN", "❌ %s MAIN에 연결할 수 없습니다!", BARO_NAME_MAIN);
        if (cbmp388_sub.get_handle()    == NULL) ESP_LOGE("MAIN", "❌ %s SUB에 연결할 수 없습니다!", BARO_NAME_SUB);

        ESP_LOGE("MAIN", "❌ 필수 센서 미연결! 시스템 중단");
        
        SERVO::stop_all_motors();
        
        // 무한 대기 (리부팅 필요)
        while (true) {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(GPIO_NUM_2, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    

    // ========== [4단계] 보조 태스크 생성 ==========
    BaseType_t res;

    WIFI::mavlink_tx_queue      = xQueueCreate(MAVLINK_TX_QUEUE_SIZE, sizeof(WIFI::mav_tx_packet_t));
    TELEM::mavlink_rx_queue     = xQueueCreate(MAVLINK_TX_QUEUE_SIZE , sizeof(TELEM::esp_now_data_t));

    // 2. 송신 Task 생성 (우선순위를 높게 설정)
    xTaskCreatePinnedToCore(WIFI::mavlink_tx_task, "mavlink_tx_task", 4096, NULL, 15, NULL, 0);

    res = xTaskCreatePinnedToCore(FLYSKY::flysky_task,"flysky",4096,NULL,12,NULL,0);
    if (res != pdPASS) ESP_LOGE("MAIN", "❌ 1.Flysky Task is failed!  code: %d", res);
    else ESP_LOGI("MAIN", "✓ 1.Flysky Task is passed...");

    res = xTaskCreatePinnedToCore(GPS::gps_ubx_mode_task, "gps", 4096, NULL, 5, NULL, 0);
    if (res != pdPASS) ESP_LOGE("MAIN", "❌ 2.Gps Task is Failed! code: %d", res);
    else ESP_LOGI("MAIN", "✓ 2.Gps Task is passed... ");
    
    res = xTaskCreatePinnedToCore(TELEM::telemetry_task, "telemetry", 8192, NULL, 15, NULL, 0);
    if (res != pdPASS) ESP_LOGE("MAIN", "❌ 3.Telemetry Task is failed! code: %d", res);
    else ESP_LOGI("MAIN", "✓ 3.Telemetry task is passed...");
    
    res= xTaskCreatePinnedToCore(BATT::battery_check_task, "Battery", 4096, NULL, 10, NULL, 0);
    if (res != pdPASS) ESP_LOGE("MAIN", "❌ 4.Battery Check Task is failed! code: %d", res);
    else ESP_LOGI("MAIN", "✓ 4.Battery Task is passed...");
    
    res= xTaskCreatePinnedToCore(ERR::error_manager_task, "ErrMgr", 4096, NULL, 10, &ERR::xErrorHandle, 0);
    if (res != pdPASS) ESP_LOGE("MAIN", "❌ 5.Error Check Task is failed! code: %d", res);
    else ESP_LOGI("MAIN", "✓ 5.Error Check Task is passed...");


    // ========== [5단계] 비행 제어 태스크 생성 (모든 검증 완료 후) ==========
    res = xTaskCreatePinnedToCore(FLIGHT::flight_task, "flight", 8192, NULL, 24, NULL, 1);
    if (res != pdPASS) {
        ESP_LOGE("MAIN", "❌ 6.Flight Task is Failed! code: %d", res);
        // 모터 안전 정지
        for(auto& comp : SERVO::comparators) mcpwm_comparator_set_compare_value(comp, 1000);
    } else {
        ESP_LOGI("MAIN", "✓ 6.Flight Task is passed...");
    }

    ESP_LOGI("MAIN", "✅ All Processes is passed... Flight ready!");
    
    // 콜백이 등록되어야지 데이터가 들어온다.
    WIFI::connect_callback();

    TIMER::setupTimer();

    while (true) {
        gpio_set_level(GPIO_NUM_2, (led_state = !led_state));
        vTaskDelay(pdMS_TO_TICKS(500)); // 메인 태스크가 종료되지 않게 붙잡음
    }
}
#endif

