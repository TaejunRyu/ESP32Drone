#include <esp_ota_ops.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>

esp_err_t run_ota_update(const char* url) {
    esp_http_client_config_t config = {};
    config.url = url;
    config.skip_cert_common_name_check = true; // 테스트용 (HTTPS 인증서 체크 건너뜀)

    printf("Starting OTA update from: %s\n", url);

    // OTA 실행 (비차단 태스크에서 호출 권장)
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        printf("OTA Update Success! Rebooting...\n");
        esp_restart(); // 성공 시 즉시 재부팅
    } else {
        printf("OTA Update Failed! Error: %d\n", ret);
    }
    return ret;
}
void start_ota_if_safe(const char* firmware_url) {
    // 1. 시동 상태 체크 (필수!)
    if (is_armed) { 
        printf("OTA Rejected: Motors are armed!\n");
        send_command_ack(MAV_CMD_DO_CONTROL_VIDEO, MAV_RESULT_TEMPORARILY_REJECTED);
        return;
    }

    // 2. 배터리 전압 체크 (권장)
    if (battery_voltage < 11.1f) { // 예: 3S 배터리 기준
        printf("OTA Rejected: Low Battery!\n");
        return;
    }

    // 3. 업데이트 실행
    run_ota_update(firmware_url);
}
