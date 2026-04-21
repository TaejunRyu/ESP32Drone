#include "ryu_nvs.h"
#include "ryu_paramtable.h"

namespace NVS
{

// [저장] 특정 인덱스의 파라미터 정보 저장 (Pointer 제외)
esp_err_t save_param_struct_to_nvs(int index) {
    using  namespace PARAM;
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // NVS Key는 최대 15자이므로 name의 일부를 사용
    char key[16];
    snprintf(key, sizeof(key), "p_%d", index); // 인덱스 번호를 키로 사용 (가장 안전)

    // 실제 저장할 데이터 구조 (Pointer는 저장 제외)
    struct {
        char name[16];
        float value;
        uint8_t type;
    } temp_data;

    
    memcpy(temp_data.name,params[index].name.data(),16); // std::string_view에서 char 배열로 복사

    float *param_ptr =  reinterpret_cast<float*>(&values);

    temp_data.value = param_ptr[index];
    temp_data.type  =  params[index].type;

    // Binary Blob으로 저장
    err = nvs_set_blob(my_handle, key, &temp_data, sizeof(temp_data));
    if (err == ESP_OK) nvs_commit(my_handle);

    nvs_close(my_handle);
    return err;
}

// [로드] 부팅 시 NVS에서 값을 읽어와 pointer와 매칭
void load_params_struct_from_nvs() {
    using namespace PARAM;
    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK) return;

    for (int i = 0; i < count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "p_%d", i);

        struct {
            char name[16];
            float value;
            uint8_t type;
        } temp_data;

        size_t required_size = sizeof(temp_data);
        if (nvs_get_blob(my_handle, key, &temp_data, &required_size) == ESP_OK) {
            // 1. 값 업데이트
            float *param_ptr =  reinterpret_cast<float*>(&values);
            param_ptr[i] = temp_data.value;
            // 2. 연결된 실제 변수(Pointer)가 있다면 동기화 (가장 중요)
        }
    }
    nvs_close(my_handle);
}



inline constexpr char NVS_MAIN_IMU_OFFSET[] = "MAINIMU_OFFSET";
inline constexpr char  NVS_SUB_IMU_OFFSET[] = "SUBIMU_OFFSET";
// NVS에 오프셋 저장
esp_err_t save_offsets_to_nvs(char *nvs_name,const std::array<float, 3>& acc, const std::array<float, 3>& gyro) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(nvs_name, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // 가속도와 자이로 오프셋 저장 (blob 방식)
    nvs_set_blob(my_handle, "ACC_OFF", acc.data(), sizeof(float) * 3);
    nvs_set_blob(my_handle, "GYRO_OFF", gyro.data(), sizeof(float) * 3);
    
    err = nvs_commit(my_handle);
    nvs_close(my_handle);
    return err;
}

// NVS에서 오프셋 불러오기
esp_err_t load_offsets_from_nvs(char *nvs_name,std::array<float, 3>& acc, std::array<float, 3>& gyro) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(nvs_name, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(float) * 3;
    nvs_get_blob(my_handle, "ACC_OFF", acc.data(), &required_size);
    nvs_get_blob(my_handle, "GYRO_OFF", gyro.data(), &required_size);

    nvs_close(my_handle);
    return ESP_OK;
}



// 3. 통합 관리 로직 (app_main 전략)
// 이 로직은 부팅 시 NVS를 체크하여 데이터가 없으면 캘리브레이션을 유도합니다.

SensorCalibration calib_data = {};

void init_nvs_and_load_calib() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        size_t size = sizeof(SensorCalibration);
        // NVS에서 기존 데이터 로드 시도
        if (nvs_get_blob(handle, "calib_data", &calib_data, &size) != ESP_OK || !calib_data.is_calibrated) {
            ESP_LOGW("FC", "캘리브레이션 데이터 없음! 새로 측정합니다...");
            
            // 1. 가속도/자이로 정적 캘리브레이션 수행 (수평 유지 필수)
            // calib_data.acc_offset, gyro_offset 채우기
            
            // 2. 지자기 캘리브레이션 수행 (기체를 8자로 돌리기)
            // calib_data.mag_offset 채우기
            
            calib_data.is_calibrated = true;
            nvs_set_blob(handle, "calib_data", &calib_data, sizeof(SensorCalibration));
            nvs_commit(handle);
        } else {
            ESP_LOGI("FC", "NVS에서 캘리브레이션 로드 완료.");
        }
        nvs_close(handle);
    }
}
}