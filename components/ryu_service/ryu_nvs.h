#pragma once

#include <string>
#include <mutex>
#include <nvs_flash.h>
#include <esp_log.h>

namespace Service{

class NvsManager {
public:
    // 싱글톤 인스턴스 반환 (Thread-safe)
    static NvsManager& get_instance() {
        static NvsManager instance;
        return instance;
    }

    // 복사 및 할당 방지
    NvsManager(const NvsManager&) = delete;
    NvsManager& operator=(const NvsManager&) = delete;

    // Nvs 초기화 및 네임스페이스 열기
    esp_err_t initialize(const char* namespace_name = "storage") {
        if (_initialized) return ESP_OK;
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        esp_err_t err nvs_open(namespace_name, NVS_READWRITE, &handle);

        _initialized = true;
        return  err;
    }

    // 정수형(int32_t) 데이터 저장 및 읽기
    esp_err_t setInt(const char* key, int32_t value) {
        std::lock_guard<std::mutex> lock(mtx);
        esp_err_t err = nvs_set_i32(handle, key, value);
        if (err == ESP_OK) nvs_commit(handle);
        return err;
    }

    int32_t getInt(const char* key, int32_t default_val = 0) {
        std::lock_guard<std::mutex> lock(mtx);
        int32_t value = default_val;
        nvs_get_i32(handle, key, &value);
        return value;
    }

    // 문자열 데이터 저장 및 읽기
    esp_err_t setString(const char* key, const std::string& value) {
        std::lock_guard<std::mutex> lock(mtx);
        esp_err_t err = nvs_set_str(handle, key, value.c_str());
        if (err == ESP_OK) nvs_commit(handle);
        return err;
    }

    std::string getString(const char* key) {
        std::lock_guard<std::mutex> lock(mtx);
        size_t required_size;
        if (nvs_get_str(handle, key, nullptr, &required_size) != ESP_OK) return "";
        
        char* buf = new char[required_size];
        nvs_get_str(handle, key, buf, &required_size);
        std::string res(buf);
        delete[] buf;
        return res;
    }

private:
    NvsManager() : handle(0) {} // 생성자 private
    nvs_handle_t handle;
    std::mutex mtx; // 멀티스레드 환경 대비
    bool _initialized = false;
};

} // namespace Service




// #include "NvsManager.hpp"

// extern "C" void app_main() {
//     auto& nvs = NvsManager::getInstance();
    
//     // 1. 초기화
//     if (nvs.init() == ESP_OK) {
//         ESP_LOGI("NVS", "NVS initialized successfully.");
        
//         // 2. 데이터 쓰기
//         nvs.setInt("boot_count", nvs.getInt("boot_count") + 1);
//         nvs.setString("dev_name", "ESP32-S3-V6.0");

//         // 3. 데이터 읽기
//         int32_t boots = nvs.getInt("boot_count");
//         std::string name = nvs.getString("dev_name");

//         ESP_LOGI("NVS", "Device: %s, Boot Count: %ld", name.c_str(), boots);
//     }
// }