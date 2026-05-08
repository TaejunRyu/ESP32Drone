#include "ryu_nvs.h"

namespace Service{

esp_err_t Service::NvsManager::initialize(const char *namespace_name)
{
        if (_initialized) return ESP_OK;
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        esp_err_t err = nvs_open(namespace_name, NVS_READWRITE, &handle);

        _initialized = true;
        return  err;
}

esp_err_t NvsManager::setInt(const char *key, int32_t value)
{
    std::lock_guard<std::mutex> lock(mtx);
    esp_err_t err = nvs_set_i32(handle, key, value);
    if (err == ESP_OK) nvs_commit(handle);
    return err;
}

int32_t NvsManager::getInt(const char *key, int32_t default_val)
{
    std::lock_guard<std::mutex> lock(mtx);
    int32_t value = default_val;
    nvs_get_i32(handle, key, &value);
    return value;
}

esp_err_t NvsManager::setString(const char *key, const std::string &value)
 {
    std::lock_guard<std::mutex> lock(mtx);
    esp_err_t err = nvs_set_str(handle, key, value.c_str());
    if (err == ESP_OK) nvs_commit(handle);
    return err;
}
std::string NvsManager::getString(const char *key)
 {
    std::lock_guard<std::mutex> lock(mtx);
    size_t required_size;
    if (nvs_get_str(handle, key, nullptr, &required_size) != ESP_OK) return "";
    
    char* buf = new char[required_size];
    nvs_get_str(handle, key, buf, &required_size);
    std::string res(buf);
    delete[] buf;
    return res;
}


} // namespace Service