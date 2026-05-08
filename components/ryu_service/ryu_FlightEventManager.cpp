/**
 * @file ryu_FlightEventManager.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "ryu_FlightEventManager.h"
#include <esp_log.h>
#include "ryu_config.h"
#include "ryu_mavlink.h"

namespace Event{
    ESP_EVENT_DEFINE_BASE(SYS_MODE_EVENT_BASE);
}

namespace Service{

esp_err_t FlightEventManager::initialize() {
    if(_initialized) return ESP_OK;

    // 이벤트 핸들러 등록:    
    esp_err_t err = esp_event_handler_instance_register(Event::SYS_MODE_EVENT_BASE, ESP_EVENT_ANY_ID,
                                               &FlightEventManager::event_handler, this, NULL);
    if (err != ESP_OK){
        ESP_LOGI(TAG,"esp_event_handler_instance_register failed.");
        return err;
    }
    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    
    return err; 
}

void FlightEventManager::event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    auto* obj = static_cast<FlightEventManager*>(arg);
    auto* fault = static_cast<Event::mode_change_event_t*>(data);

    switch(id){
        case Event::MODE_ARM:{
            g_sys.is_armed = true;
            Service::Mavlink::get_instance().send_status_text("Armming state");
            ESP_LOGI(TAG,"ARMING.................");
            break;
        }
        case Event::MODE_DISARM:{
             g_sys.is_armed = false;
            Service::Mavlink::get_instance().send_status_text("DisArmming state");
            ESP_LOGI(TAG,"DISARMING.................");
            break;
        }
    } // switch(id)
}



}// namespace Service