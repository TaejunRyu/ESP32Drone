#pragma once
#include <esp_event.h>

namespace Service {
    ESP_EVENT_DECLARE_BASE(SYS_FAULT_EVENT_BASE);

    enum e_fault_id {
        FAULT_ID_IMU,
        FAULT_ID_MAG,
        FAULT_ID_BARO,
        FAULT_ID_GPS,
        FAULT_ID_RC,
        FAULT_ID_BATTERY
    };

    typedef struct {
        e_fault_id id;
        bool is_recovered; // false=에러발생, true=복구됨
        esp_err_t reason;
    } fault_event_data_t;
}
