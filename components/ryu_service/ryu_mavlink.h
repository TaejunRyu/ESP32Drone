#pragma once
#define MAVLINK_USE_MESSAGE_INFO

#include <c_library_v2/common/mavlink.h>
#include <c_library_v2/mavlink_get_info.h>

namespace Service
{
  
class Mavlink{
    private:
        Mavlink();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Mavlink(const Mavlink&) = delete;
        Mavlink& operator=(const Mavlink&) = delete;
        ~Mavlink();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Mavlink& get_instance() {
            static Mavlink* instance = new Mavlink(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
        void send_status_text(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
        void send_mavlink_msg(mavlink_message_t *msg);
        void send_mav_command_ack(uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_sysid, uint8_t target_compid);
        void handle_mavlink_message(mavlink_message_t *msg);

        void MAV_CMD_COMPONENT_ARM_DISARM_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_NAV_TAKEOFF_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_DO_SET_HOME_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_REQUEST_MESSAGE_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_PREFLIGHT_CALIBRATION_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_SET_MESSAGE_INTERVAL_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
        void MAV_CMD_REQUEST_PROTOCOL_VERSION_func(mavlink_message_t *msg, mavlink_command_long_t cmd);

         
        void on_timer_tick();
     
        void initialize();
    private:
        bool _initialized;
        static const char* TAG;

};



}