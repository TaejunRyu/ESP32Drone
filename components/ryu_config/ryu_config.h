#pragma once
#include <array>
#include <string_view>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <lwip/sockets.h>   // ESP-IDF 전용 소켓 라이브러리
#include <esp_adc/adc_oneshot.h>
#include <c_library_v2/common/mavlink.h>
//#include "ryu_common_std.h"
#include "ryu_bmp388.h"
//#include "error_proc.h"

#ifdef __cplusplus
extern "C" {
#endif
inline constexpr char TAG[] = "ESP32DRONE";
inline constexpr uint8_t SYSTEM_ID      = 1;
inline constexpr uint8_t COMPONENT_ID   = 1;

inline constexpr  float RAD_TO_DEG = 180.0f/M_PI;
inline constexpr  float DEG_TO_RAD = M_PI/180.0f;

inline constexpr uint8_t MAIN = 0;
inline constexpr uint8_t SUB  = 1; 

inline constexpr uint8_t  X  =  0;
inline constexpr uint8_t  Y  =  1;
inline constexpr uint8_t  Z  =  2;

inline constexpr uint8_t  FL =  0;
inline constexpr uint8_t  FR =  1;
inline constexpr uint8_t  RL =  2;
inline constexpr uint8_t  RR =  3;


// 현재 사용하는 센서 이름을 상수로 정의하여 코드 가독성 향상
inline constexpr char IMU_NAME_MAIN[]     ="ICM20948";
inline constexpr char IMU_NAME_SUB[]      ="ICM20948";
inline constexpr char MAG_NAME_MAIN[]     ="IST8310";
inline constexpr char MAG_NAME_SUB[]      ="AK09916";
inline constexpr char BARO_NAME_MAIN[]    ="BMP388";
inline constexpr char BARO_NAME_SUB[]     ="BMP388";


inline constexpr uint32_t   I2C_SPEED   = 400000;
inline constexpr uint32_t   GPS_UART_BAUD_RATE = 115200;

inline constexpr bool USE_BMP388_SPI = false;  // true로 설정하면 BMP388 SPI 경로 사용
inline constexpr gpio_num_t  VSPI_SCLK   = GPIO_NUM_18;
inline constexpr gpio_num_t  VSPI_MISO   = GPIO_NUM_19;
inline constexpr gpio_num_t  VSPI_MOSI   = GPIO_NUM_23;
// 개별 CS 핀 (센서마다 하나씩)
inline constexpr gpio_num_t  VSPI_CS1    = GPIO_NUM_5;  // ICM20948 #1
inline constexpr gpio_num_t  VSPI_CS2    = GPIO_NUM_25; // ICM20948 #2
inline constexpr gpio_num_t  VSPI_CS3    = GPIO_NUM_15; // BMP388 SPI CS

//남은핀.
//12번(MTDI)은 부팅 시 내부 Flash 전압을 결정합니다. 
//SCLK 라인에 연결된 SD 카드의 저항 성분 때문에 전압이 꼬여서 부팅이 안 될 수 있습니다.
//해결: idf.py menuconfig → Serial flasher config → Flash voltage (3.3V)로 강제 고정하세요. (이렇게 하면 12번 핀의 전압과 상관없이 정상 부팅됩니다.)
inline constexpr gpio_num_t  HSPI_SCLK    = GPIO_NUM_12; //  MTDI핀
inline constexpr gpio_num_t  HSPI_MISO    = GPIO_NUM_15; // 일반핀.
inline constexpr gpio_num_t  HSPI_MOSI    = GPIO_NUM_13; //  JTAG핀
//2 상황: 부팅 시 이 핀은 Low여야 합니다.
//체크: 보통 SD 카드 모듈의 CS 핀에는 10kΩ 정도의 풀업 저항이 달려 있어 부팅 시 High가 되기 쉽습니다.
//해결: 만약 전원을 켰는데 아무 반응이 없다면, 
//2번 핀을 Low로 살짝 잡아주거나 부팅 후에만 CS로 동작하게 회로를 점검해야 합니다. (정 안 되면 14번 핀과 바꾸는 게 상책입니다.)
inline constexpr gpio_num_t  HSPI_CS      = GPIO_NUM_2;  // 부팅관련
inline constexpr gpio_num_t  T1    = GPIO_NUM_0;  // 부팅관련
  


inline constexpr gpio_num_t BUZZER_GPIO = GPIO_NUM_14;  // 부저가 연결된 GPIO 번호

inline constexpr gpio_num_t FLYSKY_PPM_PIN = GPIO_NUM_4; // PPM으로 데이터를 보낼때

inline constexpr gpio_num_t GPS_RX      = GPIO_NUM_16;  // UART1 
inline constexpr gpio_num_t GPS_TX      = GPIO_NUM_17;
inline constexpr gpio_num_t I2C_SDA     = GPIO_NUM_21; // 숫자에 직접 타입을 지정
inline constexpr gpio_num_t I2C_SCL     = GPIO_NUM_22;


// FC가 공중 운반물이 있을경우 떨어뜨리는 SERVO MOTOR 
//inline constexpr  gpio_num_t   SERVO_MOTOR_PIN   = GPIO_NUM_25;
inline constexpr  gpio_num_t   MOTOR_FRONT_LEFT  = GPIO_NUM_26;
inline constexpr  gpio_num_t   MOTOR_FRONT_RIGHT = GPIO_NUM_27;
inline constexpr  gpio_num_t   MOTOR_REAR_LEFT   = GPIO_NUM_32;
inline constexpr  gpio_num_t   MOTOR_REAR_RIGHT  = GPIO_NUM_33;
inline constexpr gpio_num_t    BATTERY_ADC_PIN   = GPIO_NUM_34;
inline constexpr adc_channel_t BATTERY_ADC_CH    = ADC_CHANNEL_6;




// --- Sensors 핸들 및 전역 변수 ---
extern i2c_master_bus_handle_t i2c_handle;
extern i2c_master_dev_handle_t imu_handle[2]; 
extern i2c_master_dev_handle_t mag_handle[2];
extern class CBMP388 cbmp388_main;
extern class CBMP388 cbmp388_sub;

//==============================================
// 드론의 현재 상태를 저장하는 구조체 
// //==============================================
// MAV_STATE_ (system_status) 주요 값
// 값 (Enum)	상태                    의미 및 QGC 반응
// 0	        MAV_STATE_UNINIT	    시스템 초기화 중 (센서 체크 전)
// 1	        MAV_STATE_BOOT	        부팅 완료, 시스템 검사 시작
// 2	        MAV_STATE_CALIBRATING	자이로/지자기 센서 교정 중 (QGC에 메시지 표시)
// 3	        MAV_STATE_STANDBY	    시동 전 대기 상태. 모든 시스템 정상, Arming 가능
// 4	        MAV_STATE_ACTIVE	    시동 완료 및 비행 중. 모터가 돌고 있는 상태
// 5	        MAV_STATE_CRITICAL	    긴급 상황. 통신 두절이나 배터리 부족 시 발생
// 6	        MAV_STATE_EMERGENCY	    위험 상황. 기체 추락 위험, 즉각적인 조치 필요
// 7	        MAV_STATE_POWEROFF	    시스템 종료 중
// 8	        MAV_STATE_FLIGHT_TERMINATION	강제 비행 종료 (낙하산 전개 등 최후 수단)

// 기체 시동 상태 정의
typedef enum {
    ARM_STATE_DISARMED = 0,
    ARM_STATE_PREARM_CHECK,
    ARM_STATE_ARMED,
    ARM_STATE_FAILSAFE
} arming_state_t;

typedef enum {
    MODE_MANUAL = 0,      // 수동
    MODE_STABILIZED,      // Stabilized
    MODE_ACRO,            // Acro
    MODE_ALTCTL,          // 고도 (Altitude)
    MODE_OFFBOARD,        // 오프보드
    MODE_POSCTL,          // 위치 (Position)
    MODE_LOITER,          // 대기 (Hold)
    MODE_MISSION,         // 미션 (Auto)
    MODE_RTL,             // 복귀 (Return)
    MODE_PRECISION_LAND   // 정밀 착륙
} flight_mode_t;

typedef struct sys_t {
    flight_mode_t  flight_mode;     // 현재 비행 모드 => 이건 아직 미정 그냥 qgc와 연계하기 위하여 정의 
    uint8_t     system_status;      // standby(3), active(4), critical 등
    uint32_t    system_health;      // 현재 시스템의 상태 error_proc.h에서 주로 사용
    bool        is_armed;           // 시동 상태
    bool        manual_hold_mode;   // flysky controller에서 hold mode 지정
    bool        error_hold_mode;    // 센서의 오류로 인한 고정 비행
    bool        gps_ready;          // GPS 수신 준비 완료 (이것이 필요할까?) 
    bool        payload_dropped;    // 투하 완료 여부
    float       battery_voltage;    // 배터리 전압 ( 바로 구할수 있는데 필요할까 ?)
    uint32_t    loop_count;         // 비행 루프 카운터 (이게 왜 필요하지 ?)
} sys_t;
extern sys_t g_sys;


// --- 자세 및 IMU 데이터 ---
// 센서에서 생성되어지는 roll,pitch,yaw
typedef struct attitude_data_t{
    float   roll; 
    float   pitch;
    float   yaw;
    uint16_t heading;
    float   rollspeed;
    float   pitchspeed;
    float   yawspeed;
} attitude_data_t;
extern attitude_data_t g_attitude;






// QGC에 보내는 PID 디버그 데이터 구조체
// QGC에서 Roll PID 튜닝을 위해 사용 (실제 제어에는 사용 안 함)
typedef struct qgc_roll_pid_t {
    float target;
    float current;
    float output; 
} qgc_roll_pid_t;

extern qgc_roll_pid_t qgc_roll_pid;


// PID 구조체
// 프레임워크 전반에서 공유되므로 연속적인 초기화와 링크를 위해 inline 선언합니다.
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float err_prev;  // Outer Loop(각도)용: 이전 오차 저장
    float prev_rate; // Inner Loop(각속도)용: 이전 자이로 값 저장
} drone_pid_t;

// PID 구조체 초기화 (기본값 0)
// 제어기 명칭	       역할	    P (Proportional)	                I (Integral)	D (Derivative)	비고
// Alt Position     (Outer)	    목표 고도 유지	1.0 ~ 2.0	        0.0	            0.0	단위:       (m) -> (m/s) 변환
// Alt Rate         (Inner)	    상승/하강 속도 제어	50.0 ~ 100.0	 10.0 ~ 20.0	 0.05	        단위: (m/s) -> PWM 변량
extern drone_pid_t pid_alt_pos;     //목표고도유지 
extern drone_pid_t pid_alt_rate;    //상승/하강 속도제어

// 1. 각도 제어용 (Outer Loop) - P값 위주
// PID 구조체 초기값 (추천 가이드)
// 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
// Angle (Outer)	    각도 유지	    4.5	                0.0	            0.0	            오직 P값만 사용해도
extern drone_pid_t pid_roll_angle;  //각도유지
extern drone_pid_t pid_pitch_angle;
extern drone_pid_t pid_yaw_angle;

// 2. 각속도 제어용 (Inner Loop) - 실제 기체 반응 결정
// 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
// Rate (Inner)	        진동/회전 제어	0.15	            0.1	            0.003	        가장 정밀하게 튜닝 필요
// Yaw Rate	회전        속도 제어	    0.20	            0.05	        0.0	            값은 거의 사용 안 함
extern drone_pid_t pid_roll_rate;   //진동/회전 제어
extern drone_pid_t pid_pitch_rate;
extern drone_pid_t pid_yaw_rate;    // 회전 속도제어


// --- RC 송수신기 데이터 ---
typedef struct rc_data_t {
    volatile float throttle;  // 스로틀 (0~100%)
    volatile float roll;      // 롤 (-100~100)
    volatile float pitch;     // 피치 (-100~100)
    volatile float yaw;       // 요 (-100~100)
    volatile float aux1;      // 보조 채널 1 (고도 유지)
    volatile float aux2;      // 보조 채널 2
    volatile float aux3;      // 보조 채널 3 (SWC 3단)
    volatile float aux4;      // 보조 채널 4
} rc_data_t;
extern rc_data_t g_rc;  ///< 조종기 입력 데이터


// --- GPS 데이터 ---
typedef struct gps_data_t {
    uint32_t    iTOW;           // gps 시간
    int         date;           // 기본 날자
    float       utc_time;       // 기본 시간 
    double      lat;            // 위도 (deg)
    double      lon;            // 경도 (deg)
    float       alt;            // 상대 고도 (홈 기준)
    float       home_alt;       // 처음 시스템 시작시(gps가동시)의 고도  
    uint8_t     sats;           // 위성 개수
    uint16_t    pDOP;           // 위치 정밀도 저하율 (0.01 단위)
    uint8_t     fixType;        // GPS 위치 고정 여부
    int16_t     velN;           // (North Velocity),  GPS정보  VGT 정보에서 뽑아야한다. 
    int16_t     velE;           // (East Velocity), 
    int16_t     velD;           // (Down Velocity)
    uint16_t    gSpeed;      // 지표 속도 (mm/s)
    uint16_t    headMot;        // 이동 방향 (Degree * 10^-5 -> Centi-Degree)
    float       magDec ; // 자기 편차 적용 (필요 시)
    uint32_t    hAcc; 
    uint32_t    vAcc; 
    uint32_t    sAcc;
    int32_t     height;         // 타원체 고도 (mm)    
    int32_t     hMSL;           // 해수면 고도 (mm)
    TickType_t  last_update_tick;
} gps_data_t;
extern gps_data_t g_gps;  ///< GPS 위치 데이터


// --- 고도 데이터 ---
typedef struct altitude_data_t {
    float current;   // 현재 고도 (기압계 기준, m)
    float home;      // 홈 고도 (초기 기압계 값)
    float target;    // 목표 고도 (고도 유지 모드)
} altitude_data_t;
extern altitude_data_t g_altitude;  ///< 고도 관련 데이터

// 현재의 gps위치나 QGC에서 보내온 위치를 홈의 위치로 설정
typedef struct qgc_home_pos_t {
    double lat;
    double lon;
    double alt;
    bool   is_set;
} qgc_home_pos_t;
extern qgc_home_pos_t qgc_home_pos;



// --- 배터리 데이터 ---
typedef struct battery_data_t {
    float voltage;   // 현재 전압 (V)
    float percentage; // 배터리 백분율 (%)
    bool is_low;               // 저전압 경고 상태
} battery_data_t;
extern battery_data_t g_battery;  ///< 배터리 데이터


// --- 제어 목표값 (flight_task 로컬 정적 변수) ---
// 주의: 이들은 flight_task 내부 정적 변수로 유지되므로 외부 접근 불가
// 필요할 경우 아래에 추가
// static float target_yaw_angle;
// static float target_alt;
// static bool last_alt_hold_state;


// --- PID 디버그 데이터 (QGC 텔레메트리용) ---
typedef struct pid_debug_t {
    float target;  // 목표값
    float current; // 현재값
    float output;  // PID 출력
} pid_debug_t;

extern pid_debug_t g_roll_pid;   ///< 롤 PID 디버그 데이터
extern pid_debug_t g_pitch_pid;  ///< 피치 PID 디버그 데이터
extern pid_debug_t g_yaw_pid;    ///< 요 PID 디버그 데이터
extern pid_debug_t g_alt_pid;    ///< 고도 PID 디버그 데이터

// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 시스템별 사용자 지정 모드가 활성화됩니다. 이 플래그를 사용하여 사용자 지정 모드를 활성화할 경우 다른 모든 플래그는 무시해야 합니다. | */
// MAV_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 시스템에 테스트 모드가 활성화되었습니다. 이 플래그는 임시 시스템 테스트를 위한 것이며 안정적인 구현에는 사용해서는 안 됩니다. | */
// MAV_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 자율 비행 모드가 활성화되어 시스템이 자체적으로 목표 위치를 찾습니다. 유도 플래그는 실제 구현에 따라 설정하거나 설정하지 않을 수 있습니다. | */
// MAV_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 유도 비행 모드가 활성화되어 시스템이 웨이포인트/임무 항목을 따라 비행합니다. | */
// MAV_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 시스템이 전자적으로 자세(및 선택적으로 위치)를 안정화합니다. 하지만 이동하려면 추가 제어 입력이 필요합니다. | */
// MAV_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 하드웨어 인 더 루프 시뮬레이션. 모든 모터/액추에이터는 차단되지만 내부 소프트웨어는 완전히 작동합니다. | */
// MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64,/* 0b01000000 원격 제어 입력이 활성화되었습니다. | */
// MAV_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV 안전 장치가 활성화되었습니다. 모터가 활성화/작동 중/시동 가능합니다. 비행 준비 완료. 추가 참고: 이 플래그는 MAV_CMD_DO_SET_MODE 명령과 함께 전송될 때 무시되어야 하며, 대신 MAV_CMD_COMPONENT_ARM_DISARM을 사용해야 합니다. 하지만 이 플래그는 무장 상태를 보고하는 데에는 여전히 사용할 수 있습니다. | */
// MAV_MODE_FLAG_ENUM_END=129, /* | */


// manual : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
// alt_hold : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED
// pos_hold : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED
// acro : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
// stabilize : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED
//             main_mode   custom_mode
// manual      1           0x00010000(65536)
// alt_hold    2           0x00020000(131072)
// pos_hold    3           0x00030000(196608)
// acro        5           0x00050000(327680)
// stabilize   7           0x00070000(458752)
typedef struct heartbeat_t{
    uint8_t     base_mode;
    uint32_t    custom_mode;
}heartbeat_t;
extern heartbeat_t g_heartbeat;


/*현재 flight_task에서 imu 데이터를 읽어서 여기에 보관하고 작업능 시행 current_data*/
// 실제로 가지고 있어야하나 
typedef struct imu_data_t {
    std::array<float,3> acc;  // 가속도
    std::array<float,3> gyro; // 자이로
    std::array<float,3> mag;  // 지자계
} imu_data_t;
extern imu_data_t g_imu;  ///< IMU 원본 데이터 (가속도도, 자이로, 지자계)


// imu main,sub의 offset 저장 보관.
typedef struct imu_offset_t{
    std::array<float,3> acc;
    std::array<float,3> gyro; 
}imu_offset_t;
extern imu_offset_t g_imu_offset[2];



// 2. 기압 및 고도 데이터
typedef struct baro_t{
    float pressure;            // 현재 기압 (hPa)
    float raw_altitude;        // 필터 전 고도 (m)
    float filtered_altitude;   // LPF 적용 후 고도 (m)
    float ground_pressure;     // 이륙 지점 기준 기압
} baro_t;
extern baro_t g_baro;


// typedef struct {
//     // 1. IMU 데이터 (자이로, 가속도, 기울기)
//     struct {
//         float roll, pitch, yaw;    // 최종 계산된 각도 (degree)
//         float gx, gy, gz;          // 자이로 원시 데이터
//         float ax, ay, az;          // 가속도 원시 데이터
//     } imu;

//     // 2. 기압 및 고도 데이터
//     struct {
//         float pressure;            // 현재 기압 (hPa)
//         float raw_altitude;        // 필터 전 고도 (m)
//         float filtered_altitude;   // LPF 적용 후 고도 (m)
//         float ground_pressure;     // 이륙 지점 기준 기압
//     } baro;

//     // 3. GPS 데이터
//     struct {
//         double lat, lon;           // 현재 위도, 경도
//         float distance_to_target;  // 목표지점까지의 거리 (m)
//         bool is_gps_fixed;         // GPS 수신 상태
//     } gps;

//     // 4. 시스템 상태
//     bool is_armed;                 // 모터 가동 상태
//     bool payload_dropped;          // 투하 완료 여부
//     float battery_voltage;         // 배터리 전압
// } drone_status_t;

// // 전역 변수 선언
// drone_status_t g_drone;


// typedef struct {
//     // 1. 목표 지점 정보 (Target)
//     struct {
//         double lat, lon;           // 목표 위도, 경도
//         float target_alt;          // 투하 목표 고도 (예: 3.0m)
//         float dist_to_target;      // 현재 위치에서 목표까지의 거리 (m)
//         bool arrived;              // 목표 지점 도달 여부
//     } target;

//     // 2. 기압 및 고도 (Barometer)
//     struct {
//         float pressure;            // 현재 기압 (hPa)
//         float ground_pressure;     // 이륙 시 저장된 지면 기압
//         float rel_alt;             // 상대 고도 (m)
//         float filtered_alt;        // LPF 적용된 안정된 고도 (m)
//     } baro;

//     // 3. 전원 상태 (Power)
//     struct {
//         float voltage;             // 현재 배터리 전압 (V)
//         float percentage;          // 남은 잔량 (%)
//         bool is_low;               // 저전압 경고 상태
//     } battery;

//     // 4. 시스템 플래그 (Status)
//     bool is_armed;                 // 모터 가동 여부
//     bool payload_dropped;          // 낚시 도구 투하 완료 여부
//     int lpf_alpha;                 // 필터 계수 (가변 조정용)
// } drone_status_t;

// // 전역 변수 생성
// drone_status_t g_drone = {0};

#ifdef __cplusplus
}
#endif
