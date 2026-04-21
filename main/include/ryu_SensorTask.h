#include <array>


namespace SENSOR{


/**
 * @brief 
 *      1. 공유될데이터
 * 
 */
struct SensorSharedData{
    struct IMU{
        std::array<float,3> acc1;
        std::array<float,3> gyro1;
        std::array<float,3> acc2;
        std::array<float,3> gyro2;
        std::array<float,3> filtered_acc;
        std::array<float,3> filtered_gyro;
        uint64_t timestamp;
    }imu;
    struct MAG{
        std::array<float,3> mag1;
        std::array<float,3> mag2;
        std::array<float,3> filtered_mag;
        uint64_t timestamp;
    }mag;
    struct BARO{
        float pressure1;
        float pressure2;
        float filetred_pressure;
        uint64_t timestamp;
    } baro;
};


extern void setup_filters();


}//namespace SENSOR
