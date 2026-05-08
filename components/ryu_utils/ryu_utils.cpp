#include "ryu_utils.h"

namespace Utils{
/**
 * @brief 
 *      1. value값이 zone에 들어오면 0.0f로 처리  
 * @param value 
 * @param zone 
 * @return float 
 */
float Apply_DeadZone(float value, float zone)
{
    return (std::abs(value) < zone) ? 0.0f : value;
}


}