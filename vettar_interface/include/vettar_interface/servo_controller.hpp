#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <memory>
#include "vettar_interface/pca9685.hpp"

namespace vettar_interface
{

class ServoController {
public:
    // 생성자: PCA9685 인스턴스와 채널 번호를 받음
    ServoController(std::shared_ptr<PCA9685> pca, uint8_t channel);
    
    // 서보 각도 설정 (라디안 단위로 변경)
    void set_angle(double angle_radians);
    
    // 서보 펄스 범위 설정
    void set_pulse_range(int min_pulse, int max_pulse);

private:
    std::shared_ptr<PCA9685> pca_;  // PCA9685 인스턴스
    uint8_t channel_;               // 서보 채널 번호
    int min_pulse_;                 // 최소 펄스 폭
    int max_pulse_;                 // 최대 펄스 폭
    double current_angle_;          // 현재 서보 각도 (라디안)
};

} // namespace vettar_interface

#endif // SERVO_CONTROLLER_HPP