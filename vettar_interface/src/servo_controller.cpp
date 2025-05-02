#include "vettar_interface/servo_controller.hpp"
#include <cmath>

namespace vettar_interface
{

ServoController::ServoController(std::shared_ptr<PCA9685> pca, uint8_t channel)
    : pca_(pca), channel_(channel), min_pulse_(500), max_pulse_(2500), current_angle_(M_PI/2) {
    // 초기화 시에는 서보 모터를 직접 제어하지 않음
    // 첫 명령이 들어올 때까지 대기
}

void ServoController::set_angle(double angle_radians) {
    // 각도 제한 (0-π 라디안 범위를 가짐)
    angle_radians = std::max(0.0, std::min(M_PI, angle_radians));

    // 라디안을 각도로 변환 (0-180)
    double angle_degrees = angle_radians * 180.0 / M_PI;
    
    // 파이썬 adafruit_servo 라이브러리와 정확히 동일한 방식으로 펄스 계산
    // pulse = self._min_pulse + (angle / 180) * (self._max_pulse - self._min_pulse)
    // 반올림이나 버림을 적용하지 않고 그대로 floating point 계산
    double pulse = min_pulse_ + (angle_degrees / 180.0) * (max_pulse_ - min_pulse_);
    
    // 펄스 설정
    pca_->set_pulse_width(channel_, pulse);
    
    // 현재 각도 저장
    current_angle_ = angle_radians;
}

// 서보 펄스 범위 설정
void ServoController::set_pulse_range(int min_pulse, int max_pulse) {
    min_pulse_ = min_pulse;
    max_pulse_ = max_pulse;
    
    // 현재 각도는 업데이트하지만 서보는 움직이지 않도록 함
    current_angle_ = M_PI/2.0;  // 90도로 설정만 하고 실제 모터는 움직이지 않음
}

} // namespace vettar_interface