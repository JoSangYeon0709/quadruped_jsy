#ifndef VETTAR_INTERFACE_PCA9685_HPP
#define VETTAR_INTERFACE_PCA9685_HPP

#include <cstdint>
#include <string>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

// I2C includes
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace vettar_interface
{

class PCA9685 {
public:
    // 시뮬레이션 모드 파라미터 추가
    PCA9685(int bus_number = 1, uint8_t address = 0x40, uint32_t reference_clock_speed = 25000000);
    ~PCA9685();
    
    void reset();
    void set_frequency(uint16_t freq_hz);
    void set_pwm(uint8_t channel, uint16_t on, uint16_t off);
    // 매개변수 타입을 double로 변경
    void set_pulse_width(uint8_t channel, double pulse_width_us);

private:
    void write_register(uint8_t reg, uint8_t value);
    uint8_t read_register(uint8_t reg);
    
    int fd_ = -1;
    int bus_number_;
    uint8_t address_;
    uint32_t reference_clock_speed_ = 25000000;
    uint16_t frequency_ = 50;
};

}  // namespace vettar_interface

#endif  // VETTAR_INTERFACE_PCA9685_HPP