#include "vettar_interface/pca9685.hpp"
#include <stdexcept>
#include <cmath>

namespace vettar_interface {

PCA9685::PCA9685(int bus_number, uint8_t address, uint32_t reference_clock_speed)
    : bus_number_(bus_number), address_(address), 
      reference_clock_speed_(reference_clock_speed), frequency_(50) {  // 기본 주파수 50Hz 초기화
    
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", bus_number);
    
    fd_ = open(filename, O_RDWR);
    if (fd_ < 0) {
        std::string error_msg = "Failed to open I2C bus: " + std::string(filename);
        RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "%s", error_msg.c_str());
        throw std::runtime_error(error_msg);
    }
    
    if (ioctl(fd_, I2C_SLAVE, address) < 0) {
        std::string error_msg = "Failed to acquire bus access or talk to slave at address " + std::to_string(address);
        RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "%s", error_msg.c_str());
        close(fd_);
        fd_ = -1;
        throw std::runtime_error(error_msg);
    }
    
    // Initialize PCA9685
    reset();
    set_frequency(frequency_);  // 기본 주파수로 초기화
}

PCA9685::~PCA9685() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

void PCA9685::reset() {
    if (fd_ < 0) {
        throw std::runtime_error("Invalid file descriptor in reset()");
    }
    write_register(0x00, 0x00);  // MODE1 register, reset
}

void PCA9685::set_frequency(uint16_t freq_hz) {
    if (fd_ < 0) {
        throw std::runtime_error("Invalid file descriptor in set_frequency()");
    }
    frequency_ = freq_hz;  // 멤버 변수에 주파수 저장
    
    // 파이썬 방식으로 prescale 값 계산 - 정확히 adafruit_pca9685.py와 동일하게
    // prescale = int(self.reference_clock_speed / 4096.0 / freq + 0.5) - 1
    double prescale_float = reference_clock_speed_ / 4096.0 / freq_hz + 0.5;
    uint16_t prescale_value = static_cast<uint16_t>(prescale_float) - 1;
    
    uint8_t oldmode = read_register(0x00);
    write_register(0x00, (oldmode & 0x7F) | 0x10); // Sleep 모드로 전환
    write_register(0xFE, prescale_value);          // prescale 값 설정
    write_register(0x00, oldmode);                 // 이전 모드로 복원
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    // autoincrement 활성화 (파이썬 코드와 동일하게)
    write_register(0x00, oldmode | 0xA0);  // 0xA0 = 0x80 (auto-increment) | 0x20 (auto-increment on stop)
}

void PCA9685::set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (fd_ < 0) {
        throw std::runtime_error("Invalid file descriptor in set_pwm()");
    }
    write_register(0x06 + 4 * channel, on & 0xFF);
    write_register(0x07 + 4 * channel, on >> 8);
    write_register(0x08 + 4 * channel, off & 0xFF);
    write_register(0x09 + 4 * channel, off >> 8);
}

void PCA9685::set_pulse_width(uint8_t channel, double pulse_width_us) {
    if (fd_ < 0) {
        throw std::runtime_error("Invalid file descriptor in set_pulse_width()");
    }
    // 정확한 파이썬 adafruit_motor.servo 방식으로 구현
    // 1. min_duty와 max_duty 계산
    // self._min_duty = int((min_pulse * self._pwm_out.frequency) / 1000000 * 0xFFFF)
    double min_duty_float = (pulse_width_us * frequency_) / 1000000.0 * 0xFFFF;
    
    // 파이썬의 int()는 버림(truncate)을 수행
    uint16_t duty_cycle = static_cast<uint16_t>(min_duty_float);
    
    // 특별 케이스 처리
    uint16_t final_on = 0;
    uint16_t final_off = 0;
    
    if (duty_cycle >= 0xFFFF) {
        // 완전히 켜짐 특수 케이스
        final_on = 0x1000;
        final_off = 0;
    } else if (duty_cycle < 0x0010) {
        // 완전히 꺼짐 특수 케이스
        final_on = 0;
        final_off = 0x1000;
    } else {
        // 일반적인 경우: 16비트에서 12비트로 변환 (>> 4)
        uint16_t value = duty_cycle >> 4;
        final_on = 0;
        final_off = value;
    }
    
    set_pwm(channel, final_on, final_off);
}

void PCA9685::write_register(uint8_t reg, uint8_t value) {
    if (fd_ < 0) {
        throw std::runtime_error("Invalid file descriptor in write_register()");
    }
    uint8_t buffer[2] = {reg, value};
    if (write(fd_, buffer, 2) != 2) {
        std::string error_msg = "Failed to write to I2C device register " + std::to_string(reg);
        RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "%s", error_msg.c_str());
        throw std::runtime_error(error_msg);
    }
}

uint8_t PCA9685::read_register(uint8_t reg) {
    if (fd_ < 0) {
        throw std::runtime_error("Invalid file descriptor in read_register()");
    }
    if (write(fd_, &reg, 1) != 1) {
        std::string error_msg = "Failed to write register to I2C device";
        RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "%s", error_msg.c_str());
        throw std::runtime_error(error_msg);
    }
    
    uint8_t value;
    if (read(fd_, &value, 1) != 1) {
        std::string error_msg = "Failed to read from I2C device";
        RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "%s", error_msg.c_str());
        throw std::runtime_error(error_msg);
    }
    
    return value;
}

}  // namespace vettar_interface