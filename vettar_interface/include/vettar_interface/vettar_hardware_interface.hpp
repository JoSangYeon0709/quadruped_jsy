#ifndef VETTAR_HARDWARE_INTERFACE_HPP
#define VETTAR_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// YAML 라이브러리 추가
#include <yaml-cpp/yaml.h>

// 다음 헤더 파일들 추가
#include "vettar_interface/pca9685.hpp"            // PCA9685 컨트롤러 헤더
#include "vettar_interface/servo_controller.hpp"   // ServoController 헤더

namespace vettar_interface
{

class VettarHardwareInterface : public hardware_interface::SystemInterface
{
public:
  VettarHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 서보 설정 구조체
  struct ServoConfig {
    int min_pulse;
    int max_pulse;
    bool is_reversed = false;
  };

  // 서보 모터와 관련된 멤버 변수
  std::vector<std::string> joint_names_;
  std::vector<std::string> special_joint_names_;  // 특수 조인트 이름 목록
  std::vector<std::string> all_joint_names_;      // 모든 조인트 이름 목록
  
  std::vector<double> servo_positions_;
  std::vector<double> servo_commands_;
  
  // 서보 펄스 범위 및 설정 관련 멤버 변수
  std::vector<ServoConfig> servo_configs_;
  
  // PCA9685 클록 속도 관련 멤버 변수
  uint32_t reference_clock_speed_ = 25000000;  // 기본값 25MHz
  
  // 서보 PWM 주파수 멤버 변수
  uint16_t servo_hz_ = 50;  // 기본값 50
  
  // PCA9685 컨트롤러
  std::shared_ptr<PCA9685> pca_;
  
  // 서보 컨트롤러 배열
  std::vector<std::shared_ptr<ServoController>> servos_;
  
  // 자동 초기화 이동 비활성화 플래그
  bool disable_auto_move_ = true;
  
  // 초기 명령 감지 플래그
  bool is_first_command_ = true;

  // YAML 설정 파일 로드 메서드
  bool load_servo_config(const std::string& config_path);

  // 특수 조인트 위치 계산 함수
  void calculate_special_joint_positions();
};

} // namespace vettar_interface

#endif // VETTAR_HARDWARE_INTERFACE_HPP