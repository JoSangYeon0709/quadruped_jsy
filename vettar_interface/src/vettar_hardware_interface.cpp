#include "vettar_interface/vettar_hardware_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <fstream>


namespace vettar_interface 
{

VettarHardwareInterface::VettarHardwareInterface() : disable_auto_move_(true), is_first_command_(true) {}

// YAML 설정 파일 로드 메서드 구현
bool VettarHardwareInterface::load_servo_config(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        // PCA9685 레퍼런스 클록 속도 로드
        if (config["pca9685"] && config["pca9685"]["reference_clock_speed"]) {
            reference_clock_speed_ = config["pca9685"]["reference_clock_speed"].as<uint32_t>();
            RCLCPP_INFO(rclcpp::get_logger("VettarHardwareInterface"), 
                        "PCA9685 Reference Clock Speed: %u Hz", reference_clock_speed_);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("VettarHardwareInterface"), 
                        "No reference clock speed found in config. Using default: %u Hz", reference_clock_speed_);
        }

        // 서보 주파수 로드
        if (config["pca9685"] && config["pca9685"]["servo_hz"]) {
            servo_hz_ = config["pca9685"]["servo_hz"].as<uint16_t>();
            RCLCPP_INFO(rclcpp::get_logger("VettarHardwareInterface"), 
                        "Servo PWM Frequency: %u Hz", servo_hz_);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("VettarHardwareInterface"), 
                        "No servo frequency found in config. Using default: %u Hz", servo_hz_);
        }

        // 서보 펄스 설정 로드
        if (!config["servo_pulses"]) {
            RCLCPP_ERROR(rclcpp::get_logger("VettarHardwareInterface"), 
                         "Invalid servo config file structure");
            return false;
        }

        // 서보 설정 초기화
        servo_configs_.clear();
        servo_configs_.resize(12);

        for (int i = 0; i < 12; ++i) {
            std::string channel_str = std::to_string(i);
            if (config["servo_pulses"][channel_str]) {
                servo_configs_[i].min_pulse = config["servo_pulses"][channel_str]["min"].as<int>();
                servo_configs_[i].max_pulse = config["servo_pulses"][channel_str]["max"].as<int>();
                
                // 최소값이 최대값보다 큰 경우 reversed로 간주
                servo_configs_[i].is_reversed = 
                    servo_configs_[i].min_pulse > servo_configs_[i].max_pulse;
            } else {
                RCLCPP_WARN(rclcpp::get_logger("VettarHardwareInterface"), 
                            "No config found for channel %d", i);
                return false;
            }
        }

        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("VettarHardwareInterface"), 
                     "Error parsing servo config: %s", e.what());
        return false;
    }
}

hardware_interface::CallbackReturn VettarHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 미리 정의된 조인트 이름 설정
    joint_names_ = {
        "F_L_0", "F_L_1", "F_L_2",  // 앞 왼쪽 다리
        "F_R_3", "F_R_4", "F_R_5",  // 앞 오른쪽 다리
        "B_L_6", "B_L_7", "B_L_8",  // 뒤 왼쪽 다리
        "B_R_9", "B_R_10", "B_R_11" // 뒤 오른쪽 다리
    };
    
    // 특수 조인트 이름 추가 (하드웨어가 실제로 제어하지 않는 계산된 조인트)
    special_joint_names_ = {
        "thighs_to_calf_F_L", "calf_ctrl_b_to_link_2_F_L",
        "thighs_to_calf_F_R", "calf_ctrl_b_to_link_2_F_R",
        "thighs_to_calf_B_L", "calf_ctrl_b_to_link_2_B_L",
        "thighs_to_calf_B_R", "calf_ctrl_b_to_link_2_B_R"
    };
    
    // 모든 조인트 이름 목록 (물리적 + 특수 조인트)
    all_joint_names_ = joint_names_;
    all_joint_names_.insert(all_joint_names_.end(), special_joint_names_.begin(), special_joint_names_.end());

    // 서보 설정 파일 로드
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("vettar_interface");
    std::string config_path = package_share_directory + "/config/servo_pulses.yaml";
    
    if (!load_servo_config(config_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("VettarHardwareInterface"), 
                     "Failed to load servo configuration");
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        // PCA9685 초기화
        pca_ = std::make_shared<PCA9685>(1, 0x40, reference_clock_speed_);
        
        // YAML에서 읽어온 주파수로 설정
        pca_->set_frequency(servo_hz_);

        // 서보 컨트롤러 초기화 및 펄스 범위 설정
        servos_.clear();
        for (uint8_t channel = 0; channel < 12; channel++) {
            auto servo_controller = std::make_shared<ServoController>(pca_, channel);
            
            // 각 서보에 min/max 펄스 설정
            servo_controller->set_pulse_range(
                servo_configs_[channel].min_pulse, 
                servo_configs_[channel].max_pulse
            );
            
            servos_.push_back(servo_controller);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("VettarHardwareInterface"), 
                     "Error initializing hardware: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 초기 위치 설정 (라디안)
    // 물리적 서보 + 특수 조인트를 위한 공간 확보
    uint total_joints = joint_names_.size() + special_joint_names_.size();
    servo_positions_.resize(total_joints, M_PI/2.0);  // 90도를 라디안으로 (π/2)
    servo_commands_.resize(joint_names_.size(), M_PI/2.0);  // 명령은 실제 서보에만 필요

    // 자동 초기화 이동 비활성화
    disable_auto_move_ = true;
    is_first_command_ = true;

    RCLCPP_INFO(rclcpp::get_logger("VettarHardwareInterface"), 
                "Servo Hardware Interface initialized with %lu physical and %lu special joints - Auto movement disabled",
                joint_names_.size(), special_joint_names_.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

// export_state_interfaces 메서드
std::vector<hardware_interface::StateInterface> VettarHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    // 모든 조인트(물리적 + 특수)에 대한 state interface 내보내기
    for (uint i = 0; i < all_joint_names_.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                all_joint_names_[i], 
                "position", 
                &servo_positions_[i]
            )
        );
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VettarHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // 물리적 조인트에 대해서만 command interface 내보내기
    for (uint i = 0; i < joint_names_.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint_names_[i], 
                "position", 
                &servo_commands_[i]
            )
        );
    }
    
    return command_interfaces;
}

// 특수 조인트의 위치 계산 함수
void VettarHardwareInterface::calculate_special_joint_positions() {
    // 특수 조인트들의 시작 인덱스 계산
    uint special_joint_start_idx = joint_names_.size();
    
    servo_positions_[special_joint_start_idx] = -servo_positions_[1] + servo_positions_[2];
    servo_positions_[special_joint_start_idx + 1] = servo_positions_[1] - servo_positions_[2];
    servo_positions_[special_joint_start_idx + 2] = -servo_positions_[4] + servo_positions_[5];
    servo_positions_[special_joint_start_idx + 3] = servo_positions_[4] - servo_positions_[5];
    servo_positions_[special_joint_start_idx + 4] = -servo_positions_[7] + servo_positions_[8];
    servo_positions_[special_joint_start_idx + 5] = servo_positions_[7] - servo_positions_[8];
    servo_positions_[special_joint_start_idx + 6] = -servo_positions_[10] + servo_positions_[11];
    servo_positions_[special_joint_start_idx + 7] = servo_positions_[10] - servo_positions_[11];
}

hardware_interface::CallbackReturn VettarHardwareInterface::on_activate(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("VettarHardwareInterface"), 
                "Starting robot - waiting for commands");
    
    // 자동 초기화 이동 비활성화 유지
    disable_auto_move_ = true;
    is_first_command_ = true;
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VettarHardwareInterface::on_deactivate(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("VettarHardwareInterface"), "Stopping robot");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VettarHardwareInterface::read(
    [[maybe_unused]] const rclcpp::Time& time, 
    [[maybe_unused]] const rclcpp::Duration& period) {
    // 여기서는 서보 모터에 위치 피드백이 없기 때문에 
    // 마지막으로 전송된 명령을 현재 위치로 간주
    for (uint i = 0; i < servo_commands_.size(); i++) {
        servo_positions_[i] = servo_commands_[i];
    }
    
    // 특수 조인트 위치 계산
    calculate_special_joint_positions();
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type VettarHardwareInterface::write(
    [[maybe_unused]] const rclcpp::Time& time, 
    [[maybe_unused]] const rclcpp::Duration& period) {
    // 첫 실제 명령 감지
    if (is_first_command_) {
        bool has_command = false;
        static bool reported = false;
        
        // 명령이 변경되었는지 확인 (첫 명령 감지)
        for (uint i = 0; i < servo_commands_.size(); i++) {
            // 기본값과 다른 명령이 들어왔는지 확인
            double diff = std::abs(servo_commands_[i] - M_PI/2.0);
            if (diff > 0.01) {  // 미세한 차이는 무시
                has_command = true;
                break;
            }
        }
        
        // 첫 실제 명령이 감지되면 상태 업데이트
        if (has_command) {
            if (!reported) {
                RCLCPP_INFO(rclcpp::get_logger("VettarHardwareInterface"), 
                           "First command detected - servos are now active");
                reported = true;
            }
            disable_auto_move_ = false;
            is_first_command_ = false;
        } else {
            // 초기 명령인 경우 모터 제어 스킵
            return hardware_interface::return_type::OK;
        }
    }
    
    // 자동 모드가 비활성화되지 않았으면 모터 제어 스킵
    if (disable_auto_move_) {
        return hardware_interface::return_type::OK;
    }
    
    // 실제 사용자 명령인 경우 모터 제어
    for (uint i = 0; i < servos_.size(); i++) {
        servos_[i]->set_angle(servo_commands_[i]);
    }
    
    return hardware_interface::return_type::OK;
}

} // namespace vettar_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(vettar_interface::VettarHardwareInterface, hardware_interface::SystemInterface)