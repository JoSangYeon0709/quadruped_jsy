cmake_minimum_required(VERSION 3.8)
project(vettar_interface)

# 컴파일러 옵션 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)  # YAML 라이브러리 추가
find_package(ament_index_cpp REQUIRED)  # ament_get_share_directory를 위해 추가
find_package(std_msgs REQUIRED)  # simulation_mode 노드에서 필요
find_package(std_srvs REQUIRED)

# 헤더 폴더 설정
include_directories(include)

# 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/pca9685.cpp
  src/servo_controller.cpp
  src/vettar_hardware_interface.cpp
)

# 실행파일 생성
add_executable(simulation_mode src/simulation_mode.cpp)

ament_target_dependencies(simulation_mode
  rclcpp
  std_msgs
  std_srvs
)
# 대상 링크 라이브러리
target_link_libraries(${PROJECT_NAME}
  yaml-cpp  # YAML 라이브러리 링크
)

# ament 종속성 추가
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
  std_msgs
  std_srvs
)

# 플러그인 등록
pluginlib_export_plugin_description_file(hardware_interface vettar_hardware_interface.xml)

# 설치 대상
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 실행파일 설치
install(
  TARGETS simulation_mode
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(
  DIRECTORY include/
  DESTINATION include
)

# 기타 파일 설치
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2 플러그인 선언 파일 설치
install(
  FILES vettar_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# 패키지 내보내기
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml-cpp
  ament_index_cpp
  std_msgs
  std_srvs
)

ament_package()
