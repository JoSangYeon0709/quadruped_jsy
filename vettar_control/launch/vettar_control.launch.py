from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_dir = get_package_share_directory('vettar_control')
    
    # 설정 파일 경로 지정
    config_file = os.path.join(pkg_dir, 'config', 'vettar_params.yaml')
    
    return LaunchDescription([
        # 파라미터 파일을 위한 런치 인자
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
        ),
        
        # IK 노드
        Node(
            package='vettar_control',
            executable='ik_node',
            name='ik_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
        
        # 보행 생성기 노드
        Node(
            package='vettar_control',
            executable='gait_generator',
            name='gait_generator',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        )
    ])