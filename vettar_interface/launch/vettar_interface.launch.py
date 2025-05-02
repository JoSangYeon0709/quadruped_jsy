import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # Launch 인자 가져오기
    simulation_mode_raw = LaunchConfiguration('simulation_mode').perform(context)
    simulation_mode_bool = simulation_mode_raw.lower() in ['true', '1', 'yes']
    controller_type = LaunchConfiguration('controller_type').perform(context)

    # 패키지 경로
    description_pkg = get_package_share_directory('vettar_description')
    interface_pkg = get_package_share_directory('vettar_interface')

    # xacro → URDF
    xacro_file = os.path.join(description_pkg, 'urdf', 'vettar_description.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={
        'simulation_mode': simulation_mode_raw  # xacro는 문자열 전달
    })
    robot_description = {'robot_description': doc.toxml()}

    # 컨트롤러 설정 파일
    controller_config = os.path.join(interface_pkg, 'config', 'vettar_controllers.yaml')

    use_sim_time = simulation_mode_bool

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['servo_position_controller', '-c', '/controller_manager'],
            condition=LaunchConfigurationEquals('controller_type', 'position'),
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['servo_trajectory_controller', '-c', '/controller_manager'],
            condition=LaunchConfigurationEquals('controller_type', 'trajectory'),
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_type',
            default_value='position',
            description='Choose servo controller: position or trajectory'
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='false',
            description='Enable simulation mode and simulation time if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
