import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_share_directory = get_package_share_directory('vettar_description')
    urdf = os.path.join(package_share_directory, 'urdf', 'vettar_description.urdf')
    rviz_config = os.path.join(package_share_directory, 'rviz', 'rviz_set.rviz')
    
    # config_file = os.path.join(package_share_directory, 'config', 'joint_config.yaml')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',),
    
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'zeros.F_L_0': math.pi/2,
                'zeros.F_L_1': math.pi/2,
                'zeros.F_L_2': math.pi/2,
                'zeros.F_R_3': math.pi/2,
                'zeros.F_R_4': math.pi/2,
                'zeros.F_R_5': math.pi/2,
                'zeros.B_L_6': math.pi/2,
                'zeros.B_L_7': math.pi/2,
                'zeros.B_L_8': math.pi/2,
                'zeros.B_R_9': math.pi/2,
                'zeros.B_R_10': math.pi/2,
                'zeros.B_R_11': math.pi/2,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )
    ])



if __name__ == '__main__':
    generate_launch_description()
