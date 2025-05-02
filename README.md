만드는중


ros2 launch vettar_interface vettar_interface.launch.py
혹은
ros2 launch vettar_interface vettar_interface.launch.py controller_type:=position
ros2 launch vettar_interface vettar_interface.launch.py controller_type:=trajectory
으로 컨트롤러 유형 선택가능.

ros2 launch vettar_control vettar_control.launch.py

ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

