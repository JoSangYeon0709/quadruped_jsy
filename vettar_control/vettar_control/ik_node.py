#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import inv, norm
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray  # 다리 위치 메시지 타입

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        
        # 로봇 파라미터 개별 선언
        self.declare_parameter('link_1', 0.04)  # m
        self.declare_parameter('link_2', 0.1)   # m
        self.declare_parameter('link_3', 0.1)   # m
        
        # 로봇 몸체 크기
        self.declare_parameter('body_length', 0.19)  # m
        self.declare_parameter('body_width', 0.11)   # m
        self.declare_parameter('body_height', 0.0)   # m
        
        # 서보 오프셋 (라디안)
        self.declare_parameter('servo_offset_1', 0.7854)  # 45도 (π/4)
        self.declare_parameter('servo_offset_2', 0.0)     # 0도
        self.declare_parameter('modeling_offset', 0.2705) # 15.5도
        
        # 개별 다리 위치 파라미터
        self.declare_parameter('default_position_FL', [40.0, 50.0, -100.0])
        self.declare_parameter('default_position_FR', [40.0, -50.0, -100.0])
        self.declare_parameter('default_position_BL', [-40.0, 50.0, -100.0])
        self.declare_parameter('default_position_BR', [-40.0, -50.0, -100.0])
        
        # 초기 다리 위치 설정
        fl_pos = self.get_parameter('default_position_FL').value
        fr_pos = self.get_parameter('default_position_FR').value
        bl_pos = self.get_parameter('default_position_BL').value
        br_pos = self.get_parameter('default_position_BR').value
        
        self.default_positions = [fl_pos, fr_pos, bl_pos, br_pos]
        
        # 퍼블리셔 - 모든 관절 각도를 담은 메시지 발행 (라디안)
        self.joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 
            '/servo_position_controller/commands', 
            10
        )
        
        # 포즈 구독자 - 로봇 몸체의 전체 자세 제어용
        self.pose_subscription = self.create_subscription(
            Pose,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # 다리 위치 구독자 - 보행 생성기로부터 직접 다리 위치 받기
        self.leg_positions_subscription = self.create_subscription(
            Float32MultiArray,
            'leg_positions',
            self.leg_positions_callback,
            10
        )
        
        # 로봇 객체 생성 (파라미터 전달)
        self.quadruped = Quadruped(
            link_1=self.get_parameter('link_1').value,
            link_2=self.get_parameter('link_2').value,
            link_3=self.get_parameter('link_3').value,
            body_length=self.get_parameter('body_length').value,
            body_width=self.get_parameter('body_width').value,
            body_height=self.get_parameter('body_height').value,
            servo_offset_1=self.get_parameter('servo_offset_1').value,
            servo_offset_2=self.get_parameter('servo_offset_2').value,
            modeling_offset=self.get_parameter('modeling_offset').value
        )
        
        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # 로봇 포즈 초기화
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_rotation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        
        self.get_logger().info('IK Node initialized with parameters')
    
    def parameters_callback(self, params):
        """파라미터가 변경되었을 때 호출되는 콜백 함수"""
        for param in params:
            if param.name == 'link_1':
                self.quadruped.link_1 = param.value * self.quadruped.unit
            elif param.name == 'link_2':
                self.quadruped.link_2 = param.value * self.quadruped.unit
            elif param.name == 'link_3':
                self.quadruped.link_3 = param.value * self.quadruped.unit
            elif param.name == 'body_length':
                self.quadruped.length = param.value * self.quadruped.unit
                # 다리 원점 위치 다시 계산
                self.quadruped.update_leg_origins()
            elif param.name == 'body_width':
                self.quadruped.width = param.value * self.quadruped.unit
                # 다리 원점 위치 다시 계산
                self.quadruped.update_leg_origins()
            elif param.name == 'body_height':
                self.quadruped.height = param.value * self.quadruped.unit
                # 다리 원점 위치 다시 계산
                self.quadruped.update_leg_origins()
            elif param.name == 'servo_offset_1':
                self.quadruped.servo_offset_1 = param.value
            elif param.name == 'servo_offset_2':
                self.quadruped.servo_offset_2 = param.value
            elif param.name == 'modeling_offset':
                self.quadruped.modeling_offset = param.value
            elif param.name == 'default_position_FL':
                self.default_positions[0] = param.value
            elif param.name == 'default_position_FR':
                self.default_positions[1] = param.value
            elif param.name == 'default_position_BL':
                self.default_positions[2] = param.value
            elif param.name == 'default_position_BR':
                self.default_positions[3] = param.value
                
        # 정상적으로 파라미터 업데이트 완료
        return True
    
    def pose_callback(self, msg):
        """포즈 메시지가 수신되었을 때 처리하는 콜백 함수"""
        # 포즈에서 위치와 회전 정보 추출
        self.robot_position = [msg.position.x, msg.position.y, msg.position.z]
        
        # 쿼터니언을 오일러로 변환
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.robot_rotation = quaternion_to_euler(quaternion)
        
        # 로봇 포즈 적용 (기본 다리 위치 기준)
        self.apply_robot_pose(self.default_positions)
    
    def leg_positions_callback(self, msg):
        """다리 위치 메시지가 수신되었을 때 처리하는 콜백 함수"""
        # 1차원 배열을 다리별 위치 배열로 변환 (12개 원소 -> 4x3 배열)
        leg_positions = np.array(msg.data).reshape(4, 3).tolist()
        
        # 로봇 포즈 적용 (새 다리 위치 기준)
        self.apply_robot_pose(leg_positions)
    
    def apply_robot_pose(self, leg_positions):
        """
        현재 로봇 포즈(position, rotation)를 적용하여 역기구학 계산
        
        Args:
            leg_positions: 다리 위치 리스트 (4x3 배열, 단위: mm)
        """
        # 로봇 포즈 설정
        self.quadruped.body_x = self.robot_position[0]
        self.quadruped.body_y = self.robot_position[1]
        self.quadruped.body_z = self.robot_position[2]
        self.quadruped.roll = self.robot_rotation[0]
        self.quadruped.pitch = self.robot_rotation[1]
        self.quadruped.yaw = self.robot_rotation[2]
        
        # 다리 위치 계산
        results = self.quadruped.calculate_all_legs(leg_positions)
        
        # 라디안 각도만 추출
        angles = []
        for leg_result in results:
            for i in range(3):  # 각 다리당 3개의 관절 각도
                angles.append(leg_result[i])
        
        # 계산된 관절 각도 발행
        self.publish_joint_angles(angles)
    
    def publish_joint_angles(self, angles):
        """계산된 관절 각도를 발행"""
        msg = Float64MultiArray()
        msg.data = angles
        self.joint_angles_publisher.publish(msg)
        self.get_logger().debug(f'Published joint angles (rad): {angles}')


class Quadruped:
    def __init__(self, link_1=0.04, link_2=0.1, link_3=0.1, 
                 body_length=0.19, body_width=0.11, body_height=0.0,
                 servo_offset_1=0.7854, servo_offset_2=0.0, modeling_offset=0.2705):
        # 다리 ID 정의
        self.leg_F_L = 0
        self.leg_F_R = 1
        self.leg_B_L = 2
        self.leg_B_R = 3

        # 단위 변환 (m -> mm)
        self.unit = 1000
        
        # 링크 길이 (m to mm)
        self.link_1 = link_1 * self.unit
        self.link_2 = link_2 * self.unit
        self.link_3 = link_3 * self.unit

        # 로봇 몸체 크기 (m to mm)
        self.length = body_length * self.unit
        self.width = body_width * self.unit
        self.height = body_height * self.unit
        
        # 다리 원점 위치 계산
        self.update_leg_origins()

        # F:+ , L: +
        # B:- , R: -
        self.leg_direction = np.array([
            [ 1,  1,  1],  # F_L
            [ 1, -1,  1],  # F_R
            [-1,  1,  1],  # B_L
            [-1, -1,  1]   # B_R
        ])
        
        # 서보 오프셋 설정 (이미 라디안)
        self.servo_offset_1 = servo_offset_1
        self.servo_offset_2 = servo_offset_2
        self.modeling_offset = modeling_offset
        self.rad_90 = np.pi/2

        self.last_position_list = [
                [10,  40, -150],  # F_L
                [10, -40, -150],  # F_R
                [-10,  40, -150], # B_L
                [-10, -40, -150]  # B_R
            ]
        
        # 초기 자세 설정
        self.body_x = 0
        self.body_y = 0
        self.body_z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
    
    def update_leg_origins(self):
        """로봇 몸체 크기에 맞게 다리 원점 위치 업데이트"""
        self.leg_origins = np.array([
            [ self.length/2,  self.width/2, self.height/2],  # F_L
            [ self.length/2, -self.width/2, self.height/2],  # F_R
            [-self.length/2,  self.width/2, self.height/2],  # B_L
            [-self.length/2, -self.width/2, self.height/2]   # B_R
        ])

    def leg_ik(self, xyz, rot=[0,0,0], leg_select=0, center_offset=[0,0,0]):
        """다리의 역기구학 계산 (몸체 좌표계 기준)"""
        xyz_trans = np.asarray((inv(e_to_r_matrix(rot, size=3)) @ 
                              ((np.array(xyz) + self.leg_origins[leg_select] - np.array(center_offset)))))
        xyz_ = np.asarray(xyz_trans - self.leg_origins[leg_select] + np.array(center_offset)).flatten()
        return self.ik_cal(xyz_, self.leg_direction[leg_select][0], self.leg_direction[leg_select][1])

    def ik_cal(self, xyz, leg_direction_F_B, leg_direction_L_R):
        """역기구학 계산 (다리 좌표계 기준)"""
        y, z = xyz[1], xyz[2]
        len_A = norm([0,y,z])

        alpha_1 = point_to_rad(z,y)
        alpha_2 = np.arcsin(np.sin(self.rad_90) * self.link_1/len_A)
        alpha_3 = np.pi - self.rad_90 - alpha_2

        theta_1 = alpha_1 + alpha_3 * leg_direction_L_R
        
        joint_1 = np.array([0,0,0])
        joint_2 = np.array([0,self.link_1*np.cos(theta_1), self.link_1*np.sin(theta_1)])
        joint_4 = np.array(xyz)
        joint_4_2_matrix = create_tf_matrix(joint_4 - joint_2)

        r = theta_1 + self.rad_90 * leg_direction_L_R - np.pi/2
        rot_matrix = e_to_r_matrix([-r,0,0])
        joint_4_2_matrix_ = rot_matrix @ joint_4_2_matrix

        x_, z_ = joint_4_2_matrix_[0,3], joint_4_2_matrix_[2,3]
        len_B = norm([x_,0,z_])

        beta_1 = point_to_rad(z_,x_)
        beta_2 = np.arccos((self.link_2**2 + len_B**2 - self.link_3**2)/(2*self.link_2*len_B))
        beta_3 = np.arccos((self.link_2**2 + self.link_3**2 - len_B**2)/(2*self.link_2*self.link_3))

        theta_2 = beta_1 - beta_2
        theta_3 = np.pi - beta_3

        joint_3_ = create_tf_matrix(np.array([self.link_2*np.cos(theta_2),0, self.link_2*np.sin(theta_2)]))
        joint_3_matrix = np.array(create_tf_matrix(joint_2) + np.linalg.inv(rot_matrix) @ joint_3_)
        joint_3 = joint_3_matrix[:3, 3]

        # 각도 오프셋 적용 (라디안 유지)
        angles = self.add_servo_offset([theta_1,theta_2,theta_3], leg_direction_F_B, leg_direction_L_R)
        
        # 관절 위치와 함께 반환 (각도는 라디안으로)
        return [*angles, joint_1, joint_2, joint_3, joint_4]

    def add_servo_offset(self, angles, leg_direction_F_B, leg_direction_L_R):
        """서보 각도 오프셋 적용"""
        angles[1] -= np.pi

        if leg_direction_L_R == 1:     # 왼쪽
            if leg_direction_F_B == 1: # 앞
                theta_1 = angles[0] - np.pi - np.pi/2
            else:                      # 뒤
                theta_1 = np.pi - angles[0] + np.pi + np.pi/2 
            theta_2 = -angles[1] + np.pi - self.servo_offset_1
            theta_3 = np.pi - angles[1] - angles[2] + self.servo_offset_2 + self.modeling_offset

        elif leg_direction_L_R == -1:  # 오른쪽
            if leg_direction_F_B == 1: # 앞
                theta_1 = angles[0] - np.pi/2
            else:                      # 뒤
                theta_1 = -angles[0] + np.pi/2 + np.pi
            theta_2 = angles[1] + self.servo_offset_1
            theta_3 = angles[1] + angles[2] - self.servo_offset_2 - self.modeling_offset
        
        return [theta_1, theta_2, theta_3]

    def calculate_all_legs(self, positions, rot=[0,0,0], center_offset=[0,0,0]):
        """모든 다리의 역기구학 계산"""
        results = []
        for i in range(4):
            angles = self.leg_ik(positions[i], rot=rot, leg_select=i, center_offset=center_offset)
            results.append(angles)
        return results


# util 모듈 함수를 통합
def euler_to_quaternion(r):
    (roll, pitch, yaw) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_to_euler(q):
    """쿼터니언을 오일러 각도(라디안)로 변환"""
    qx, qy, qz, qw = q
    
    # 롤 (x축 회전)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # 피치 (y축 회전)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # 90도 if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # 요 (z축 회전)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

def quaternion_to_r_matrix(quaternion):
    (qx,qy,qz,qw) = (quaternion[0],quaternion[1],quaternion[2],quaternion[3])
    r_matrix = np.array([[1-2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),    2*(qx*qz + qw*qy),    0],
                         [2*(qx*qy + qw*qz),    1-2*(qx**2 + qz**2),  2*(qy*qz - qw*qx),    0],
                         [2*(qx*qz - qw*qy),    2*(qy*qz + qw*qx),    1-2*(qx**2 + qy**2),  0],
                         [0,                    0,                    0,                    1]])
    return r_matrix

def e_to_r_matrix(r,size=4):
    quaternion = euler_to_quaternion(r)
    r_matrix = quaternion_to_r_matrix(quaternion)
    if(size==3):
        r_matrix = r_matrix[:3,:3]
    return r_matrix

def point_to_rad(vertical,horizontal):
    angle = np.arctan2(vertical,horizontal)
    if angle < 0:
        angle += 2* np.pi
    return angle

def create_tf_matrix(xyz):
    matrix = np.eye(4)
    matrix[0, 3] = xyz[0]
    matrix[1, 3] = xyz[1]
    matrix[2, 3] = xyz[2]
    
    return matrix


def main(args=None):
    rclpy.init(args=args)
    ik_node = IKNode()
    
    try:
        rclpy.spin(ik_node)
    except KeyboardInterrupt:
        pass
    finally:
        ik_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()