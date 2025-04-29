import numpy as np
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
from util import e_to_r_matrix, point_to_rad, create_tf_matrix
import copy

class Quadruped:
    def __init__(self):
        # 다리 ID 정의
        self.leg_F_L = 0
        self.leg_F_R = 1
        self.leg_B_L = 2
        self.leg_B_R = 3

        # 단위 변환 (m -> mm)
        self.unit = 1000
        
        # 링크 길이
        self.link_1 = 0.04 * self.unit
        self.link_2 = 0.1 * self.unit
        self.link_3 = 0.1 * self.unit

        # 로봇 몸체 크기
        self.length = 0.19 * self.unit
        self.width = 0.11 * self.unit
        self.hight = 0.0 * self.unit
        
        # 다리 원점 위치 (F_L, F_R, B_L, B_R 순서)
        self.leg_origins = np.array([
            [ self.length/2,  self.width/2, self.hight/2],
            [ self.length/2, -self.width/2, self.hight/2],
            [-self.length/2,  self.width/2, self.hight/2],
            [-self.length/2, -self.width/2, self.hight/2]
        ])

        # F:+ , L: +
        # B:- , R: -
        self.leg_direction = np.array([
            [ 1,  1,  1],  # F_L
            [ 1, -1,  1],  # F_R
            [-1,  1,  1],  # B_L
            [-1, -1,  1]   # B_R
        ])

        # 서보 오프셋 설정
        self.servo_offset_1 = np.pi / 4
        self.servo_offset_2 = np.pi * 15.5/180
        self.rad_90 = np.pi/2

        # 걸음걸이 순서
        self.crawling_seq = [3, 1, 2, 0]

        # 마지막 다리 위치 저장
        self.last_position_list = [
            [10,  40, -150],  # F_L
            [10, -40, -150],  # F_R
            [-10,  40, -150], # B_L
            [-10, -40, -150]  # B_R
        ]

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

        angles = self.add_servo_offset([theta_1,theta_2,theta_3], leg_direction_F_B, leg_direction_L_R)
        return [*[np.degrees(angle) for angle in angles], joint_1, joint_2, joint_3, joint_4]

    def add_servo_offset(self, angles, leg_direction_F_B, leg_direction_L_R):
        """서보 각도 오프셋 적용"""
        angles[1] -= np.pi

        if leg_direction_L_R == 1:     # 왼쪽
            if leg_direction_F_B == 1: # 앞
                theta_1 = angles[0] - np.pi - np.pi/2
            else:                      # 뒤
                theta_1 = np.pi - angles[0] + np.pi + np.pi/2 
            theta_2 = -angles[1] + np.pi - self.servo_offset_1
            theta_3 = np.pi - angles[1] - angles[2] + self.servo_offset_2

        elif leg_direction_L_R == -1:  # 오른쪽
            if leg_direction_F_B == 1: # 앞
                theta_1 = angles[0] - np.pi/2
            else:                      # 뒤
                theta_1 = -angles[0] + np.pi/2 + np.pi
            theta_2 = angles[1] + self.servo_offset_1
            theta_3 = angles[1] + angles[2] - self.servo_offset_2
        
        return [theta_1, theta_2, theta_3]

    def calculate_all_legs(self, xyz_list, plot_select=0):
        """모든 다리의 역기구학 계산"""
        results = []
        for i in range(4):
            angles = self.leg_ik(xyz_list[i], rot=[0, 0, 0], leg_select=i)
            if plot_select == 0:
                results.append([round(angle, 2) for angle in angles[:3]])
            else:
                results.append(list(angles))
        return results

    @staticmethod
    def xyz_ctrl_leg(position_list, stride_x=0, stride_y=0, stride_z=0):
        """다리 위치 제어"""
        position_list = copy.deepcopy(position_list)
        position_list[0] += stride_x
        position_list[1] += stride_y
        position_list[2] += stride_z
        return position_list

    def walk_straight(self, stride_x=40, stride_z=40):
        """직선 보행 패턴 생성"""
        waypoints = []
        positions = copy.deepcopy(self.last_position_list)
        waypoints.append(positions)

        for seq in self.crawling_seq:
            # 1. 다리 들기
            positions = copy.deepcopy(self.last_position_list)
            positions[seq] = self.xyz_ctrl_leg(positions[seq], stride_z=stride_z)
            waypoints.append(positions)

            # 2. 다리 앞으로
            positions = copy.deepcopy(positions)
            positions[seq] = self.xyz_ctrl_leg(positions[seq], stride_x=stride_x*3)
            waypoints.append(positions)

            # 3. 다리 내리기
            positions = copy.deepcopy(positions)
            positions[seq] = self.xyz_ctrl_leg(positions[seq], stride_z=-stride_z)
            waypoints.append(positions)

            # 4. 나머지 다리 뒤로
            positions = copy.deepcopy(positions)
            for i in range(4):
                if i != seq:
                    positions[i] = self.xyz_ctrl_leg(positions[i], stride_x=-stride_x)
            waypoints.append(positions)
            self.last_position_list = copy.deepcopy(positions)

        return waypoints

    def plot_robot(self, xyz_list=None, rot=[0,0,0], limit=200):
        """로봇 시각화"""
        if xyz_list is None:
            xyz_list = self.last_position_list
            
        ax = self.setup_plot(limit)
        
        # 베이스 프레임 그리기
        base_points = np.vstack([self.leg_origins, self.leg_origins[0]])
        ax.plot3D(base_points[:,0], base_points[:,1], base_points[:,2], 'r-')
        
        # 다리 그리기
        for i, xyz in enumerate(xyz_list):
            angles = self.leg_ik(xyz, rot, i)
            joints = np.vstack([angles[3:]])  # joint_1, joint_2, joint_3, joint_4
            # 다리 원점으로 이동
            joints = joints + self.leg_origins[i]
            ax.plot3D(joints[:,0], joints[:,1], joints[:,2], 'b-')
        
        plt.show()

    @staticmethod
    def setup_plot(limit):
        """플롯 설정"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_box_aspect([1,1,1])
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-limit, limit])
        ax.set_ylim([-limit, limit])
        ax.set_zlim([-limit, limit])
        ax.set_title('Robot Visualization')
        return ax

    def plot_robot_realtime(self, waypoints, interval=0.1):
        """실시간 로봇 시각화"""
        plt.ion()  # 대화형 모드 활성화
        
        # 그림 설정
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 시각화 범위 설정
        limit = 300
        
        for xyz_list in waypoints:
            # 현재 뷰 각도 저장
            elev = ax.elev
            azim = ax.azim
            
            ax.clear()  # 이전 프레임 지우기
            
            # 축 설정
            ax.set_xlim([-limit, limit])
            ax.set_ylim([-limit, limit])
            ax.set_zlim([-limit, limit])
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            ax.set_title('Robot Visualization')
            ax.grid(True)
            
            # 베이스 프레임 그리기
            base_points = np.vstack([self.leg_origins, self.leg_origins[0]])
            ax.plot3D(base_points[:,0], base_points[:,1], base_points[:,2], 'r-', linewidth=2, label='Base')
            
            # 각 다리 그리기
            colors = ['b', 'g', 'c', 'm']  # 각 다리별 다른 색상
            leg_names = ['F_L', 'F_R', 'B_L', 'B_R']
            
            for i, xyz in enumerate(xyz_list):
                angles = self.leg_ik(xyz, rot=[0,0,0], leg_select=i)
                joints = np.vstack([angles[3:]])  # joint_1, joint_2, joint_3, joint_4
                
                # 다리 원점으로 이동
                joints = joints + self.leg_origins[i]
                
                # 다리 그리기
                ax.plot3D(joints[:,0], joints[:,1], joints[:,2], 
                         color=colors[i], linewidth=2, label=leg_names[i])
                
                # 관절 표시
                ax.scatter(joints[:,0], joints[:,1], joints[:,2], 
                          color=colors[i], s=50)
            
            ax.legend()  # 범례 표시
            
            # 시점 설정 (3D 뷰 각도 유지)
            ax.view_init(elev=elev, azim=azim)
            
            plt.draw()
            plt.pause(interval)
        
        plt.ioff()
        plt.show()

# 사용 예시
if __name__ == "__main__":
    robot = Quadruped()
    
    # 보행 패턴 생성 (한 스텝만)
    waypoints = robot.walk_straight()
    
    # 실시간 시각화
    robot.plot_robot_realtime(waypoints, interval=0.5)

    # waypoints 확인
    print("Number of waypoints:", len(waypoints))
    print("First waypoint:", waypoints[0]) 