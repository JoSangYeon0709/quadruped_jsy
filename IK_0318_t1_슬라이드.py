import numpy as np
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
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
        self.servo_offset_1 = np.pi * 45/180
        self.servo_offset_2 = np.pi * 0
        self.rad_90 = np.pi/2

        self.modeling_offset = np.pi * 15.5/180 # 종아리 파츠 각도 오프셋임. 모델링에서 변경되면 바꿔야함.

        # 초기 다리 위치
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

    def update_robot_pose(self, x, y, z, roll, pitch, yaw):
        """로봇 자세 업데이트 - 다리 길이 조절을 통한 몸통 위치 제어"""
        self.body_x = x
        self.body_y = y
        self.body_z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        
        positions = []
        for i in range(4):
            leg_pos = copy.deepcopy(self.last_position_list[i])
            leg_pos[0] -= x
            leg_pos[1] -= y
            leg_pos[2] -= z
            positions.append(leg_pos)
        
        results = []
        for i in range(4):
            angles = self.leg_ik(positions[i], rot=[roll, pitch, yaw], leg_select=i)
            results.append(angles)
        return results

    def create_interactive_plot(self, limit=80):
        """슬라이더가 있는 인터랙티브 플롯 생성"""
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        plt.subplots_adjust(bottom=0.35)

        # 슬라이더 생성
        slider_ax = [
            plt.axes([0.2, 0.25, 0.6, 0.02]),  # x
            plt.axes([0.2, 0.20, 0.6, 0.02]),  # y
            plt.axes([0.2, 0.15, 0.6, 0.02]),  # z
            plt.axes([0.2, 0.10, 0.6, 0.02]),  # roll
            plt.axes([0.2, 0.05, 0.6, 0.02]),  # pitch
            plt.axes([0.2, 0.00, 0.6, 0.02])   # yaw
        ]

        sliders = [
            Slider(slider_ax[0], 'X Position', -limit, limit, valinit=0),
            Slider(slider_ax[1], 'Y Position', -limit, limit, valinit=0),
            Slider(slider_ax[2], 'Body Height', -limit-30, limit-50, valinit=-15),
            Slider(slider_ax[3], 'Roll', -np.pi/3, np.pi/3, valinit=0),
            Slider(slider_ax[4], 'Pitch', -np.pi/3, np.pi/3, valinit=0),
            Slider(slider_ax[5], 'Yaw', -np.pi/3, np.pi/3, valinit=0)
        ]

        def update(val):
            """슬라이더 값 변경시 호출되는 함수"""
            x = sliders[0].val
            y = sliders[1].val
            z = sliders[2].val
            roll = sliders[3].val
            pitch = sliders[4].val
            yaw = sliders[5].val

            # 로봇 자세 업데이트
            joints = self.update_robot_pose(x, y, z, roll, pitch, yaw)
            
            # 시각화 업데이트
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            zlim = ax.get_zlim()
            elev = ax.elev
            azim = ax.azim

            ax.clear()

            self.plot_current_pose(ax, joints)
            ax.view_init(elev=elev, azim=azim)
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
            ax.set_zlim(zlim)
            fig.canvas.draw_idle()

        # 슬라이더 이벤트 연결
        for slider in sliders:
            slider.on_changed(update)

        # 초기 포즈 그리기
        joints = self.update_robot_pose(0, 0, 0, 0, 0, 0)
        self.plot_current_pose(ax, joints)

        plt.show()

    def plot_current_pose(self, ax, joints, limit=300):
        """현재 자세 시각화"""
        # 축 설정
        ax.set_xlim([-limit, limit])
        ax.set_ylim([-limit, limit])
        ax.set_zlim([-limit, limit])
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_title('Robot Pose Control')
        ax.grid(True)

        # 로봇 자세 변환 행렬 생성
        rot_matrix = e_to_r_matrix([self.roll, self.pitch, self.yaw], size=3)
        offset = np.array([self.body_x, self.body_y, self.body_z])

        # 베이스 프레임 그리기
        base_points = np.vstack([self.leg_origins, self.leg_origins[0]])
        transformed_base = np.array([rot_matrix @ p[:3] + offset for p in base_points])
        ax.plot3D(transformed_base[:,0], transformed_base[:,1], 
                 transformed_base[:,2], 'r-', linewidth=2, label='Base')

        colors = ['b', 'g', 'c', 'm']
        leg_names = ['F_L', 'F_R', 'B_L', 'B_R']

        for i, joint_set in enumerate(joints):
            j1, j2, j3, j4 = [np.array(j)[:3] for j in joint_set[3:]]
            
            # 다리 원점에 상대적인 위치로 변환
            leg_origin = self.leg_origins[i]
            joint_positions = np.array([
                j1 + leg_origin,
                j2 + leg_origin,
                j3 + leg_origin,
                j4 + leg_origin
            ])

            # 로봇 자세에 따른 변환 적용
            transformed_joints = np.array([rot_matrix @ p + offset for p in joint_positions])

            ax.plot3D(transformed_joints[:,0], transformed_joints[:,1], 
                     transformed_joints[:,2], color=colors[i], 
                     linewidth=2, label=leg_names[i])

            ax.scatter(transformed_joints[:,0], transformed_joints[:,1], 
                      transformed_joints[:,2], color=colors[i], s=50)

        ax.legend()
        ax.view_init(elev=20, azim=45)

# 사용 예시
if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')  # 백엔드를 TkAgg로 설정
    
    vettar = Quadruped()
    
    try:
        vettar.create_interactive_plot()
        plt.show()  # 메인 스레드에서 plot 표시
    except KeyboardInterrupt:
        plt.close('all') 