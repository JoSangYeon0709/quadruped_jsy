import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
from matplotlib.widgets import Slider

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
        self.height = 0.0 * self.unit
        
        # 다리 원점 위치 (F_L, F_R, B_L, B_R 순서)
        self.leg_origins = np.array([
            [ self.length/2,  self.width/2, self.height/2],
            [ self.length/2, -self.width/2, self.height/2],
            [-self.length/2,  self.width/2, self.height/2],
            [-self.length/2, -self.width/2, self.height/2]
        ])

        # F:+ , L: +
        # B:- , R: -
        self.leg_direction = np.array([
            [ 1,  1,  1],  # F_L
            [ 1, -1,  1],  # F_R
            [-1,  1,  1],  # B_L
            [-1, -1,  1]   # B_R
        ])

        self.modeling_offset = 15.5 # 종아리 파츠 각도 오프셋임. 모델링에서 변경되면 바꿔야함.

        # 각 다리별 서보 오프셋 수정
        self.servo_offsets = {
            'F_L': [90, 135, self.modeling_offset],    # 앞왼쪽
            'F_R': [90, 45, -self.modeling_offset],    # 앞오른쪽
            'B_L': [90, 135, self.modeling_offset],    # 뒤왼쪽
            'B_R': [90, 45, -self.modeling_offset]     # 뒤오른쪽
        }
        

    def create_leg_transforms(self, leg_idx):
        """각 다리의 변환 행렬 생성"""
        leg_dir_L_R = self.leg_direction[leg_idx][1]
        origin = self.leg_origins[leg_idx]

        M_3 = np.array([[1, 0, 0, -(self.link_2+self.link_3)],
                        [0, 1, 0, self.link_1 * leg_dir_L_R],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        M_2 = np.array([[1, 0, 0, -self.link_2],
                        [0, 1, 0, self.link_1 * leg_dir_L_R],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        M_1 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, self.link_1 * leg_dir_L_R],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        # 스크류 축 설정
        W_3 = np.array([0, -1, 0])
        W_2 = np.array([0, -1, 0])
        W_1 = np.array([1, 0, 0])

        q_3 = np.array([-self.link_2, 0, 0])
        q_2 = np.array([0, self.link_1 * leg_dir_L_R, 0])
        q_1 = np.array([0, 0, 0])

        V_3 = np.cross(-W_3, q_3)
        V_2 = np.cross(-W_2, q_2)
        V_1 = np.cross(-W_1, q_1)

        S_3 = np.concatenate((W_3, V_3))
        S_2 = np.concatenate((W_2, V_2))
        S_1 = np.concatenate((W_1, V_1))

        return M_1, M_2, M_3, S_1, S_2, S_3, origin

    def exponential_matrix(self, twist, theta):
        """지수 행렬 계산"""
        W = twist[:3]
        V = twist[3:]
        
        if np.allclose(W, 0):
            R = np.eye(3)
            t = V * theta
        else:
            W_skew = np.array([[0, -W[2], W[1]],
                              [W[2], 0, -W[0]],
                              [-W[1], W[0], 0]])
            R = expm(W_skew * theta)
            t = (np.eye(3) - R).dot(np.cross(W, V)) + np.outer(W, W).dot(V) * theta
        
        matrix = np.eye(4)
        matrix[:3, :3] = R
        matrix[:3, 3] = t
        return matrix

    def calculate_leg_positions(self, theta_1, theta_2, theta_3, leg_idx):
        """각 다리의 관절 위치 계산"""
        M_1, M_2, M_3, S_1, S_2, S_3, origin = self.create_leg_transforms(leg_idx)
        leg_dir_F_B = self.leg_direction[leg_idx][0]
        leg_dir_L_R = self.leg_direction[leg_idx][1]
        leg_name = ['F_L', 'F_R', 'B_L', 'B_R'][leg_idx]
        
        # 서보 오프셋 가져오기
        servo_1_offset = np.radians(self.servo_offsets[leg_name][0])
        servo_2_offset = np.radians(self.servo_offsets[leg_name][1])
        servo_3_offset = np.radians(self.servo_offsets[leg_name][2])

        # 각도를 라디안으로 변환
        theta_1 = np.radians(theta_1)
        theta_2 = np.radians(theta_2)
        theta_3 = np.radians(theta_3)

        # 모든 다리에서 1번, 2번 모터를 CCW 방향으로 설정
        if leg_dir_L_R == 1:  # 왼쪽
            if leg_dir_F_B == 1:  # F_L
                theta_1_adj = theta_1 - servo_1_offset
                theta_2_adj = -(theta_2 - servo_2_offset)
                theta_3_adj = -(theta_3 - servo_3_offset + np.pi ) -theta_2_adj
            else:  # B_L
                theta_1_adj = -(theta_1 - servo_1_offset)
                theta_2_adj = -(theta_2 - servo_2_offset)
                theta_3_adj = -(theta_3 - servo_3_offset + np.pi ) -theta_2_adj
        else:  # 오른쪽
            if leg_dir_F_B == 1:  # F_R
                theta_1_adj = theta_1 - servo_1_offset
                theta_2_adj = theta_2 - servo_2_offset
                theta_3_adj = theta_3 - servo_3_offset -theta_2_adj
            else:  # B_R
                theta_1_adj = -(theta_1 - servo_1_offset)
                theta_2_adj = theta_2 - servo_2_offset
                theta_3_adj = theta_3 - servo_3_offset -theta_2_adj

        # 변환 행렬 계산
        T_1 = self.exponential_matrix(S_1, theta_1_adj)
        T_2 = self.exponential_matrix(S_2, theta_2_adj)
        T_3 = self.exponential_matrix(S_3, theta_3_adj)

        # 각 관절 위치 계산
        g_1 = T_1 @ M_1
        g_2 = T_1 @ T_2 @ M_2
        g_3 = T_1 @ T_2 @ T_3 @ M_3

        # 원점 기준 위치로 변환
        joint_1 = origin
        joint_2 = g_1[:3, 3] + origin
        joint_3 = g_2[:3, 3] + origin
        joint_4 = g_3[:3, 3] + origin

        return [joint_1, joint_2, joint_3, joint_4]

    def create_interactive_plot(self):
        """인터랙티브 플롯 생성"""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 슬라이더용 별도 창
        fig_sliders = plt.figure(figsize=(8, 5))
        

        ax.set_xlim([-300, 300])
        ax.set_ylim([-300, 300])
        ax.set_zlim([-300, 300])
        ax.view_init(elev=20, azim=45)

        # 앞쪽 다리부터 순서
        leg_names = ['F_L', 'F_R', 'B_L', 'B_R']
        sliders = []
        
        # 초기 모터 각도 설정
        initial_angles = [
            [90.0, 86.3, 56.0],    # F_L
            [90.0, 93.7, 124.0],   # F_R
            [90.0, 94.5, 64.2],    # B_L
            [90.0, 85.5, 115.8]    # B_R
        ]

        # 슬라이더 생성
        for i, leg_name in enumerate(leg_names):
            base_height = 0.8 - i * 0.2  # 슬라이더 간격 조정
            slider_ax = [
                fig_sliders.add_axes([0.2, base_height + 0.1, 0.6, 0.03]),
                fig_sliders.add_axes([0.2, base_height + 0.05, 0.6, 0.03]),
                fig_sliders.add_axes([0.2, base_height, 0.6, 0.03]),
            ]

            leg_sliders = [
                Slider(
                    slider_ax[0], 
                    f'{leg_name} θ₁ (deg)', 
                    0, 180, 
                    valinit=initial_angles[i][0],
                    valfmt='%.2f°'
                ),
                Slider(
                    slider_ax[1], 
                    f'{leg_name} θ₂ (deg)', 
                    0, 180, 
                    valinit=initial_angles[i][1],
                    valfmt='%.2f°'
                ),
                Slider(
                    slider_ax[2], 
                    f'{leg_name} θ₃ (deg)', 
                    0, 180, 
                    valinit=initial_angles[i][2],
                    valfmt='%.2f°'
                )
            ]
            sliders.append(leg_sliders)



        def update(val):
            """슬라이더 값이 변경될 때 호출되는 함수"""
        
            # 그래프 설정
            xlim = ax.get_xlim()
            ylim = ax.get_ylim()
            zlim = ax.get_zlim()
            elev = ax.elev
            azim = ax.azim

            ax.clear()

            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
            ax.set_zlim(zlim)
            ax.set_title('Robot FK')
            ax.grid(True)

            
            all_joints = []
            for i in range(4):
                theta_1 = sliders[i][0].val
                theta_2 = sliders[i][1].val
                theta_3 = sliders[i][2].val
                joints = self.calculate_leg_positions(theta_1, theta_2, theta_3, i)
                all_joints.append(joints)
            
            # 베이스 프레임 그리기
            base_points = np.vstack([self.leg_origins, self.leg_origins[0]])
            ax.plot3D(base_points[:,0], base_points[:,1], base_points[:,2], 
                    'r-', linewidth=2, label='Base')

            # 각 다리 그리기
            colors = ['b', 'g', 'c', 'm']
            leg_names = ['F_L', 'F_R', 'B_L', 'B_R']
            
            for i, joints in enumerate(all_joints):
                joints = np.array(joints)
                ax.plot3D(joints[:,0], joints[:,1], joints[:,2], 
                        color=colors[i], linewidth=2, label=leg_names[i])
                ax.scatter(joints[:,0], joints[:,1], joints[:,2], 
                        color=colors[i], s=50)
            
            ax.legend()
            ax.view_init(elev=elev, azim=azim)
        
            fig.canvas.draw_idle()

        # 모든 슬라이더에 업데이트 함수 연결
        for leg_sliders in sliders:
            for slider in leg_sliders:
                slider.on_changed(update)

        # 초기 포즈 그리기
        update(None)

        plt.show()

if __name__ == "__main__":
    import matplotlib
    matplotlib.use('TkAgg')
    
    vettar = Quadruped()
    vettar.create_interactive_plot() 