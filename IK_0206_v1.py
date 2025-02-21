



# 설명. JSY 2025.2.6.

#            X
#            |
#            |
#   Y   ---- .
#            Z

# 좌표계는 위와 같음. 로봇의 정면이 X축을 바라보는 방향임.
# F, L 는 +
# B, R 는 - 임.

# F, B 는 X 축 방향
# L, R 은 Y 축 방향임.
# 아래 코드에서도 이 부분을 따라서 +,-으로 다리의 방향을 결정함.


# 4족보행 형상

#   평면도
#    .        .
#    |        |
#    |--------|
# 1. F_L   2. F_R
#
#
#    정면도           우측면도
# 1. F_L   2. F_R          2. F_R
#    .        .               .
#    |        |               |
#    |--------|         |-----|
#      |    |           |
#    . |    | .         |     .
#    | |    | |         |     |
#    |--------|         |-----|
# 3. B_L   4. B_R          4. B_R
#
#    모터 번호 (정면도 기준)
#    F_L      F_R
#    2        5     
#    1 0    3 4
#
#    8 6    9 11
#    7        10
#    B_L     B_R
#
# 관절이 몸 중심에서 멀어지는 게 숫자가 더 큼. 
# 선 결합시 B_L과 B_R 주의!!
# 

# 다리 순서
# 1. F_L
# 2. F_R
# 3. B_L
# 4. B_R

# servo1, servo2, servo3 에서 
# servo1은 다리 x축의 회전방향
# servo2는 y축 회전방향, 그리고 어깨쪽
# servo3은 y축 회전방향, 그리고 무릎 부분 관절임


import copy
import numpy as np
import serial
import time
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
from util import e_to_r_matrix,point_to_rad,create_tf_matrix

class leg_kinematics():
    def __init__(self, port="COM8", baudrate=115200):
        self.port = port
        self.baudrate = baudrate

        # self.init_serial()

        # self.init_pose()

        # 다리별 인덱스 값
        self.leg_F_L = 0
        self.leg_F_R = 1
        self.leg_B_L = 2
        self.leg_B_R = 3

        # 단위 m -> mm
        self.unit = 1000
        # 링크 길이
        self.link_1 = 0.04 * self.unit
        self.link_2 = 0.1 * self.unit
        self.link_3 = 0.1 * self.unit
        # 몸통 폭, 너비, 깊이
        self.length = 0.40 * self.unit
        self.width = 0.3 * self.unit
        self.hight = 0.0 * self.unit
        
        # leg_F_L, leg_F_R, leg_B_L, leg_B_R 순서 xyz
        self.leg_origins = np.array([[ self.length/2,  self.width/2, self.hight/2],
                                     [ self.length/2, -self.width/2, self.hight/2],
                                     [-self.length/2,  self.width/2, self.hight/2],
                                     [-self.length/2, -self.width/2, self.hight/2]])

        # F:+ , L: +
        # B:- , R: -
        # hight 때문에 마지막 + 하나 추가해둠.
        # 다리 방향값 담고있는 배열
        self.leg_direction = np.array([[ 1,  1,  1],
                                       [ 1, -1,  1],
                                       [-1,  1,  1],
                                       [-1, -1,  1]])
        # 서보 오프셋 설정 부분
        self.servo_offset_1 = np.pi / 4
        self.servo_offset_2 = np.pi * 15.5/180
        self.rad_90 = np.pi/2

        # 다리 걸음걸이 패턴에서 다리 4개중 들어올리는 다리 순서임.
        self.crawling_seq = [3, 1, 2, 0]

        # 로봇 4발의 마지막 XYZ 위치 값.
        self.last_position_list = [[10,  40, -150],[10, -40, -150],[-10,  40, -150],[-10, -40, -150]]
        self.straight_line_waypoints =[]



    # 시리얼 통신 설정 하는 함수.
    def init_serial(self):
        self.seq = serial.Serial()
        self.seq.port = self.port
        self.seq.baudrate = self.baudrate
        self.seq.parity = serial.PARITY_NONE
        self.seq.stopbits = serial.STOPBITS_ONE
        self.seq.bytesize = serial.EIGHTBITS
        self.seq.timeout = 1
        try:
            self.seq.open()
            print(f"Port {self.port} Connected.")
        except Exception as e:
            print(f"Can't open port: {e}")


    # 다리 어깨를 원점으로 하는 각 다리 좌표계를 로봇 몸통 중심을 원점으로 하는 좌표계로 바꾸는 함수.
    def leg_ik(self, xyz, rot=[0,0,0], leg_select=0, center_offset=[0,0,0]):
        xyz_body_frame = np.array(xyz) + self.leg_origins[leg_select] - np.array(center_offset)
        XYZ = inv(e_to_r_matrix(rot, size=3)) @ xyz_body_frame
        xyz_trans = XYZ - self.leg_origins[leg_select] + np.array(center_offset)
        return self.ik_cal(xyz_trans, self.leg_direction[leg_select][0], self.leg_direction[leg_select][1])

    # 역기구학 계산하는 함수. 입력된 전후좌우에 맞춰 계산함.
    def ik_cal(self, xyz, leg_direction_F_B, leg_direction_L_R):
        y,z = xyz[1],xyz[2]
        len_A = norm([0,y,z])
        alpha_1 = point_to_rad(z,y)
        alpha_2 = np.arcsin(np.sin(self.rad_90) * self.link_1/len_A)
        alpha_3 = np.pi - self.rad_90 - alpha_2
        theta_1 = alpha_1 + alpha_3* leg_direction_L_R
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
        servo_angle_1,servo_angle_2,servo_angle_3 = self.add_servo_offset([theta_1,theta_2,theta_3],leg_direction_F_B,leg_direction_L_R)
        servo_angle_1 = np.degrees(servo_angle_1)
        servo_angle_2 = np.degrees(servo_angle_2)
        servo_angle_3 = np.degrees(servo_angle_3)
        # print(np.degrees(servo_angle_1), np.degrees(servo_angle_2), np.degrees(servo_angle_3))
        return servo_angle_1, servo_angle_2, servo_angle_3, joint_1, joint_2, joint_3, joint_4
        
    # 전후좌우 서보모터 축 방향에 따른 각도 보정 함수
    def add_servo_offset(self,angles,leg_direction_F_B,leg_direction_L_R):
        angles[1] -= np.pi

        if leg_direction_L_R == 1:     # 왼쪽
            if leg_direction_F_B == 1: # 앞
                theta_1 = angles[0] - np.pi - np.pi/2
            else:                      # 뒤
                theta_1 = np.pi - angles[0] + np.pi + np.pi/2 
            theta_2 = -angles[1] + np.pi - self.servo_offset_1
            theta_3 = np.pi -angles[1] - angles[2] + self.servo_offset_2

        elif leg_direction_L_R == -1:  # 오른쪽
            if leg_direction_F_B == 1: # 앞
                theta_1 = angles[0] - np.pi/2
            else:                      # 뒤
                theta_1 = -angles[0] + np.pi/2 + np.pi
            theta_2 = angles[1] + self.servo_offset_1
            theta_3 = angles[1] + angles[2] - self.servo_offset_2
        return [theta_1,theta_2,theta_3]

    # 초기 자세 지정용.
    def init_pose(self):
        cmd="1a90.0b86.3c56.0d90.0e93.7f124.0g90.0h90.6i59.9j90.0k89.4l120.1m"
        self.seq.write(cmd.encode())
        time.sleep(1)

    # XYZ 위치 값을 서보모터 각도값으로 바꾸기 위해 leg_ik 함수 호출 하는 함수
    def poses_calc_all_leg(self, xyz_list, plot_select=0):
        # ex) xyz_all = [ [0,40,-100], [0,-40,-100], [0,40,-100], [0,-40,-100] ]
        #     dog.poses_calc_all_leg(xyz_all)
        #     output -> [[90.0, 105.0, 45.5], [90.0, 75.0, 134.5], [90.0, 105.0, 45.5], [90.0, 75.0, 134.5]]
        servo_angles_list = []
        for i in range(4):
            angles = self.leg_ik(xyz_list[i], rot=[0, 0, 0], leg_select=i)
            if plot_select == 0:
                servo_angles_list.append([round(angle, 1) for angle in angles[:3]])
            else:
                servo_angles_list.append(list(angles))
        # print(self.servo_angles_list)
        return servo_angles_list
    
    # plan_list 순서대로 동작을 위해 다리 동작 함수 호출
    def move(self, plan_list, sec=1):
        for plan in plan_list:
            self.ctrl_leg(plan,sec)

    # 4발을 waypoint대로 움직이기 위해 사용하는 함수 
    def planning(self, waypoint_list):
        # ex)  plan = cat.planning(waypoints)
        #      time.sleep(2)
        #      cat.move(plan,sec=1.1)
        move_plan_list = [self.poses_calc_all_leg(waypoint) for waypoint in waypoint_list]
        # print(move_plan_list)
        return move_plan_list
    
    # 다리 동작 함수
    def ctrl_leg(self,servo_angles_list,sec=1):
        cmd = "1a"+str(float(servo_angles_list[0][0]))+'b'+str(float(servo_angles_list[0][1]))+'c'+str(float(servo_angles_list[0][2])) +'d'+str(float(servo_angles_list[1][0]))+'e'+str(float(servo_angles_list[1][1]))+'f'+str(float(servo_angles_list[1][2]))+'g'+str(float(servo_angles_list[2][0]))+'h'+str(float(servo_angles_list[2][1]))+'i'+str(float(servo_angles_list[2][2]))+'j'+str(float(servo_angles_list[3][0]))+'k'+str(float(servo_angles_list[3][1]))+'l'+str(float(servo_angles_list[3][2]))+'m'
        # print(cmd)
        print(servo_angles_list)
        self.seq.write(cmd.encode())
        time.sleep(sec)

    def plot_all_leg(self,limit=200):
        ax=self.plot_setup(limit)
        plt.show()
    

    # 로봇이 직선으로 이동하기위해 4발의 XYZ 위치 값을 계산하는 함수. 내부에 pl
    def move_straight_line(self):
        for loop in range(0,4):
            for seq in crawling_seq:
                for small_pattern in range(0,3):
                    tmp_position_list = copy.deepcopy(self.last_position_list)
                    if small_pattern == 0:
                        tmp_position_list[seq] = self.xyz_ctrl_leg(tmp_position_list[seq], stride_z=40)

                    # 패턴 1. 특정 다리 든 체로 앞으로. 다른 다리들 뒤로
                    elif small_pattern == 1:
                        for i in range(0,4):
                            if i == seq:
                                tmp_position_list[seq] = self.xyz_ctrl_leg(tmp_position_list[seq], stride_x=25*3)
                            else:
                                tmp_position_list[i] = self.xyz_ctrl_leg(tmp_position_list[i], stride_x=-25)

                    # 패턴 3. 들었던 다리 내리기.
                    elif small_pattern == 2:
                        tmp_position_list[seq] = self.xyz_ctrl_leg(tmp_position_list[seq], stride_z=-40)
                        
                    # self.last_position_list = copy.deepcopy(tmp_position_list)
                    # self.straight_line_waypoints.append(tmp_position_list)
                    plan = self.planning(copy.deepcopy(tmp_position_list))
                    self.move(plan,sec=1.1)
                    # 아 지금 이거 waypiont 가 한개 리스트에 [ [],[],[],[] ] 이거 인지 [] 인지 그차이 인거같은데.. 그레서 에러뜸 일단..


    @staticmethod
    def xyz_ctrl_leg(position_list, stride_x=0, stride_y=0, stride_z=0):
        # 한 다리의 position를 하나의 리스트로 받음. 그리고 간격 만큼 더해서 반환해줌
        # ex)  a=[10,40,-150]
        #      b=a
        #      b=xyz_ctrl_leg(a,stride_x=20)
        #      print(a,b)
        position_list = copy.deepcopy(position_list)
        position_list[0] += stride_x
        position_list[1] += stride_y
        position_list[2] += stride_z
        return position_list
    
    @staticmethod
    def plot_setup(limit=200):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_box_aspect([1, 1, 1])
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_xlim([-limit, limit])
        ax.set_ylim([-limit, limit])
        ax.set_zlim([-limit, limit])
        ax.set_title('3D view')
        return ax


cat = leg_kinematics("COM8",115200)

stride_x = 20
stride_y = 0
stride_z = 30
crawling_seq=[3, 1, 2, 0]


waypoints=[]

leg_0 = [210,  190, -150]
leg_1 = [210, -190, -150]
leg_2 = [-210,  190, -150]
leg_3 = [-210, -190, -150]
last_position_list = [leg_0,leg_1,leg_2,leg_3]

waypoints.append(last_position_list)

cat.move_straight_line()

# leg_0 = [10,  40, -150]
# leg_1 = [10, -40, -150]
# leg_2 = [-10,  40, -150]
# leg_3 = [-10, -40, -100]
# last_position_list_1 = [leg_0,leg_1,leg_2,leg_3]
# waypoints.append(last_position_list_1)

# leg_0 = [10,  40, -150]
# leg_1 = [10, -40, -150]
# leg_2 = [-10,  40, -150]
# leg_3 = [-10, -40, -150]
# last_position_list_2 = [leg_0,leg_1,leg_2,leg_3]
# waypoints.append(last_position_list_2)

# cat.move(plan,sec=1.1)
