



# 설명. JSY 25.1.23.

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



import numpy as np
import serial
import time
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
from util import e_to_r_matrix,point_to_rad,create_tf_matrix

class leg_kinematics():
    def __init__(self, port="COM8", baudrate=115200):
        self.seq = serial.Serial()
        self.seq.port = port
        self.seq.baudrate = baudrate
        self.seq.parity = serial.PARITY_NONE
        self.seq.stopbits = serial.STOPBITS_ONE
        self.seq.bytesize = serial.EIGHTBITS
        self.seq.timeout = 1
        try:
            self.seq.open()
            print(f"port {port} Connected.")
        except Exception as e:
            print(f"Can't open port: {e}")

        self.seq.write("0".encode())
        time.sleep(2)
        # self.init_pose()

        self.leg_F_L = 0
        self.leg_F_R = 1
        self.leg_B_L = 2
        self.leg_B_R = 3

        self.unit = 1000

        self.link_1 = 0.04 * self.unit
        self.link_2 = 0.1 * self.unit
        self.link_3 = 0.1 * self.unit

        self.length = 0.20 * self.unit
        self.width = 0.15 * self.unit
        self.hight = 0.0 * self.unit
        
        # leg_F_L, leg_F_R, leg_B_L, leg_B_R 순서 xyz
        self.leg_origins = np.array([[ self.length/2,  self.width/2, self.hight/2],
                                     [ self.length/2, -self.width/2, self.hight/2],
                                     [-self.length/2,  self.width/2, self.hight/2],
                                     [-self.length/2, -self.width/2, self.hight/2]])

        # F:+ , L: +
        # B:- , R: -
        # hight 때문에 마지막 + 하나 추가해둠.

        self.leg_direction = np.array([[ 1,  1,  1],
                                       [ 1, -1,  1],
                                       [-1,  1,  1],
                                       [-1, -1,  1]])
        
        self.servo_offset_1 = np.pi / 4
        self.servo_offset_2 = np.pi * 15.5/180
        self.rad_90 = np.pi/2
    
    def leg_ik(self, xyz, rot = [0,0,0], leg_select=0, center_offset=[0,0,0]):
        xyz_trans = np.asarray((inv(e_to_r_matrix(rot, size=3)) @ ((np.array(xyz) + self.leg_origins[leg_select] - np.array(center_offset)))))

        xyz_ = np.asarray(xyz_trans - self.leg_origins[leg_select] + np.array(center_offset)).flatten()
        return self.ik_cal(xyz_, self.leg_direction[leg_select][0], self.leg_direction[leg_select][1])
    
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
    
    def plot_all_leg(self,limit=200):
        ax=self.plot_setup(limit)
        plt.show()

    def ctrl_leg(self,servo_angles_list,sec=1):
        cmd = "1a"+str(float(servo_angles_list[0][0]))+'b'+str(float(servo_angles_list[0][1]))+'c'+str(float(servo_angles_list[0][2])) +'d'+str(float(servo_angles_list[1][0]))+'e'+str(float(servo_angles_list[1][1]))+'f'+str(float(servo_angles_list[1][2]))+'g'+str(float(servo_angles_list[2][0]))+'h'+str(float(servo_angles_list[2][1]))+'i'+str(float(servo_angles_list[2][2]))+'j'+str(float(servo_angles_list[3][0]))+'k'+str(float(servo_angles_list[3][1]))+'l'+str(float(servo_angles_list[3][2]))+'m'
        print(cmd)
        print(servo_angles_list)
        self.seq.write(cmd.encode())
        time.sleep(sec)

    def forward(self, plan_list, sec=1):
        for plan in plan_list:
            self.ctrl_leg(plan,sec)
    
    def move_planning(self, waypoint_list):
        move_plan_list = [self.poses_calc_all_leg(waypoint) for waypoint in waypoint_list]
        # print(move_plan_list)
        return move_plan_list
    
    def init_pose(self):
        cmd="1a90.0b86.3c56.0d90.0e93.7f124.0g90.0h90.6i59.9j90.0k89.4l120.1m"
        self.seq.write(cmd.encode())
        time.sleep(1)

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


dog = leg_kinematics("COM8",115200)

xyz = [0,40,-100]

leg_0 = [0,40,-100]
leg_1 = [0,-40,-100]
leg_2 = [0,40,-100]
leg_3 = [0,-40,-100]
xyz_all = [leg_0,leg_1,leg_2,leg_3]

# dog.pose_calc_all(xyz_all)
# dog.ctrl_leg()

# 아래 위 무빙
# waypoints=[]
# for i in range(0,2):
#     leg_0 = [10, 40,-50]
#     leg_1 = [10,-40,-50]
#     leg_2 = [0, 40,-50]
#     leg_3 = [0,-40,-50]
#     waypoints.append([leg_0,leg_1,leg_2,leg_3])

#     leg_0 = [10,  40, -120]
#     leg_1 = [10, -40, -120]
#     leg_2 = [0,  40, -120]
#     leg_3 = [0, -40, -120]
#     waypoints.append([leg_0,leg_1,leg_2,leg_3])

forward_step_val = 80
waypoints=[]
# 초기 자세
leg_0 = [10,  40, -140]
leg_1 = [10, -40, -140]
leg_2 = [-10,  40, -140]
leg_3 = [-10, -40, -140]
waypoints.append([leg_0,leg_1,leg_2,leg_3])

for i in range(0,4):
    # 오른쪽 앞발, 왼쪽 뒷발 들어서서
    leg_0 = [10,  40, -140]  # 왼쪽 앞발
    leg_1 = [10, -40, -70]  # 오른쪽 앞발
    leg_2 = [-10,  40, -70]   # 왼쪽 뒷발
    leg_3 = [-10, -40, -140]   # 오른쪽 뒷발
    waypoints.append([leg_0,leg_1,leg_2,leg_3])

    # 오른쪽 앞발, 왼쪽 뒷발 앞으로 이동
    leg_0 = [10,  40, -140]  # 왼쪽 앞발
    leg_1 = [10 +forward_step_val, -40, -70]  # 오른쪽 앞발
    leg_2 = [-10  +forward_step_val,  40, -70]   # 왼쪽 뒷발
    leg_3 = [-10, -40, -140]   # 오른쪽 뒷발
    waypoints.append([leg_0,leg_1,leg_2,leg_3])

    # 오른쪽 앞발, 왼쪽 뒷발 내려놓기기
    leg_0 = [10,  40, -140]  # 왼쪽 앞발
    leg_1 = [10 +forward_step_val, -40, -140]  # 오른쪽 앞발
    leg_2 = [-10 +forward_step_val,  40, -140]   # 왼쪽 뒷발
    leg_3 = [-10, -40, -140]   # 오른쪽 뒷발
    waypoints.append([leg_0,leg_1,leg_2,leg_3])

    # 초기 자세에 왼쪽 앞발, 오른쪽 뒷발 들기
    leg_0 = [10,  40, -70]
    leg_1 = [10, -40, -140]
    leg_2 = [-10,  40, -140]
    leg_3 = [-10, -40, -70]
    waypoints.append([leg_0,leg_1,leg_2,leg_3])

    # 왼쪽 앞발, 오른쪽 뒷발 앞으로
    leg_0 = [10+forward_step_val,  40, -70]
    leg_1 = [10, -40, -140]
    leg_2 = [-10,  40, -140]
    leg_3 = [-10+forward_step_val, -40, -70]
    waypoints.append([leg_0,leg_1,leg_2,leg_3])

    # 왼쪽 앞발, 오른쪽 뒷발 내리기기
    leg_0 = [10+forward_step_val,  40, -140]
    leg_1 = [10, -40, -140]
    leg_2 = [-10,  40, -140]
    leg_3 = [-10+forward_step_val, -40, -140]
    waypoints.append([leg_0,leg_1,leg_2,leg_3])

# 초기 자세
leg_0 = [10,  40, -140]
leg_1 = [10, -40, -140]
leg_2 = [-10,  40, -140]
leg_3 = [-10, -40, -140]
waypoints.append([leg_0,leg_1,leg_2,leg_3])

plan = dog.move_planning(waypoints)
time.sleep(2)

dog.forward(plan,sec=1.1)
# time.sleep(3)
# dog.init_pose()

# print(dog.poses_calc_all_leg(xyz_all))
# print(plan)

# 이거 다리별로 앞으로 간거 뒤로 간거 위치를 만들어놓고 그걸 조합하는게 더 패턴 만들기 쉬울듯.
# 앞으로 가는 정도나 그런거도 변수로 넣고.
