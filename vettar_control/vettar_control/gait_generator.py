#!/usr/bin/env python3
# vettar_control/vettar_control/gait_generator.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import math

class BezierCurveGenerator:
    def __init__(self):
        """베지어 곡선 생성기 초기화"""
        pass
        
    def generate_bezier_curve(self, control_points, num_points=100):
        """
        주어진 제어점으로 베지어 곡선 생성
        
        Args:
            control_points: 제어점 리스트 (각 제어점은 [x, y, z] 형태)
            num_points: 곡선을 구성하는 점의 개수
            
        Returns:
            points: 곡선 상의 점들 (num_points x 3 배열)
        """
        n = len(control_points) - 1  # 베지어 곡선의 차수
        control_points = np.array(control_points)
        
        # 베지어 곡선 상의 점들 계산
        points = np.zeros((num_points, 3))
        for i in range(num_points):
            t = i / (num_points - 1)
            point = np.zeros(3)
            
            for j in range(n + 1):
                # 베지어 곡선 공식 적용: B(t) = Σ(j=0 to n) binom(n,j) * (1-t)^(n-j) * t^j * P_j
                coef = self._binomial_coefficient(n, j) * (1 - t) ** (n - j) * t ** j
                point += coef * control_points[j]
                
            points[i] = point
            
        return points
    
    def _binomial_coefficient(self, n, k):
        """이항 계수 계산 (nCk)"""
        if k < 0 or k > n:
            return 0
        if k == 0 or k == n:
            return 1
        
        # 파스칼의 삼각형을 이용한 계산
        res = 1
        for i in range(k):
            res = res * (n - i) // (i + 1)
        return res
    
    def create_walking_trajectory(self, stride_length, stride_height, step_width=0, num_points=50, lift_ratio=0.25):
        """
        워킹/크롤 보행을 위한 다리 궤적 생성
        
        Args:
            stride_length: 보폭(mm)
            stride_height: 들어올리는 높이(mm)
            step_width: 좌우 방향 보폭(mm), 기본값은 0
            num_points: 궤적 점의 개수
            lift_ratio: 전체 주기 중 들어올리는 단계의 비율 (0-1 사이 값)
            
        Returns:
            full_trajectory: 완전한 보행 주기의 궤적 점들
        """
        # 들어올리는 단계 (다리를 들어 앞으로 이동) 점의 개수 계산
        lift_points_count = int(num_points * lift_ratio)
        # 지지 단계 (지면에 닿은 상태로 뒤로 이동) 점의 개수 계산
        stance_points_count = num_points - lift_points_count
        
        # 들어올리는 단계: 반타원형 궤적을 위한 제어점 설정
        lift_control_points = [
            [0, 0, 0],                           # 시작점
            [stride_length/4, step_width/2, stride_height*0.8],  # 중간 제어점 1
            [stride_length/2, step_width, stride_height],       # 정점
            [3*stride_length/4, step_width/2, stride_height*0.8],  # 중간 제어점 2
            [stride_length, 0, 0]                # 끝점
        ]
        
        # 지지 단계: 땅에 붙어서 반대 방향으로 움직임
        stance_control_points = [
            [stride_length, 0, 0],           # 시작점
            [stride_length*0.75, 0, 0],      # 중간점 1
            [stride_length*0.5, 0, 0],       # 중간점 2
            [stride_length*0.25, 0, 0],      # 중간점 3
            [0, 0, 0]                        # 끝점
        ]
        
        # 베지어 곡선 생성
        lift_points = self.generate_bezier_curve(lift_control_points, lift_points_count)
        stance_points = self.generate_bezier_curve(stance_control_points, stance_points_count)
        
        # 전체 궤적 (들어올리는 단계 + 지지 단계)
        full_trajectory = np.vstack([lift_points, stance_points])
        
        return full_trajectory


class GaitGeneratorNode(Node):
    def __init__(self):
        super().__init__('gait_generator')
        
        # 매개변수 개별 선언
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        # 기본 다리 위치 선언
        self.declare_parameter('leg_position_FL', [95.0, 55.0, -150.0])
        self.declare_parameter('leg_position_FR', [95.0, -55.0, -150.0])
        self.declare_parameter('leg_position_BL', [-95.0, 55.0, -150.0])
        self.declare_parameter('leg_position_BR', [-95.0, -55.0, -150.0])
        
        # 다른 파라미터 선언
        self.declare_parameter('body_length', 190.0)  # mm
        self.declare_parameter('body_width', 110.0)   # mm
        self.declare_parameter('stride_length', 60.0)  # mm
        self.declare_parameter('stride_height', 30.0)  # mm
        self.declare_parameter('step_width', 0.0)      # mm
        self.declare_parameter('lift_ratio', 0.2)      # 전체 주기 중 들어올리는 단계의 비율
        self.declare_parameter('gait_cycle_time', 2.0)  # 초
        
        # 매개변수 가져오기
        self.update_rate = self.get_parameter('update_rate').value
        
        # 기본 다리 위치 설정
        fl_pos = self.get_parameter('leg_position_FL').value
        fr_pos = self.get_parameter('leg_position_FR').value
        bl_pos = self.get_parameter('leg_position_BL').value
        br_pos = self.get_parameter('leg_position_BR').value
        
        self.leg_positions = np.array([fl_pos, fr_pos, bl_pos, br_pos])
        
        self.body_length = self.get_parameter('body_length').value
        self.body_width = self.get_parameter('body_width').value
        self.stride_length = self.get_parameter('stride_length').value
        self.stride_height = self.get_parameter('stride_height').value
        self.step_width = self.get_parameter('step_width').value
        self.lift_ratio = self.get_parameter('lift_ratio').value
        self.gait_cycle_time = self.get_parameter('gait_cycle_time').value
        
        # 다리 ID 정의
        self.FL = 0  # Front Left
        self.FR = 1  # Front Right
        self.BL = 2  # Back Left
        self.BR = 3  # Back Right
        
        # 워킹 보행 위상: 항상 3개의 다리가 지면에 닿도록 함
        # FR -> BR -> FL -> BL 순서로 움직임
        self.phase = [0.5, 0.0, 0.75, 0.25]  # [FL, FR, BL, BR]의 위상 (0-1 사이 값)
        
        # 베지어 곡선 생성기 초기화
        self.curve_generator = BezierCurveGenerator()
        
        # 보행 관련 변수
        self.is_walking = False
        self.walk_direction = [0.0, 0.0, 0.0]  # [x, y, yaw] 방향
        self.current_time = 0.0
        self.num_points = int(self.update_rate * self.gait_cycle_time)  # 한 주기당 포인트 수
        
        # 궤적 캐시
        self.trajectories = None
        
        # Twist 메시지를 구독하여 로봇 동작 명령 수신
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 다리 위치를 발행할 퍼블리셔
        self.leg_positions_pub = self.create_publisher(
            Float32MultiArray,
            'leg_positions',
            10
        )
        
        # 다리 움직임 업데이트 타이머
        self.timer = self.create_timer(1.0/self.update_rate, self.update_leg_positions)
        
        self.get_logger().info('Gait Generator node started')
    
    def cmd_vel_callback(self, msg):
        """
        속도 명령을 받아 보행 방향과 상태 업데이트
        """
        # 선형 속도와 각속도를 보행 방향으로 변환
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        # 로봇이 움직이는지 확인
        if abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(angular_z) > 0.01:
            self.is_walking = True
            # 방향 설정 (x: 전진/후진, y: 좌/우, yaw: 회전)
            self.walk_direction = [linear_x, linear_y, angular_z]
            
            # 이동 속도에 따라 보폭 조정
            speed = math.sqrt(linear_x**2 + linear_y**2)
            max_speed = 1.0  # 최대 속도 기준값
            self.stride_length = self.get_parameter('stride_length').value * min(speed / max_speed, 1.0)
            
            # 방향에 따른 보폭 조정
            if linear_x < 0:  # 후진
                self.stride_length *= -1
            
            # 회전 시 옆으로 움직이는 폭 조정
            self.step_width = self.get_parameter('step_width').value * abs(angular_z)
            if angular_z > 0:  # 왼쪽으로 회전
                self.step_width *= -1
                
            # 궤적 무효화 (다음 업데이트에서 새로 생성)
            self.trajectories = None
            
            self.get_logger().debug(f'Walking command received. Direction: [{linear_x:.2f}, {linear_y:.2f}, {angular_z:.2f}]')
        else:
            self.is_walking = False
            self.get_logger().debug('Walking stopped')
    
    def generate_leg_trajectories(self):
        """
        현재 설정된 매개변수로 다리 궤적 생성
        """
        # 기본 궤적 생성
        basic_trajectory = self.curve_generator.create_walking_trajectory(
            abs(self.stride_length), 
            self.stride_height, 
            abs(self.step_width), 
            self.num_points, 
            self.lift_ratio
        )
        
        # 각 다리의 좌표계에 맞게 변환
        leg_trajectories = []
        for i in range(4):
            # 기본 위치를 기준으로 상대적인 위치 계산
            base_position = self.leg_positions[i].copy()
            
            # 기본 궤적 복사
            trajectory = np.copy(basic_trajectory)
            
            # 후진 시 방향 반전
            if self.stride_length < 0:
                trajectory[:, 0] *= -1
            
            # 좌/우 다리 방향 조정 (모든 다리에 적용)
            if i in [self.FR, self.BR]:  # 오른쪽 다리
                trajectory[:, 1] *= -1
            
            # 회전 시 추가 조정 (yaw에 따라)
            if self.walk_direction[2] != 0:
                # 회전 중심을 기준으로 다리 궤적 조정
                # (이 부분은 회전 동작을 더 정교하게 구현하려면 추가 개발 필요)
                pass
            
            # 각 다리의 기본 위치를 더함
            trajectory += base_position
            
            leg_trajectories.append(trajectory)
        
        return leg_trajectories
    
    def get_leg_position_at_time(self, leg_idx, t, trajectories):
        """
        주어진 시간 t에서 특정 다리의 위치 반환
        
        Args:
            leg_idx: 다리 인덱스 (0:FL, 1:FR, 2:BL, 3:BR)
            t: 정규화된 시간 (0-1 범위)
            trajectories: 다리 궤적
            
        Returns:
            position: 다리 위치 [x, y, z]
        """
        trajectory = trajectories[leg_idx]
        num_points = len(trajectory)
        
        # 다리의 위상 적용
        phase_t = (t + self.phase[leg_idx]) % 1.0
        
        # 궤적에서의 인덱스 계산
        idx = int(phase_t * num_points) % num_points
        
        return trajectory[idx]
    
    def update_leg_positions(self):
        """
        주기적으로 다리 위치를 업데이트하고 발행
        """
        if self.is_walking:
            # 궤적이 없으면 생성
            if self.trajectories is None:
                self.trajectories = self.generate_leg_trajectories()
            
            # 현재 시간에 따른 위치 계산
            t = (self.current_time % self.gait_cycle_time) / self.gait_cycle_time
            
            # 각 다리 위치 계산
            leg_positions_flat = []
            for i in range(4):
                leg_pos = self.get_leg_position_at_time(i, t, self.trajectories)
                # x, y, z 순서로 플랫 리스트에 추가
                leg_positions_flat.extend(leg_pos.tolist())
            
            # Float32MultiArray 메시지 생성 및 발행
            msg = Float32MultiArray()
            msg.data = leg_positions_flat  # [FL_x, FL_y, FL_z, FR_x, FR_y, FR_z, BL_x, BL_y, BL_z, BR_x, BR_y, BR_z]
            self.leg_positions_pub.publish(msg)
            
            # 시간 업데이트
            self.current_time += 1.0 / self.update_rate
        else:
            # 걷지 않는 경우, 기본 자세 유지
            leg_positions_flat = []
            for pos in self.leg_positions:
                leg_positions_flat.extend(pos)
            
            msg = Float32MultiArray()
            msg.data = leg_positions_flat
            self.leg_positions_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gait_generator = GaitGeneratorNode()
    
    try:
        rclpy.spin(gait_generator)
    except KeyboardInterrupt:
        pass
    finally:
        gait_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()