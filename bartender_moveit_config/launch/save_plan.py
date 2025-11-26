#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from datetime import datetime
import csv
import os

class MoveItPlanSaver(Node):
    def __init__(self):
        super().__init__('moveit_plan_saver')
        
        # RViz가 계획을 시각화할 때 쏘는 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.listener_callback,
            10)
        self.get_logger().info('waiting for plan... RViz에서 [Plan] 버튼을 눌러주세요!')

    def listener_callback(self, msg):
        self.get_logger().info('Plan received! Saving to CSV...')
        
        # 메시지 안에 계획된 경로가 들어있습니다.
        # 보통 첫 번째 궤적(trajectory[0])을 가져옵니다.
        traj = msg.trajectory[0]
        joint_traj = traj.joint_trajectory
        
        file_name = f'captured_plan_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        
        with open(file_name, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # 1. 헤더 저장 (관절 이름)
            header = ['time'] + list(joint_traj.joint_names)
            writer.writerow(header)
            
            # 2. 데이터 저장
            for point in joint_traj.points:
                # 시간 변환
                time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                # 위치(각도) 데이터
                row = [time_sec] + list(point.positions)
                writer.writerow(row)
                
        self.get_logger().info(f'Successfully saved to {os.getcwd()}/{file_name}')
        
        # 저장이 끝났으니 노드를 종료합니다.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    saver = MoveItPlanSaver()
    try:
        rclpy.spin(saver)
    except SystemExit:
        pass # 정상 종료
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()