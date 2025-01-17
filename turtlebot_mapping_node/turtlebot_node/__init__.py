import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.client import GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry 
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist

import math
import time
import random

#total_map_length = len(self.map_data)
# self.total_pos_list = []
                # for map_index in range(total_map_length):
                #     if self.map_data[map_index] == 0:
                #         map_index_result = self.check_neighbor_pixel(map_index)  
                #         if map_index_result is True:
                #             x_pos = (map_index % self.width) * self.map_resolution + self.map_x
                #             y_pos = (map_index // self.width) * self.map_resolution + self.map_y
                #             pos_list = [x_pos, y_pos]
                #             self.total_pos_list.append(pos_list)
                            
                # self.total_pos_list_length = len(self.total_pos_list)
                # self.pos_list_index = 0
                # print(self.total_pos_list)
                # if self.total_pos_list_length == 0:
                #     self.get_logger().info('Mapping Finish') 
                # else:
                #     self.send_goal(self.total_pos_list)           
                            
                            
class turtlebot_node(Node):
    def __init__(self):
        super().__init__('move_node')
        self.nav_run_status = False
              
        # 맵 토픽 서브스크라이버
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_sub_callback, 10)
        
        # nav2 액션 클라이언트
        self.navigate_to_pose = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
    # 맵 토픽을 통한 자동 매핑    
    def map_sub_callback(self, msg):
        # 계산에 필요한 데이터 변수에 저장
        self.map_data = msg.data
        self.map_resolution = msg.info.resolution
        self.map_x = msg.info.origin.position.x
        self.map_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height   
        
        # nav2 액션이 동작 하지 않는 경우 실행      
        if self.nav_run_status is False:
            self.nav_run_status = True 
            self.map_index_check = False                           
            used_index = set()   
            mapping_finish_check_count = 0

            while self.map_index_check is False:
                self.chosen_map_index = random.randrange(len(self.map_data))
                if self.map_data[self.chosen_map_index] == 0 and self.chosen_map_index not in used_index:
                    map_index_result = self.check_neighbor_pixel(self.chosen_map_index)
                    
                    # 매핑 종료 로직    
                    mapping_finish_check_count += 1
                    print(f"mapping_finish_check_count: {mapping_finish_check_count}")
                    if mapping_finish_check_count > 100:
                        self.get_logger().info("Mapping Finish")
                        self.nav_run_status = True
                        return
                        
                    if map_index_result is True:
                        target_x_pos = (self.chosen_map_index % self.width) * self.map_resolution + self.map_x
                        target_y_pos = (self.chosen_map_index // self.width) * self.map_resolution + self.map_y
                        target_pos_list = [target_x_pos, target_y_pos]
                        self.map_index_check = True        
                used_index.add(self.chosen_map_index)
            self.send_goal(target_pos_list)

        # 선택한 지점 조건에 안맞을 시 취소
        elif self.nav_run_status is True:
            after_map_index_result = self.check_neighbor_pixel(self.chosen_map_index)
            if after_map_index_result is False:
                self.navigate_to_pose._cancel_goal_async(self.goal_handle)
                self.get_logger().info("Action Canceled")
                self.nav_run_status = False
    
    # 선택된 인덱스 주위의 픽셀 체크
    def check_neighbor_pixel(self, map_index):
        total = 0
        for x in range(-2, 3):
            for y in range(-2, 3):
                self.check_map_index = map_index + (x * self.width + y)
                if self.check_map_index >= 0 and self.check_map_index <= len(self.map_data):
                    total += self.map_data[self.check_map_index]

        check_value = total / 25            
        if check_value < 0:
            return True
        else:
            return False
    
    # nav2 서버에 목표 전달        
    def send_goal(self, pos_list):
        wait_count = 1
        while not self.navigate_to_pose.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self.get_logger().info('Navigate action server is not available.')
                return False
            wait_count += 1
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = pos_list[0]
        goal_msg.pose.pose.position.y = pos_list[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.send_goal_future = self.navigate_to_pose.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
        return True
        
    def navigate_to_pose_action_goal(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info("Action goal rejected.")
            return

        self.get_logger().info("Action goal accepted.")
        self.action_result_future = self.goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)
        
    def navigate_to_pose_action_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback
        # self.get_logger().info("Action feedback: {0}".format(action_feedback))

    def navigate_to_pose_action_result(self, future):
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Action succeeded!.")
        else:
            self.get_logger().info(f"Action failed with status: {action_status}")   
        self.nav_run_status = False                     
                
def main(args=None):
    rclpy.init(args=args)
    
    node = turtlebot_node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)   
    
    executor.spin()  
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
    
