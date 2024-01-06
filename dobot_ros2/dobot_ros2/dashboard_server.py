import threading
from api.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
import rclpy as rp
import time 
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from dobot_ros2_interface.srv import Dashboard
from dobot_ros2_interface.action import Move

from rcl_interfaces.msg import SetParametersResult

class DashboardServer(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.server = self.create_service(Dashboard, 
                              'dashboard_server', self.callback_service)
        
        self.action_server = ActionServer(
            self,
            Move, 
            'move_goal_server', 
            self.execute_callback)
        
        self.ip = "192.168.1.6"              # Robot의 IP 주소
        self.gripper_p = 1  
        self.dashboard_p = 29999
        self.move_p = 30003
        self.feed_p = 30004         

        self.declare_parameter('speed_value', 10)  # 로봇 속도 (1~100 사이의 값 입력)
        self.speed_value = self.get_parameter('speed_value').value

        self.connect_robot() # 로봇 연결    
    
        self.current_actual = None

        self.add_on_set_parameters_callback(self.parameters_callbacks)

    def get_feed(self, feed: DobotApi):
        hasRead = 0

        while True:
            data = bytes()
            while hasRead < 1440:
                temp = feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0
            a = np.frombuffer(data, dtype=MyType)

            if hex((a['test_value'][0])) == '0x123456789abcdef':
                self.current_actual = a["tool_vector_actual"][0]     # Refresh Properties
            time.sleep(0.001)
    
    def connect_robot(self):
        self.get_logger().info("연결 설정 중...")
        try:
            self.dashboard = DobotApiDashboard(self.ip, self.dashboard_p)
            self.move = DobotApiMove(self.ip, self.move_p)
            self.feed = DobotApi(self.ip, self.feed_p)
            self.get_logger().info("연결 성공!!")
        
            feed_thread = threading.Thread(target=self.get_feed, args=(self.feed,))
            feed_thread.setDaemon(True)
            feed_thread.start()

        except Exception as e:
            self.get_logger().info("연결 실패")
            raise e
        
    def robot_clear(self, dashboard : DobotApiDashboard):
        dashboard.ClearError()

    def gripper_DO(self, dashboard : DobotApiDashboard, index, status):
        dashboard.ToolDO(index, status)
    
    def get_Pose(self, dashboard : DobotApiDashboard):
        dashboard.GetPose()
        
    def run_point(self, move: DobotApiMove, point_list: list):
        move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])

    def robot_speed(self, dashboard : DobotApiDashboard, speed_value):
        dashboard.SpeedFactor(speed_value)

    def callback_service(self, request, response):
        if request.req_str == 'power on':
            self.dashboard.EnableRobot()
            self.robot_speed(self.dashboard, self.speed_value)
            time.sleep(0.1)
            response.res_str = 'power on'
            
        elif request.req_str == 'power off':
            self.dashboard.DisableRobot()
            time.sleep(0.1)
            response.res_str = 'power off'

        elif request.req_str == 'clear error':
            self.dashboard.DisableRobot()
            time.sleep(0.3)
            self.robot_clear(self.dashboard)
            time.sleep(0.3)
            self.dashboard.EnableRobot()
            self.robot_speed(self.dashboard, self.speed_value)
            time.sleep(0.3)
            response.res_str = 'clear error'
        
        #gripper
        
        elif request.req_str == 'gripper on':
            self.gripper_DO(self.dashboard, self.gripper_p, 1)
            time.sleep(0.1)
            response.res_str = 'gripper on'
        
        elif request.req_str == 'gripper off':
            self.gripper_DO(self.dashboard, self.gripper_p, 0)
            time.sleep(0.1)
            response.res_str = 'gripper off'

        else:
            self.get_logger().info('Wrong Command!')
            response.res_str = "Wrong Command!"
        
        self.get_logger().info(response.res_str)
        return response
        

    def execute_callback(self, goal_handle):
        goal_x = goal_handle.request.goal_pose.position.x
        goal_y = goal_handle.request.goal_pose.position.y
        goal_z = goal_handle.request.goal_pose.position.z
        goal_w = goal_handle.request.goal_pose.orientation.w
        goal_pose = [goal_x, goal_y, goal_z, goal_w]

        self.get_logger().info(f"Move to goal{goal_pose}")
        feedback_msg = Move.Feedback()
        
        self.run_point(self.move, goal_pose)
        current_pose = self.current_actual
        
        time.sleep(0.1)
        
        while True:
            self.get_logger().info(f"current_pose is {current_pose}")
            current_pose = self.current_actual
            feedback_msg.current_pose.position.x = current_pose[0]
            feedback_msg.current_pose.position.y = current_pose[1]
            feedback_msg.current_pose.position.z = current_pose[2]
            feedback_msg.current_pose.orientation.w = current_pose[3]

            goal_handle.publish_feedback(feedback_msg)

            result = [a - b for a, b in zip(goal_pose, current_pose)]
            if all(abs(diff) <= 0.1 for diff in result): # 골 좌표와 현재 좌표 차이가 0.1이하 일때
                break
            time.sleep(1)
            
            
        goal_handle.succeed()
        self.get_logger().info(f"goal succeed")
        
        result = Move.Result()
        result.result_str = "succeed"
        return result
    
    def parameters_callbacks(self, params):
        for param in params: 
            if param.name == 'speed_value':
                if param.value <= 100 and param.value >= 0:
                    self.speed_value = param.value
                    self.robot_speed(self.dashboard, self.speed_value)
                    self.get_logger().info(f"set robot speed {self.speed_value}")
                else:
                    self.get_logger().info("speed value is 0 ~ 100")
            
        return SetParametersResult(successful=True)

    
def main(args=None):
    rp.init(args=args)
    dsbd = DashboardServer()
    rp.spin(dsbd)
    rp.shutdown()

if __name__ == '__main__':
    main()