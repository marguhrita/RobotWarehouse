from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import subprocess
import time
from util import RobotStatePublisher, RobotState
import os


class Bot():
    def __init__(self : object, initial_position : tuple[float, float, float], delivery_position : tuple[float,float,float], namespace : str, publisher : RobotStatePublisher):

        self.namespace = namespace
        self.delivery_pose = delivery_position
        self.initial_pos = initial_position

        # Start nav
        self.nav2_start()
      
        print(f"namespace:{self.namespace}")
        
        self.pub = publisher


    def set_initial_pose(self, pose : tuple[list,list,list]):
        goal_pose = f"{{header:{{stamp: {{sec: 0, nanosec: 0}}, frame_id: map}}, pose: {{position: {{x: {pose[0]}, y: {pose[1]}, z: {pose[2]}}}, orientation:{{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}}}}"
        command = ["ros2", "topic", "pub", "--once", "--qos-reliability", "reliable", f"/{self.namespace}/initialpose", "geometry_msgs/PoseWithCovarianceStamped", goal_pose]
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("Init pose set")

    def nav2_start(self):
        print(f"Launching nav for robot: {self.namespace}")
        # Run the ros2 launch command
        env = os.environ.copy()  # Copy current environment variables
        env["QT_QPA_PLATFORM"] = "xcb"  # Force Qt to use X11 instead of Wayland

        process = subprocess.Popen(
            ['ros2', 'launch', "robot_w", "multi_robot.launch.py", f"namespace:={self.namespace}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env
        )



        for line in process.stdout:
            print("Output:", line.strip())

            if b"active" in line.strip():
                self.set_initial_pose(self.initial_pos)
            

        return process
    


    def go_to_initial_pose(self):
        self.navigate_to_position(self.initial_pos, False)

    def get_pose_stamped(self, pos : tuple[float, float, float]) -> PoseStamped:

        x, y, z = pos
        # Set initial pose!
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = x
        # initial_pose.pose.position.y = y
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 1.0

        #return initial_pose
    
    def fetch_item(self, goal_pose : tuple[float, float, float]):
        self.navigate_to_position(goal_pose, True)



    def navigate_to_position(self, x,y,z, fetching : bool = False):
        print(f"namespace:{self.namespace}")


        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # goal_pose.pose.position.x = float(x)
        # goal_pose.pose.position.y = float(y)
        # goal_pose.pose.orientation.w = 1.0
        # goal_pose.pose.orientation.z = 0.0

        goal_pose = f"pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation:{{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}}}}"

        command = ["ros2", "action", "send_goal",
                    f"/{self.namespace}/navigate_to_pose",
                    "nav2_msgs/action/NavigateToPose",
                    goal_pose]

        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        print(command)
        for line in process.stdout:
            if line.strip().startswith("Goal accepted"):
                self.pub.publish(self.namespace, 2)
            elif line.strip().startswith("Goal finished"):
                self.pub.publish(self.namespace, 3)

            print("Output:", line.strip())

            
        
            
            
