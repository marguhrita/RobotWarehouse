from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import subprocess
import time
from util import RobotStatePublisher, RobotState


class Bot():
    def __init__(self : object, initial_position : tuple[float, float, float], namespace : str):
        #rclpy.init()
        self.navigator = BasicNavigator(namespace)
        print("hihi")
        self.namespace = namespace
        
        # Set initial pose
        self.initial_pos = initial_position
        initial_pose_stamped = self.get_pose_stamped(initial_position)
        
        #self.navigator.lifecycleStartup()

        #autostart nav2
        #self.nav2_process = self.nav2_autostart("map.yaml")
        #self.navigator.waitUntilNav2Active()
        print("oop")

        self.navigator.setInitialPose(initial_pose_stamped)
        print("hi?")
        self.pub = RobotStatePublisher()
        
        # Load Map
        #self.navigator.changeMap('map.yaml')

    def nav2_autostart(self, map_path : str):
        # Run the ros2 launch command
        process = subprocess.Popen(
            ["gnome-terminal", "--", 'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', f'map:={map_path}'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        print("Nav2 starting..")
        #time.sleep(5)


        return process
    

    def end_nav2_process(self):
        self.nav2_process.terminate()

    def go_to_initial_pose(self):
        self.navigate_to_position(self.initial_pos, False)

    def get_pose_stamped(self, pos : tuple[float, float, float]) -> PoseStamped:

        x, y, z = pos
        # Set initial pose!
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0

        return initial_pose
    
    def fetch_item(self, goal_pose : tuple[float, float, float]):
        self.navigate_to_position(goal_pose, True)



    def navigate_to_position(self, x,y,z, fetching : bool = False):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.orientation.z = 0.0

        print("!")

        goal_pose = f"pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation:{{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}}}}"

        command = ["ros2", "action", "send_goal",
                    f"/{self.namespace}/navigate_to_pose",
                    "nav2_msgs/action/NavigateToPose",
                    goal_pose]

        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        print(command)
        for line in process.stdout:
            if line.strip().startswith("Goal accepted")
            # You can parse or print each line of output here
            print("Output:", line.strip())

            
        
            
            
