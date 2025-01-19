from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

class Bot():

    def __init__(self : object, initial_position : tuple[float, float, float]):
        rclpy.init()
        self.navigator = BasicNavigator()
        
        # Set initial pose
        initial_pose = self.get_pose_stamped(initial_position)
        self.navigator.setInitialPose(initial_pose)
        
        #  navigator.lifecycleStartup()
        self.navigator.waitUntilNav2Active()

        # Load Map
        self.navigator.changeMap('map.yaml')


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
    
    def navigate_to_position(self, pos : tuple[float, float, float]):
        
        
    
        
