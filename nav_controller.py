from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import subprocess
import time

class Bot():

    def __init__(self : object, initial_position : tuple[float, float, float], namespace : str):
        rclpy.init()
        self.navigator = BasicNavigator()
        
        # Set initial pose
        initial_pose = self.get_pose_stamped(initial_position)
        self.navigator.setInitialPose(initial_pose)
        
        #  navigator.lifecycleStartup()

        #autostart nav2
        #self.nav2_process = self.nav2_autostart("map.yaml")
        self.navigator.waitUntilNav2Active()

        # Load Map
        self.navigator.changeMap('map.yaml')

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
        x,y,z = pos

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.orientation.z = 0.0


        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()


        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        #self.navigator.lifecycleShutdown()
            
        
            
