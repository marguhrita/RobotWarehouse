from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.time import Time


rclpy.init()
nav = BasicNavigator()

def create_pose(x,y,z, orientation : Quaternion):
    # Create the PoseStamped object
    pose_stamped = PoseStamped()

    # Set the header
    pose_stamped.header.frame_id = "map"  # Replace with the desired frame ID
    pose_stamped.header.stamp = Time().to_msg()  # Current time (requires ROS clock)

    # Set the pose
    pose_stamped.pose = Pose(
        position=Point(x=1.0, y=2.0, z=0.0),  # Set the position (x, y, z)
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Set the orientation (x, y, z, w)
    )

    return pose_stamped

identity_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

goal_pose = create_pose(0.35, -0.624, 0, identity_quaternion)
init_pose = create_pose(0.42, 0.0273, 0, identity_quaternion)

nav.setInitialPose(init_pose)
nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

# ...

path = nav.getPath(init_pose, goal_pose)
smoothed_path = nav.smoothPath(path)

# ...

nav.goToPose(goal_pose)
while not nav.isNavComplete():
  feedback = nav.getFeedback()
  if feedback.navigation_duration > 600:
    nav.cancelTask()

# ...

result = nav.getResult()
if result == NavigationResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == NavigationResult.CANCELED:
    print('Goal was canceled!')
elif result == NavigationResult.FAILED:
    print('Goal failed!')