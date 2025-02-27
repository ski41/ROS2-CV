#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int16

"""
Basic navigation demo to go to pose.
"""
class NavControl(Node):
    flag=1
    def __init__(self):
        super().__init__('Nav_control')
        # Subscribe to 'binary' topic
        self.subscription = self.create_subscription(
            Int16, 'binary', self.navigation, 10)
        self.nav_signal = self.create_publisher(Int16, 'nav_aruco', 10)

    def navigation(self, msg):
        # Instantiate BasicNavigator
        navigator = BasicNavigator()

        self.get_logger().info(f'Received binary message: {msg.data}')

        if(self.flag==1):
            # Set up the goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 0.405
            goal_pose.pose.position.y = 1.0
            goal_pose.pose.orientation.w = 1.0

            # Navigate to the goal pose
            navigator.goToPose(goal_pose)

            # Wait for navigation task to complete
            while not navigator.isTaskComplete():
                # Do something with the feedback
                feedback = navigator.getFeedback()
                if feedback:
                    print('Estimated time of arrival:', Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9, 'seconds.')

                    # Check if navigation task should be canceled based on feedback
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelTask()

            # Check the result of the navigation task
            result = navigator.getResult()
        
        if(self.flag == 0):
            result=TaskResult.SUCCEEDED

        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            #print('Starting spinning...')
            # Start spinning
            # Wait for 'binary' message to be 2 to stop spinning
            navigator.cancelTask()
            signal=Int16()
            signal.data=1
            self.nav_signal.publish(signal)
            
            
          
                		
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    exit(0)



def main():
    rclpy.init()
    node = NavControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
