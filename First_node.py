#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int16,Int8
import time


"""
Basic navigation demo to go to pose.
"""
class NavArucoControl(Node):
    
    def __init__(self):
        super().__init__('Nav_Aruco_Control')
        # Subscribe to 'binary' topic
        self.subscription = self.create_subscription(
            Int16, 'nav_aruco', self.navigation_aruco, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_logic = self.create_publisher(Int8, 'logic_message', 10)
    def navigation_aruco(self, msg):
        # Instantiate BasicNavigator
        
        self.sub_ = self.create_subscription(PoseStamped, 'map_aruco_pose',self.nav,10)

        

    def nav(self,msgs):

        print("Message recieved")
        position_x = msgs.pose.position.x 
        position_y = msgs.pose.position.y
        position_z = msgs.pose.position.z
        orientation_w = msgs.pose.orientation.w
        
        navigator = BasicNavigator()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.orientation.w = orientation_w

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
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            #self.driveOnHeading(dist=0.15, speed=0.25, time_allowance=10)
            self.move_forward(0.9,0.3)
            #navigator.cancelTask()
            # Start spinning
            # Wait for 'binary' message to be 2 to stop spinning

            #publish
            print("Moved inside, lifting")
            data_logic = Int8()
            data_logic.data=0
            self.publisher_logic.publish(data_logic)
            print("lifted")
            time.sleep(12)
            navigator.spin(spin_dist=3.14, time_allowance=10)
            time.sleep(12)
            print("Moved forward")
            self.move_forward(1.5,0.3)
            data_logic.data=1
            self.publisher_logic.publish(data_logic)
            self.move_forward(-0.9,0.3)

            # result=navigator.getResult()

            # if(result == TaskResult.SUCCEEDED):
            #     print("spin success")
            #     data_logic = Int8()
            #     data_logic.data=0
            #     self.publisher_logic(data_logic)

                    
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        exit(0)


    def move_forward(self, distance, speed):
            twist_msg = Twist()
            twist_msg.linear.x = speed  # Set linear velocity to move forward
            twist_msg.angular.z = 0.0  # No angular velocity (no turning)

            start_time = time.time()
            current_distance = 0.0

            while current_distance < distance:
                # Publish velocity command
                self.publisher.publish(twist_msg)

                # Calculate distance moved since start
                elapsed_time = time.time() - start_time
                current_distance = speed * elapsed_time

                # Sleep for a short duration to control publishing rate
                time.sleep(0.1)

            # Stop the robot after reaching the desired distance
            twist_msg.linear.x = 0.0
            self.publisher.publish(twist_msg)  

      
        

def main():
    rclpy.init()
    node = NavArucoControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
