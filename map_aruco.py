
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int16

from geometry_msgs.msg import PoseStamped


from scipy.spatial.transform import Rotation

import numpy as np

def quat_rot(quaternion):
    
    quaternion /= np.linalg.norm(quaternion)

    # Create a Rotation object from the quaternion
    rotation = Rotation.from_quat(quaternion)

    # Get the rotation matrix
    rotation_matrix = rotation.as_matrix()
    
    return rotation_matrix

def rot_quat(rotation_matrix):
    r = Rotation.from_matrix(rotation_matrix[0:3, 0:3])
    quat = r.as_quat()
    return quat

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')
        self.bin_topic = self.create_publisher(Int16, 'binary', 10)
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'turtle1').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PoseStamped, 'map_aruco_pose', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tfbroadcaster = TransformBroadcaster(self)

        # Call on_timer function every second
        self.timer = self.create_timer(1, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        #map_camera
        from_frame_rel = 'map'
        to_frame_rel = 'camera_link_optical'

        #camera_aruco
        from_frame_rel1 = 'camera_link_optical'
        to_frame_rel1 = 'aruco_marker'
        msg = Int16()
       
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                        from_frame_rel,
                        to_frame_rel,
                        rclpy.time.Time())
            aruco_transform = self.tf_buffer.lookup_transform(
                        from_frame_rel1,
                        to_frame_rel1,
                        rclpy.time.Time())

            self.get_logger().info('Transform from "map" to "cameralink":')
            # self.get_logger().info(f'Translation: (x: {transform.transform.translation.x:.4f}, y: {transform.transform.translation.y:.4f}, z: {transform.transform.translation.z:.4f})')
            # self.get_logger().info(f'Translation: (x: {transform.transform.rotation.x:.4f}, y: {transform.transform.rotation.y:.4f}, z: {transform.transform.rotation.z:.4f}, w: {transform.transform.rotation.w:.4f} )')

            t=transform.transform.translation
            r=transform.transform.rotation
            tvec = np.array([t.x,t.y,t.z])
            rvec = np.array([r.x,r.y,r.z,r.w])

            rot_mat=quat_rot(rvec)
            map_camera=np.zeros([4,4])
            map_camera[3,3]=1
            map_camera[0:3,0:3]=rot_mat
            map_camera[0:3,3]=tvec

            at=aruco_transform.transform.translation
            ar=aruco_transform.transform.rotation
            atvec = np.array([at.x,at.y,at.z])
            arvec = np.array([ar.x,ar.y,ar.z,ar.w])

            self.get_logger().info(f'Translation: (x: {at.x:.4f}, y: {at.y:.4f}, z: {at.z:.4f})')

            aruco_rot_mat=quat_rot(arvec)
            camera_aruco=np.zeros([4,4])
            camera_aruco[3,3]=1
            camera_aruco[0:3,0:3]=aruco_rot_mat
            camera_aruco[0:3,3]=atvec

            tt=tvec+atvec
            rr=rvec+arvec
            # R_mc, t_mc = map_camera[:3, :3], map_camera[:3, -1]  # Extract rotation and translation from T_mc
            # R_ca, t_ca = camera_aruco[:3, :3], camera_aruco[:3, -1]  # Extract rotation and translation from T_ca

            # # Combined rotation
            # R_ma = R_mc + R_ca

            # # Combined translation
            # t_ma = t_ca + t_mc

            # # Combine rotation and translation into T_ma
            # T_ma = np.concatenate((R_ma, t_ma[:, np.newaxis]), axis=1)

            # Calculate inverse
# T_AC = np.linalg.inv(T_CA)

# # Calculate Map to Aruco transform
# T_MA = np.dot(T_MB, T_AC)
            #aruco_camera = np.linalg.inv(camera_aruco)
            map_aruco = np.dot(map_camera,camera_aruco)
            aruco_rack=np.array([
                [1,0,0,2.5],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]
            ])
            #print(camera_aruco)
            #print(map_camera)
            map_rack=np.dot(map_aruco,aruco_rack)
            q=rot_quat(map_rack)
            # map_aruco_q = np.array([q[0],q[1],q[2],q[3]])
            #map_aruco_q = np.array(q)
            #self.get_logger().info(f'Translation: (x: {q[0]:.4f}, y: {q[1]:.4f}, z: {q[2]:.4f}, w: {q[3]:.4f})')
            #print()

            tb = TransformStamped()
            tb.header.stamp = self.get_clock().now().to_msg()
            tb.header.frame_id = 'map'
            tb.child_frame_id = 'aruco_marker'
            
            # Store the translation (i.e. position) information
            xa=map_rack[0][3]
            yb=map_rack[1][3]
            zc=map_rack[2][3]
            tb.transform.translation.x = xa
            tb.transform.translation.y = yb
            tb.transform.translation.z = zc

            e = q[0] 
            f = q[1] 
            g = q[2] 
            h = q[3] 
            


            tb.transform.rotation.x = e
            tb.transform.rotation.y = f
            tb.transform.rotation.z = g
            tb.transform.rotation.w = h

            # pose_stamped_msg = PoseStamped()
            # pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            # pose_stamped_msg.header.frame_id = 'map'  # Set the frame ID
            # pose_stamped_msg.pose.position.x = tt[0]  # Set X position
            # pose_stamped_msg.pose.position.y = tt[1] # Set Y position
            # pose_stamped_msg.pose.position.z = tt[2] # Set Z position
            # pose_stamped_msg.pose.orientation.x = rr[0]  # Set X orientation
            # pose_stamped_msg.pose.orientation.y = rr[1]  # Set Y orientation
            # pose_stamped_msg.pose.orientation.z = rr[2]  # Set Z orientation
            # pose_stamped_msg.pose.orientation.w = rr[3]  # Set W orientation
            # self.publisher_.publish(pose_stamped_msg)
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            pose_stamped_msg.header.frame_id = 'map'  # Set the frame ID
            pose_stamped_msg.pose.position.x = xa  # Set X position
            pose_stamped_msg.pose.position.y = yb # Set Y position
            pose_stamped_msg.pose.position.z = zc # Set Z position
            pose_stamped_msg.pose.orientation.x = e  # Set X orientation
            pose_stamped_msg.pose.orientation.y = f  # Set Y orientation
            pose_stamped_msg.pose.orientation.z = g  # Set Z orientation
            pose_stamped_msg.pose.orientation.w = h  # Set W orientation
            self.publisher_.publish(pose_stamped_msg)

            print(f"{xa} {yb} {zc}")


  
          # Send the transform
            msg.data=1
            self.bin_topic.publish(msg)
            self.tfbroadcaster.sendTransform(tb)
            
        except TransformException as ex:
            msg.data=0
            self.bin_topic.publish(msg)
            self.get_logger().info(
                        f'Could not transform {to_frame_rel1} to {from_frame_rel1}: {ex}')



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
