import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

def compute(frame):
    ARUCO_DICT = {
	    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    }

# Camera calibration parameters
    aruco_type = "DICT_4X4_50"
    camera_matrix = np.array([[647.32265,   0.     , 313.89857],
           [0.     , 647.37216, 240.36244],
           [0.     ,   0.     ,   1.     ]])*1.8*2.37
        
    dist_coeffs = np.array([0.051658, 0.244366, 0.011772, 0.002966, 0.000000])

#     # ArUco dictionary
#     arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
#     arucoParams =  cv2.aruco.DetectorParameters()
#     detector = cv2.aruco.ArucoDetector(arucoDict,arucoParams)

#     #while True:
# # Read a frame from the webcam
    

#         # Detect ArUco markers
#     corners, ids, rejectedImgPoints = detector.detectMarkers(frame)
        
#     coordinates=[]
        
#         # If markers are detected, estimate their pose
#     if len(corners) > 0:
#             rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.103, camera_matrix, dist_coeffs)
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    arucoParams = cv2.aruco.DetectorParameters_create()
  

# while True:
    # Read a frame from the webcam

    # Detect ArUco markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    coordinates = []

    # If markers are detected, estimate their pose
    if len(corners) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.103, camera_matrix, dist_coeffs)
            translation_vector = tvecs[0]
            rotation_vector = rvecs[0]

            # Calculate distance
            distance = np.linalg.norm(translation_vector)

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

            # Calculate Euler angles from rotation matrix
            angles_rad = cv2.RQDecomp3x3(rotation_matrix)[0]
            angles_deg = np.degrees(angles_rad)

            coordinates.append(tvecs)
            coordinates.append(rvecs)
            # Draw the axis and tag ID on the image
            for i in range(len(ids)):
                org=(tuple(corners[i][0][0].astype(int)))
                org1=(tuple(corners[i][0][2].astype(int)))
                org3=(tuple(corners[i][0][3].astype(int)))

                strng=tvecs.flatten()*100
                angle=np.degrees(np.arctan(strng[0]/strng[2]))
                
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i, :, :], tvecs[i, :, :],0.103)
                #print(f"{i} {ids[i][0]} {corners[i][0][0]}")
                cv2.putText(frame, str(ids[i][0]), org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, str(np.round(strng,3)), org1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.putText(frame, str(np.round(angle,3)), org3, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return coordinates


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
    self.declare_parameter("aruco_marker_name", "aruco_marker")
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value
    
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.tfbroadcaster = TransformBroadcaster(self)  
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    
      current_frame = self.br.imgmsg_to_cv2(data)
      coordinates=compute(current_frame)


      if(len(coordinates)!=0):
          tvecs=coordinates[0]
          rvecs=coordinates[1]
      #    msg.x=coordinates[0]
      #    msg.y=coordinates[1]
      #    msg.z=coordinates[2]
      #    self.publisher_.publish(msg)
          t = TransformStamped()
          t.header.stamp = self.get_clock().now().to_msg()
          t.header.frame_id = 'camera_link_optical'
          t.child_frame_id = self.aruco_marker_name
        
          # Store the translation (i.e. position) information
          xa=tvecs[0][0][0]
          yb=tvecs[0][0][1]
          zc=tvecs[0][0][2]
          t.transform.translation.x = xa
          t.transform.translation.y = yb
          t.transform.translation.z = zc
  
          # Store the rotation information
          rotation_matrix = np.eye(4)
          rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0][0]))[0]
          r = R.from_matrix(rotation_matrix[0:3, 0:3])
          quat = r.as_quat()

          # q = np.array([quat[0], quat[1], quat[2], quat[3]])
          r2 = R.from_quat(quat)
          rotation_matrix2 = r2.as_matrix()

          # Define rotation angles for 90 degrees about X-axis, Y-axis, and Z-axis
          roll_angle = -90.0  # Rotation about X-axis
          pitch_angle = 0.0  # Rotation about Y-axis
          yaw_angle = 90.0    # Rotation about Z-axis

          # Apply rotations to the rotation matrix
          rotation_matrix2 = np.dot(rotation_matrix2, R.from_euler('x', roll_angle, degrees=True).as_matrix())
          rotation_matrix2 = np.dot(rotation_matrix2, R.from_euler('y', pitch_angle, degrees=True).as_matrix())
          rotation_matrix2 = np.dot(rotation_matrix2, R.from_euler('z', yaw_angle, degrees=True).as_matrix())

          # Convert the modified rotation matrix back to a quaternion
          modified_r = R.from_matrix(rotation_matrix2)
          modified_quat = modified_r.as_quat()

          
          # Quaternion format     
          e = modified_quat[0] 
          f = modified_quat[1] 
          g = modified_quat[2] 
          h = modified_quat[3] 
        


          t.transform.rotation.x = e
          t.transform.rotation.y = f
          t.transform.rotation.z = g
          t.transform.rotation.w = h
  
          # Send the transform
          self.tfbroadcaster.sendTransform(t)
      
      # Display image
      cv2.imshow("camera", current_frame)
      
      cv2.waitKey(1)

  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
