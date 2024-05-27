import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
from image_geometry import PinholeCameraModel
import tf2_ros as tf
import numpy as np
import cv2

class Detector(Node):


    def __init__(self):
        super().__init__('minimal_publisher')
        self.bridge = CvBridge()
        self.Kx = 0.01
        self.Kw = 0.01
        self.targetWidth = 60
        self.cameraInfo = None
        self.cameraModel = PinholeCameraModel()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.callback, qos_profile=qos_policy)
        self.publishers_ = self.create_publisher(PoseStamped, '/goal_pose', 1)

        self.subscriberInfo = self.create_subscription(CameraInfo, '/camera/camera_info', self.saveCameraInfo, qos_profile=qos_policy)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer, self)
        self.targetGoalPosition = None
        
    def timer_callback(self):
            if self.targetGoalPosition is not None:
                self.publishers_.publish(self.targetGoalPosition)

                print(self.targetGoalPosition)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        mask = cv2.inRange(cv_image, (0, 230, 0), (30, 255, 30), cv_image)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if(len(contours) == 0):
            if(self.tf_buffer.can_transform("map", "camera_rgb_optical_frame", rclpy.time.Time())):
                p = PoseStamped()
                p.pose.position.x = 0.0
                p.pose.position.y = 0.0
                p.pose.position.z = 0.0
                p.header.stamp = rclpy.time.Time().to_msg()
                p.header.frame_id = 'camera_rgb_optical_frame'

                

                self.targetGoalPosition = self.tf_buffer.transform(p, "map")

                print("Turning")
                self.targetGoalPosition.pose.orientation.z += 0.1
            return
        

        boxes = [cv2.boundingRect(c) for c in contours] # Bounding boxes
        box = max(boxes, key=lambda x: x[2]) # Get the largest box
        x, y, w, h = box

        try:
            unitVector = self.cameraModel.projectPixelTo3dRay((x, y))
            p = PoseStamped()
            l = 70.53364/w
            p.pose.position.x = l*unitVector[0]
            p.pose.position.y = l*unitVector[1]
            p.pose.position.z = l*unitVector[2]
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            p.header.stamp = rclpy.time.Time().to_msg()
            p.header.frame_id = 'camera_rgb_optical_frame'


            self.targetGoalPosition = self.tf_buffer.transform(p, "map")
            # self.targetGoalPosition.pose.orientation.z += np.pi/2
            
        


        except Exception as e:
            print(e)
            print("Couldn't project pixel to 3d ray")
        
        # self.control(x-msg.width/2, w-self.targetWidth)

        cv2.imshow("Image", mask)
        cv2.waitKey(1)  # Wait for a key press to refresh the image window

    def saveCameraInfo(self, msg):
        self.cameraModel.fromCameraInfo(msg)

    def matrix_from_tf(self, tf_stamped):
        translation = [tf_stamped.transform.translation.x, tf_stamped.transform.translation.y, tf_stamped.transform.translation.z]
        rotation = [tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w]
        tf_matrix = tf.transformations.translation_matrix(translation) @ tf.transformations.quaternion_matrix(rotation)
        return tf_matrix






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Detector()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()