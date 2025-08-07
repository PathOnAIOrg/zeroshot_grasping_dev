#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TopDownCameraNode(Node):
    def __init__(self):
        super().__init__('topdown_camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/topdown_camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Top-down camera node started')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Add text overlay
            cv2.putText(cv_image, 'UR5 Top-Down View', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Display the image
            cv2.imshow("UR5 Top-Down Camera", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TopDownCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()