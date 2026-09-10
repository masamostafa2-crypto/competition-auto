import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        # Create a bridge to convert ROS Image messages into OpenCV images
        self.bridge = CvBridge()
        # Load the YOLOv8 small model. 
        self.model = YOLO("yolov8n.pt")

        # Subscribe to the camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/mono/image',
            self.process_image,
            10
        )

        # Publisher for the annotated (YOLO-processed) images
        self.viz_pub = self.create_publisher(Image, '/yolo/visual', 10)

        # Let the terminal know that the node has started successfully
        self.get_logger().info("YOLO visualizer started")

    def process_image(self, msg):
        # Convert the incoming ROS Image message to an OpenCV BGR frame
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')

        # Run YOLO on the frame to detect objects
        results = self.model(cv_image, verbose=False)

        # Draw bounding boxes and labels on the frame
        annotated = results[0].plot()

        # Convert the annotated OpenCV image back into a ROS Image message
        msg_out = self.bridge.cv2_to_imgmsg(annotated, 'mono8')

        # Publish the annotated image to the /yolo/visualization topic
        self.viz_pub.publish(msg_out)


def main(args=None):
    # Initialize ROS 2 and create the node
    rclpy.init(args=args)
    node = YoloDetector()

    # Keep the node alive so it can keep processing incoming frames
    rclpy.spin(node)

    # Clean shutdown when you stop the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()