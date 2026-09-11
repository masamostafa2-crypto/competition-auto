import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from detect_interfaces.action import Detect

TIMEOUT = 30.0


class Compete(Node):
    def __init__(self):
        super().__init__('compete')

        # Just used as a "is the robot alive" sanity check at startup --
        # no longer used for distance control.
        self.ultra_seen = False
        self.create_subscription(Float32, '/ultrasonic_distance', self.ultra_cb, 10)

        self.box_seen = False
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        self.target_class = "suitcase"  # set to your real target class
        self.create_subscription(Image, '/mono/image', self.image_cb, 10)

        self.client = ActionClient(self, Detect, 'target_distance')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angular_speed = 2.0  # rad/s -- constant turn speed, tune for your robot

    def ultra_cb(self, msg):
        self.ultra_seen = True

    def image_cb(self, msg):
        if self.box_seen:
            return  # already found it, stop spending CPU on inference
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        results = self.model(cv_image, verbose=False)
        for box in results[0].boxes:
            name = results[0].names[int(box.cls[0])]
            conf = float(box.conf[0])
            if name == self.target_class and conf > 0.6:
                self.box_seen = True
                break

    def wait_until(self, check_fn, timeout):
        """Spin the node while waiting for check_fn() to become True."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if check_fn():
                return True
        return False

    def rotate(self, degrees):
        """Positive degrees = turn left (CCW), negative = turn right (CW)."""
        angle_rad = math.radians(degrees)
        duration = abs(angle_rad) / self.angular_speed

        cmd = Twist()
        cmd.angular.z = self.angular_speed if degrees > 0 else -self.angular_speed

        self.get_logger().info(f'Rotating {degrees} degrees ({duration:.2f}s)')
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)

        self.cmd_vel_pub.publish(Twist()) 

    def send_goal(self, distance):
        """distance = metres to drive forward (negative = backward)."""
        self.client.wait_for_server()

        goal = Detect.Goal()
        goal.target_distance = distance

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def compete(self):
        self.get_logger().info('Waiting for the robot to start...')
        if not self.wait_until(lambda: self.ultra_seen, TIMEOUT):
            self.get_logger().error('No /ultrasonic_distance -- robot did not start.')
            return False

        # step 1: rotate to face the search direction
        self.get_logger().info('Step 1: rotate to the left')
        self.rotate(90)

        # step 2: wait until YOLO confirms the box before continuing
        self.get_logger().info('Looking for box...')
        if not self.wait_until(lambda: self.box_seen, TIMEOUT):
            self.get_logger().error('Box never detected.')
            return False

        # step 3-N: drive forward in stages -- fill in your real distances
        distances = [1.64, 1.25, 1.0]
        for i, d in enumerate(distances, start=1):
            self.get_logger().info(f'Step {i}: drive {d} m')
            if not self.send_goal(d):
                self.get_logger().error(f'Step {i} failed. Aborting.')
                return False

        # final rotate -- replace 90 with your actual required angle
        self.get_logger().info('Final rotate')
        self.rotate(90)

        # final approach
        self.get_logger().info('Final drive: 5.0 m')
        if not self.send_goal(5.0):
            self.get_logger().error('Final drive failed. Aborting.')
            return False

        self.get_logger().info('Competition complete.')
        return True


def main():
    rclpy.init()
    node = Compete()
    node.compete()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
