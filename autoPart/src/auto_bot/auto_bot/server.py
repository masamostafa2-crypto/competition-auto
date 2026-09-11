import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# Replace 'your_pkg' with the package that builds Detect.action
from detect_interfaces.action import Detect


class YoloActionTrigger(Node):
    def __init__(self):
        super().__init__('yolo_action_trigger')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        # 'suitcase' is the closest built-in COCO class to "box".
        # If you need real cardboard boxes, train/use a custom model
        # and set this to that model's class name instead.
        self.target_class = "suitcase"
        self.confidence_threshold = 0.6

        cb_group = ReentrantCallbackGroup()

        self.image_sub = self.create_subscription(
            Image, '/mono/image', self.process_image, 10,
            callback_group=cb_group)
        self.viz_pub = self.create_publisher(Image, '/yolo/visual', 10)

        self._action_client = ActionClient(
            self, Detect, 'target_distance', callback_group=cb_group)

        # How far the robot should stop from the box, in metres
        self.stop_distance = 0.07

        # Debounce so we don't spam a new goal every single frame
        # while the target stays in view.
        self._goal_active = False

        self.get_logger().info("YOLO action trigger started")

    def process_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        results = self.model(cv_image, verbose=False)
        annotated = results[0].plot()
        msg_out = self.bridge.cv2_to_imgmsg(annotated, 'mono8')
        self.viz_pub.publish(msg_out)

        box_detected = self._check_for_target(results[0])

        if box_detected and not self._goal_active:
            self.trigger_action()
        elif not box_detected:
            # Reset once the target leaves the frame, so a future
            # re-appearance can trigger the action again.
            self._goal_active = False

    def _check_for_target(self, result):
        names = result.names
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if names[cls_id] == self.target_class and conf >= self.confidence_threshold:
                return True
        return False

    def trigger_action(self):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Action server not available")
            return

        self._goal_active = True

        goal_msg = Detect.Goal()
        goal_msg.target_distance = self.stop_distance

        self.get_logger().info(
            f"{self.target_class} detected — sending goal "
            f"(target_distance={self.stop_distance})")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg.feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self._goal_active = False
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_future()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Action finished: {result}")
        self._goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = YoloActionTrigger()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
