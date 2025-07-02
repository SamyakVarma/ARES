import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler

class ExplorerNavigator(Node):
    def __init__(self):
        super().__init__('exploration_navigator')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(PointStamped, '/exploration_goal', self.goal_callback, 10)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_callback(self, msg: PointStamped):
        self.get_logger().info(f"Received goal: ({msg.point.x:.2f}, {msg.point.y:.2f})")

        # Build goal message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = msg.point.x
        goal_pose.pose.position.y = msg.point.y
        goal_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, 0.0)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.send_navigation_goal(goal_pose)

    def send_navigation_goal(self, pose: PoseStamped):
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending goal to Nav2...")
        self._send_goal_future = self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        if future.result().status == 4:
            self.get_logger().warn("Goal was aborted!")
        else:
            self.get_logger().info("Goal succeeded!")

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNavigator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()