import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np
from rrt_planner_new import InformedRRTStarFn2
from visualizer import RRTVisualizer
from geometry_msgs.msg import Twist
import threading

class ExploratorNode(Node):
    def __init__(self):
        super().__init__('explorator_node')

        self.map = None
        self.map_info = None
        self.goal = None
        self.planner = None
        self.vis = None
        self.robot_pos = np.array([0.0, 0.0])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_follow_index = 0  # Track next waypoint


        self.map_sub = self.create_subscription(OccupancyGrid, '/global_cloud_grid', self.map_cb, 10)
        self.goal_sub = self.create_subscription(Point, '/exploration_goal', self.goal_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.tick)

    def map_cb(self, msg):
        self.map_info = msg.info
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.get_logger().info("Occupancy grid received.")
        if self.vis is None:
            self.vis = RRTVisualizer(self.map, self.map_info.resolution,
                                     (self.map_info.origin.position.x, self.map_info.origin.position.y))

    def goal_cb(self, msg):
        self.goal = np.array([msg.x, msg.y])
        self.get_logger().info(f"New goal received: {self.goal}")
        self.reset_planner()

    def reset_planner(self):
        if self.map is None or self.goal is None:
            return

        start = self.robot_pos.copy()
        resolution = self.map_info.resolution
        origin = (self.map_info.origin.position.x, self.map_info.origin.position.y)
        grid_size = np.array([self.map_info.width * resolution, self.map_info.height * resolution])

        self.planner = InformedRRTStarFn2(start, self.goal, grid_size,
                                  occupancy_grid=self.map,
                                  origin=np.array(origin),
                                  resolution=resolution)

    def update_robot_pos(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.robot_pos = np.array([tf.transform.translation.x, tf.transform.translation.y])
        except:
            self.get_logger().warn("TF lookup failed.")
            return
        
    def follow_path(self):
        if not self.planner or not self.planner.path or len(self.planner.path) < 2:
            return

        target = self.planner.get_next_waypoint()
        dist = np.linalg.norm(self.robot_pos - target)

        if dist < 0.1:
            self.path_follow_index += 1
            if self.path_follow_index >= len(self.planner.path):
                self.get_logger().info("Goal reached.")
                self.stop_bot()
                return
            target = self.planner.path[self.path_follow_index]

        direction = target - self.robot_pos
        angle = np.arctan2(direction[1], direction[0])

        cmd = Twist()
        cmd.linear.x = min(0.2, dist)
        cmd.angular.z = angle  # naive: make angular control smarter later
        self.cmd_pub.publish(cmd)

        # Prune behind
        self.planner.prune_nodes_near(self.robot_pos)

    def stop_bot(self):
        self.cmd_pub.publish(Twist())

    def tick(self):
        if self.map is None or self.goal is None:
            return

        self.update_robot_pos()

        if self.planner is None:
            self.reset_planner()

        # Rewire tree around current robot pose
        self.planner.update_start_or_goal(self.robot_pos, is_start=True)

        # Add a few nodes per tick
        self.planner.update_current_position(self.robot_pos)
        self.planner.plan_multiple_steps(20)

        if self.vis:
            self.vis.draw(self.robot_pos, self.goal, self.planner.nodes, self.planner.path)
        self.follow_path()

def main(args=None):
    rclpy.init(args=args)
    node = ExploratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
