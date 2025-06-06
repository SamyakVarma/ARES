import rclpy
import tf2_ros
import numpy as np
import cv2
import math
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from scipy.ndimage import uniform_filter


class GradientExplorer(Node):
    def __init__(self):
        super().__init__('gradient_explorer')

        self.sub_map = self.create_subscription(
            OccupancyGrid, '/global_cloud_grid', self.map_callback, 10
        )
        self.pub_marker = self.create_publisher(Marker, '/frontier_markers', 10)
        self.pub_goal = self.create_publisher(Point, '/exploration_goal', 10)
        self.pub_goal_marker = self.create_publisher(Marker, '/exploration_goal_marker', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.robot_frame = 'base_link'
        self.map_frame = 'map'
        self.window_size = 6.0  # meters
        self.grid_resolution = 0.1
        self.cluster_eps = 0.105  # meters

        self.prev_frontiers = []
        self.latest_map = None

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, now, timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        _, _, yaw = self.quaternion_to_euler(tf.transform.rotation)

        # Define local window
        box_size = int(self.window_size / msg.info.resolution)
        cx = int((tx + math.cos(yaw) * self.window_size / 2 - msg.info.origin.position.x) / msg.info.resolution)
        cy = int((ty + math.sin(yaw) * self.window_size / 2 - msg.info.origin.position.y) / msg.info.resolution)

        x0 = max(0, cx - box_size // 2)
        y0 = max(0, cy - box_size // 2)
        x1 = min(msg.info.width, cx + box_size // 2)
        y1 = min(msg.info.height, cy + box_size // 2)

        # Extract and blur local subgrid
        raw_grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        subgrid = raw_grid[y0:y1, x0:x1]
        blurred = self.blur_occupancy(subgrid)

        frontiers = self.detect_frontiers(blurred)

        # Convert to world coordinates
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        res = msg.info.resolution
        local_frontiers = [
            (
                origin_x + (x + x0) * res,
                origin_y + (y + y0) * res
            )
            for y, x in frontiers
        ]

        # Filter out already mapped frontiers
        if self.latest_map:
            self.prev_frontiers = self.filter_mapped_frontiers(self.prev_frontiers, raw_grid, self.latest_map)

        # Avoid near-duplicates
        new_frontiers = self.filter_spaced_frontiers(local_frontiers, self.prev_frontiers, min_distance=0.8)
        self.prev_frontiers.extend(new_frontiers)

         # Best frontier selection
        best = self.choose_best_frontier(
            self.prev_frontiers, (tx, ty), yaw, min_dist=2.0, alpha=1.0, beta=1.5
        )
        if best:
            self.get_logger().info(f"Best frontier selected: {best}")
            self.publish_goal(best)

        self.publish_markers(self.prev_frontiers)

        vis = self.visualize_full_map(self.latest_map, self.prev_frontiers, best)
        cv2.imshow("Full Frontier Map", cv2.resize(vis, None, fx=1.5, fy=1.5))
        cv2.waitKey(1)

    def blur_occupancy(self, grid):
        img = np.zeros_like(grid, dtype=np.uint8)
        img[grid == 0] = 200      # free
        img[grid == -1] = 100     # unknown
        img[grid == 100] = 0      # obstacle

        obstacle_mask = (grid == 100).astype(np.uint8)
        blurred = cv2.GaussianBlur(img, (5, 5), sigmaX=1.0)

        blurred[obstacle_mask == 1] = 0

        processed = np.full_like(grid, -1, dtype=np.int8)
        processed[blurred > 180] = 0
        processed[blurred < 50] = 100
        processed[(blurred >= 50) & (blurred <= 180)] = -1

        processed[obstacle_mask == 1] = 100
        return processed

    def is_frontier(self, x, y, grid):
        neighborhood = grid[y - 1:y + 2, x - 1:x + 2]
        if -1 in neighborhood and 100 not in neighborhood:
            return True
        return False

    def detect_frontiers(self, grid):
        frontiers = []
        for y in range(1, grid.shape[0] - 1):
            for x in range(1, grid.shape[1] - 1):
                if grid[y, x] != 0:
                    continue
                if self.is_frontier(x, y, grid):
                    frontiers.append((y, x))
        return frontiers

    def filter_spaced_frontiers(self, new_points, existing_points, min_distance=0.5):
        filtered = []
        for pt in new_points:
            too_close = any(
                np.linalg.norm(np.array(pt) - np.array(existing)) < min_distance
                for existing in existing_points + filtered
            )
            if not too_close:
                filtered.append(pt)
        return filtered

    def filter_mapped_frontiers(self, frontiers, grid, msg):
        filtered = []
        for wx, wy in frontiers:
            gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
            gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)

            if 0 <= gx < msg.info.width and 0 <= gy < msg.info.height:
                if self.is_frontier(gx, gy, grid):
                    filtered.append((wx, wy))
        return filtered

    def choose_best_frontier(self, frontiers, robot_pos, robot_yaw, min_dist=0.8,
                             alpha=2.0, beta=1.0, gamma=1.5, delta=3.0):
        if not self.latest_map:
            return None

        ox = self.latest_map.info.origin.position.x
        oy = self.latest_map.info.origin.position.y
        res = self.latest_map.info.resolution
        grid = np.array(self.latest_map.data, dtype=np.int8).reshape(
            (self.latest_map.info.height, self.latest_map.info.width)
        )

        best_score = -float('inf')
        best_point = None

        for wx, wy in frontiers:
            dx = wx - robot_pos[0]
            dy = wy - robot_pos[1]
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                continue

            angle = math.atan2(dy, dx)
            angle_diff = abs(math.atan2(math.sin(angle - robot_yaw), math.cos(angle - robot_yaw)))
            angle_penalty = angle_diff

            # Grid coordinates
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)
            if not (0 <= gx < grid.shape[1] and 0 <= gy < grid.shape[0]):
                continue

            # Local window for unknowns and obstacles
            window = grid[max(0, gy - 5):min(grid.shape[0], gy + 5),
                          max(0, gx - 5):min(grid.shape[1], gx + 5)]

            unknown_score = np.sum(window == -1)
            obstacle_penalty = np.sum(window == 100)

            score = alpha * unknown_score - beta * angle_penalty - gamma * dist - delta * obstacle_penalty

            if score > best_score:
                best_score = score
                best_point = (wx, wy)

        return best_point



    def angle_diff(self, a, b):
        """Compute smallest signed angle difference between two angles."""
        return math.atan2(math.sin(a - b), math.cos(a - b))
    
    def visualize_full_map(self, msg: OccupancyGrid, frontier_pts, best_frontier=None):
        raw = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        vis = np.zeros((*raw.shape, 3), dtype=np.uint8)
        vis[raw == -1] = (127, 127, 127)  # unknown
        vis[raw == 0] = (255, 255, 255)   # free
        vis[raw == 100] = (0, 0, 0)       # occupied

        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        for wx, wy in frontier_pts:
            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)
            if 0 <= gx < msg.info.width and 0 <= gy < msg.info.height:
                cv2.circle(vis, (gx, gy), 1, (0, 0, 255), -1)  # red frontiers

        if best_frontier:
            bx, by = best_frontier
            gx = int((bx - ox) / res)
            gy = int((by - oy) / res)
            if 0 <= gx < msg.info.width and 0 <= gy < msg.info.height:
                cv2.circle(vis, (gx, gy), 3, (0, 255, 0), -1)  # green best

        return vis



    def publish_markers(self, points):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        marker.points = [Point(x=pt[0], y=pt[1], z=0.1) for pt in points]
        self.pub_marker.publish(marker)

    def publish_goal(self, point):
        goal_msg = Point()
        goal_msg.x = point[0]
        goal_msg.y = point[1]
        goal_msg.z = 0.0
        self.pub_goal.publish(goal_msg)

        # Also publish marker
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = goal_msg
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        self.pub_goal_marker.publish(marker)

    def visualize_frontiers(self, grid, frontier_indices):
        vis = np.zeros((*grid.shape, 3), dtype=np.uint8)
        vis[grid == -1] = (127, 127, 127)
        vis[grid == 0] = (255, 255, 255)
        vis[grid == 100] = (0, 0, 0)
        for y, x in frontier_indices:
            cv2.circle(vis, (x, y), 1, (0, 0, 255), -1)
        return vis

    def quaternion_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw


def main(args=None):
    rclpy.init(args=args)
    node = GradientExplorer()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()