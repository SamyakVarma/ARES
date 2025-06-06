import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import CameraInfo #TODO: ZED2 FOV
from collections import deque
import cv2
import numpy as np
import tf2_ros
import struct
import math


class GlobalPointCloudMapper(Node):
    def __init__(self):
        super().__init__('global_point_cloud_mapper')

        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid_size', 40.0)
        self.declare_parameter('free_radius', 1.5)
        self.declare_parameter('cloud_topic', '/rtabmap/cloud_map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_obstacle_height', 0.2)
        self.declare_parameter('max_obstacle_height', 1.5)
        self.declare_parameter('sensor_fov_deg', 30.0)  # cone angle 110 for ZED2

        self.res = self.get_parameter('grid_resolution').value
        self.size = self.get_parameter('grid_size').value
        self.free_radius = self.get_parameter('free_radius').value
        self.cloud_topic = self.get_parameter('cloud_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.min_z = self.get_parameter('min_obstacle_height').value
        self.max_z = self.get_parameter('max_obstacle_height').value
        self.fov_rad = math.radians(self.get_parameter('sensor_fov_deg').value)

        self.grid_width = int(self.size / self.res)
        self.grid_height = int(self.size / self.res)

        self.max_depth_init = min(self.grid_height/3, 4)
        self.prev_pose = None

        self.grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #self.create_subscription(CameraInfo, '/zed2/left/camera_info', self.camera_info_callback, 1) TODO: ZED2 FOV
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(Info, '/rtabmap/info', self.loop_closure_callback, 10)

        self.publisher = self.create_publisher(OccupancyGrid, 'global_cloud_grid', 1)
        self.timer = self.create_timer(1.0, self.publish_grid)
        
        self.cleanup_interval = 10.0  # seconds
        self.min_region_size = 30     # minimum size of valid free region
        self.cleanup_timer = self.create_timer(self.cleanup_interval, self.remove_isolated_free_regions_bfs)

        self.latest_cloud = None
        self.get_logger().info("Global point cloud to occupancy grid mapper started.")
        self.publish_grid()

    def camera_info_callback(self, msg: CameraInfo):
        """ZED2 camera FOV calculation"""
        fx = msg.k[0]  # focal length in pixels (horizontal)
        width = msg.width
        if fx > 0:
            self.fov_rad = 2 * math.atan2(width / 2.0, fx)
            fov_deg = math.degrees(self.fov_rad)
            self.get_logger().info(f"Updated camera FOV from camera_info: {fov_deg:.2f} degrees")
        else:
            self.get_logger().warn("Invalid fx from camera_info, using fallback FOV.")
            self.fov_rad = math.radians(110.0)

    def remove_isolated_free_regions_bfs(self):
        visited = np.zeros_like(self.grid, dtype=bool)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        removed_cells = 0
        region_id = 1

        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if self.grid[y, x] == 0 and not visited[y, x]:
                    queue = deque()
                    queue.append((x, y))
                    visited[y, x] = True
                    region = [(x, y)]

                    while queue:
                        cx, cy = queue.popleft()
                        for dx, dy in directions:
                            nx, ny = cx + dx, cy + dy
                            if (0 <= nx < self.grid_width and 0 <= ny < self.grid_height and
                                self.grid[ny, nx] == 0 and not visited[ny, nx]):
                                visited[ny, nx] = True
                                queue.append((nx, ny))
                                region.append((nx, ny))

                    if len(region) < self.min_region_size:
                        for gx, gy in region:
                            self.grid[gy, gx] = -1  # Reset to unknown
                        removed_cells += len(region)

        if removed_cells > 0:
            self.get_logger().info(f"BFS cleanup: Removed {removed_cells} disconnected free-space cells.")

    # def remove_isolated_free_regions_bfs(self):
    #     # Step 1: Create binary mask of obstacles
    #     obstacle_mask = np.uint8((self.grid == 100) * 255)

    #     # Step 2: Dilate the obstacle mask (to close small gaps)
    #     kernel = np.ones((3, 3), np.uint8)  # Or (5, 5) for stronger closure
    #     dilated_obstacles = cv2.dilate(obstacle_mask, kernel, iterations=1)

    #     # Step 3: Invert to get candidate free mask (where BFS is allowed to explore)
    #     pseudo_free_mask = (dilated_obstacles == 0)

    #     # Step 4: Run BFS on original grid, but use pseudo_free_mask for checking connectivity
    #     visited = np.zeros_like(self.grid, dtype=bool)
    #     directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    #     removed_cells = 0

    #     for y in range(self.grid_height):
    #         for x in range(self.grid_width):
    #             if self.grid[y, x] == 0 and not visited[y, x] and pseudo_free_mask[y, x]:
    #                 queue = deque()
    #                 queue.append((x, y))
    #                 visited[y, x] = True
    #                 region = [(x, y)]

    #                 while queue:
    #                     cx, cy = queue.popleft()
    #                     for dx, dy in directions:
    #                         nx, ny = cx + dx, cy + dy
    #                         if (0 <= nx < self.grid_width and 0 <= ny < self.grid_height and
    #                             self.grid[ny, nx] == 0 and not visited[ny, nx] and pseudo_free_mask[ny, nx]):
    #                             visited[ny, nx] = True
    #                             queue.append((nx, ny))
    #                             region.append((nx, ny))

    #                 # Remove region if too small
    #                 if len(region) < self.min_region_size:
    #                     for gx, gy in region:
    #                         self.grid[gy, gx] = -1  # Reset to unknown
    #                     removed_cells += len(region)

    #     if removed_cells > 0:
    #         self.get_logger().info(f"BFS cleanup: Removed {removed_cells} disconnected free-space cells.")

    def cloud_callback(self, msg):
        # Save latest cloud for loop-closure-triggered cleaning
        self.latest_cloud = msg

        # Update grid with new obstacles
        for pt in self.read_points(msg):
            x, y, z = pt
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue
            if z > self.max_z or z < self.min_z:
                continue

            gx = int((x + self.size / 2) / self.res)
            gy = int((y + self.size / 2) / self.res)

            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                # if self.grid[gy, gx] != 0:  # Don’t overwrite free
                self.grid[gy, gx] = 100
        if self.prev_pose is not None:
            x_prev, y_prev, yaw_prev = self.prev_pose
            self.mark_free_area(x_prev, y_prev, yaw_prev)

        # Mark free space around the robot
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            q = tf.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.prev_pose = (tf.transform.translation.x, tf.transform.translation.y, yaw)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return


    def loop_closure_callback(self, msg):
        if msg.loop_closure_id > 0 and self.latest_cloud:
            self.get_logger().info(f"Loop closure detected with ID {msg.loop_closure_id}, cleaning grid...")
            self.clean_stale_obstacles(self.latest_cloud)

    def clean_stale_obstacles(self, cloud_msg):
        # Step 1: Build set of valid obstacle cells from point cloud
        current_obstacles = set()

        for pt in self.read_points(cloud_msg):
            x, y, z = pt
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue
            if z > self.max_z or z < self.min_z:
                continue

            gx = int((x + self.size / 2) / self.res)
            gy = int((y + self.size / 2) / self.res)

            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                current_obstacles.add((gx, gy))

        # Step 2: Check and unmark stale obstacles
        count_cleaned = 0
        for gy in range(self.grid_height):
            for gx in range(self.grid_width):
                if self.grid[gy, gx] == 100 and (gx, gy) not in current_obstacles:
                    self.grid[gy, gx] = 0
                    count_cleaned += 1

        self.get_logger().info(f"Cleaned {count_cleaned} stale obstacle cells after loop closure.")
        
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def mark_free_area(self, x_center, y_center, yaw=0.0):
        half_fov = self.fov_rad / 2.0
        max_range = self.max_depth_init

        cx = int((x_center + self.size / 2) / self.res)
        cy = int((y_center + self.size / 2) / self.res)

        range_cells = int(max_range / self.res)
        min_gx = max(cx - range_cells, 0)
        max_gx = min(cx + range_cells, self.grid_width)
        min_gy = max(cy - range_cells, 0)
        max_gy = min(cy + range_cells, self.grid_height)

        for gx in range(min_gx, max_gx):
            for gy in range(min_gy, max_gy):
                dx = (gx - cx) * self.res
                dy = (gy - cy) * self.res
                distance = math.hypot(dx, dy)
                if distance > max_range:
                    continue

                angle = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(angle - yaw)

                if abs(angle_diff) > half_fov:
                    continue

                # Raycast to see if visible
                if self.is_visible(cx, cy, gx, gy):
                    if self.grid[gy, gx] == -1:
                        self.grid[gy, gx] = 0  # Mark as free

    def is_visible(self, x0, y0, x1, y1):
        """Bresenham's algorithm to check if line to (x1, y1) is blocked by an obstacle"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy

        while (x, y) != (x1, y1):
            if (x, y) != (x0, y0):  # skip starting cell
                if 0 <= y < self.grid_height and 0 <= x < self.grid_width:
                    if self.grid[y, x] == 100:
                        return False  # Obstacle blocks view
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True


    def publish_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.map_frame
        grid_msg.info.resolution = self.res
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = -self.size / 2
        grid_msg.info.origin.position.y = -self.size / 2
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = self.grid.flatten().tolist()
        self.publisher.publish(grid_msg)

    def read_points(self, cloud):
        fmt = self.get_struct_format(cloud)
        for i in range(cloud.width * cloud.height):
            offset = i * cloud.point_step
            pt = struct.unpack_from(fmt, cloud.data, offset)
            yield pt[:3]

    def get_struct_format(self, cloud):
        fmt = '<'
        for field in cloud.fields:
            if field.datatype == 7:  # FLOAT32
                fmt += 'f'
            else:
                raise NotImplementedError(f"Unsupported datatype {field.datatype}")
        return fmt


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPointCloudMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()