# flask_backend.py

from flask import Flask, Response, request, jsonify
from flask_cors import CORS
import cv2
import time
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
import threading
import serial
import json

app = Flask(__name__)
CORS(app)

cap = None
ros_node = None
cmd_vel_pub = None
odom_reset_cli = None
follow_pub = None
confirm_pub = None
latest_occupancy_grid = None
latest_pointcloud = None
latest_robot_pose = None

joystick_active = False
last_joystick_time = 0
joystick_timeout = 3.0  # seconds
serial_port = '/dev/ttyACM0'
baudrate = 115200

def read_serial_loop():
    global joystick_active, last_joystick_time
    ser = None
    while True:
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(serial_port, baudrate, timeout=1)
                print(f"Opened serial port {serial_port}")

            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            if line == "JOYSTICK_CONNECTED":
                joystick_active = True
                last_joystick_time = time.time()
            elif line.startswith("{") and line.endswith("}"):
                try:
                    data = json.loads(line)
                    last_joystick_time = time.time()
                    joystick_active = True

                    twist = Twist()
                    twist.linear.x = float(data.get("bot_y", 0.0)) / 2  # Forward/Back
                    twist.angular.z = -float(data.get("bot_x", 0.0)) / 2  # Left/Right
                    cmd_vel_pub.publish(twist)

                    # TODO: add turret_x, turret_y, bot_sw, turret_sw usage here if needed

                except json.JSONDecodeError:
                    print("Invalid JSON from serial:", line)
        except Exception as e:
            print("Serial error:", e)
            time.sleep(1)

        if time.time() - last_joystick_time > joystick_timeout:
            joystick_active = False

def odom_callback(msg):
    global latest_robot_pose
    latest_robot_pose = msg

def generate_frames():
    global cap
    while True:
        success, frame = cap.read()
        if not success:
            break
        frame = cv2.resize(frame, (640, 480))
        _, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.02)

def occupancy_grid_callback(msg):
    global latest_occupancy_grid
    latest_occupancy_grid = msg

def cloud_callback(msg):
    global latest_pointcloud
    latest_pointcloud = msg

@app.route('/reset_odometry', methods=['POST'])
def reset_odometry():
    global ros_node, odom_reset_cli
    if odom_reset_cli is None:
        odom_reset_cli = ros_node.create_client(Empty, '/reset_odom')
    if not odom_reset_cli.wait_for_service(timeout_sec=5.0):
        return jsonify({'status': 'error', 'message': 'Service not available'})
    future = odom_reset_cli.call_async(Empty.Request())
    rclpy.spin_until_future_complete(ros_node, future)
    return jsonify({'status': 'ok', 'message': 'Odometry reset triggered'})

@app.route('/follow_target', methods=['POST'])
def follow_target():
    global follow_pub
    follow_pub.publish(String(data='start_follow'))
    return jsonify({'status': 'ok'})

@app.route('/confirm_target', methods=['POST'])
def confirm_target():
    global confirm_pub
    confirm_pub.publish(String(data='confirm'))
    return jsonify({'status': 'ok'})

@app.route('/get_map', methods=['POST'])
def get_map():
    global latest_pointcloud
    if latest_pointcloud is None:
        return jsonify({'status': 'error', 'message': 'No data'})
    cloud_points = list(pc2.read_points(latest_pointcloud, field_names=("x", "y", "z"), skip_nans=True))
    pcd_path = '/tmp/rtabmap_map.pcd'
    with open(pcd_path, 'w') as f:
        f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n")
        f.write(f"COUNT 1 1 1\nWIDTH {len(cloud_points)}\nHEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(cloud_points)}\nDATA ascii\n")
        for pt in cloud_points:
            f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
    return jsonify({'status': 'ok', 'message': 'Map saved'})

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/occupancy_grid', methods=['GET'])
def get_occupancy_grid():
    global latest_occupancy_grid
    if latest_occupancy_grid is None:
        return jsonify({'status': 'error', 'message': 'No occupancy grid data available'})

    width = latest_occupancy_grid.info.width
    height = latest_occupancy_grid.info.height
    data = list(latest_occupancy_grid.data)
    grid = [data[i * width:(i + 1) * width] for i in range(height)]

    # Flip vertically so 0,0 is bottom-left in display
    grid = grid[::-1]

    return jsonify({
        'status': 'ok',
        'width': width,
        'height': height,
        'resolution': latest_occupancy_grid.info.resolution,
        'origin': {
            'position': {
                'x': latest_occupancy_grid.info.origin.position.x,
                'y': latest_occupancy_grid.info.origin.position.y,
                'z': latest_occupancy_grid.info.origin.position.z
            },
            'orientation': {
                'x': latest_occupancy_grid.info.origin.orientation.x,
                'y': latest_occupancy_grid.info.origin.orientation.y,
                'z': latest_occupancy_grid.info.origin.orientation.z,
                'w': latest_occupancy_grid.info.origin.orientation.w
            }
        },
        'data': grid
    })

@app.route('/robot_pose', methods=['GET'])
def robot_pose():
    global latest_robot_pose
    if latest_robot_pose is None:
        return jsonify({'status': 'error', 'message': 'No odometry data yet'})

    pos = latest_robot_pose.pose.pose.position
    twist = latest_robot_pose.twist.twist

    return jsonify({
        'status': 'ok',
        'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
        'velocity': {
            'linear': {
                'x': twist.linear.x,
                'y': twist.linear.y,
                'z': twist.linear.z
            },
            'angular': {
                'x': twist.angular.x,
                'y': twist.angular.y,
                'z': twist.angular.z
            }
        }
    })


@app.route('/cmd_vel', methods=['POST'])
def cmd_vel():
    global cmd_vel_pub
    if joystick_active:
        return jsonify({'status': 'ignored', 'message': 'Joystick in control'})
    data = request.get_json(force=True)
    twist = Twist()
    twist.linear.x = float(data.get('linear_x', 0.0)) / 2
    twist.angular.z = float(data.get('angular_z', 0.0)) / 2
    cmd_vel_pub.publish(twist)
    return jsonify({'status': 'ok'})

@app.route('/turret_cmd', methods=['POST'])
def turret_cmd():
    if joystick_active:
        return jsonify({'status': 'ignored', 'message': 'Joystick in control'})
    data = request.json
    print(f"Turret cmd: x={data.get('aim_x')}, y={data.get('aim_y')}")
    return jsonify({"status": "ok"})

@app.route('/fire', methods=['POST'])
def fire():
    if joystick_active:
        return jsonify({'status': 'ignored', 'message': 'Joystick in control'})
    print("Turret fired!")
    return jsonify({"status": "ok"})

@app.route('/')
def index():
    return "<h2>Camera Feed Server Running</h2>"

def ros_spin_thread():
    rclpy.spin(ros_node)

if __name__ == '__main__':
    if os.environ.get('WERKZEUG_RUN_MAIN') == 'true' or not app.debug:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Could not open /dev/video0")

        rclpy.init()
        ros_node = Node('web_cmd_vel_publisher')
        cmd_vel_pub = ros_node.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        ros_node.create_subscription(PointCloud2, '/rtabmap/point_cloud/global_map', cloud_callback, 10)
        ros_node.create_subscription(OccupancyGrid, '/global_cloud_grid', occupancy_grid_callback, 10)
        ros_node.create_subscription(Odometry, '/rtabmap/odom', odom_callback, 10)

        threading.Thread(target=ros_spin_thread, daemon=True).start()
        # threading.Thread(target=read_serial_loop, daemon=True).start()

    app.run(host='0.0.0.0', port=5000, debug=False)
