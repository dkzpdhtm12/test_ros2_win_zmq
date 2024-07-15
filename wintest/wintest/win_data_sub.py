
import zmq
import json
import yaml
import rclpy
import subprocess
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int8, Int32, Bool, String

PI = 3.141592

class WindowsCommunication(Node):

    def __init__(self):
        super().__init__('windows_communication')
        self.zmq_context = zmq.Context()
        self.zmq_socket_sub = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket_sub.connect("tcp://192.168.0.248:12345")
        self.zmq_socket_sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self.zmq_socket_pub = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket_pub.bind("tcp://*:12346")

        self.cylinder_ticks_ros2_topic_subscriber = self.create_subscription(
            Int32,
            '/cylinder/ticks',
            self.cylinder_ticks_ros2_topic_subscriber_callback,
            10)

        self.joint_states_ros2_topic_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_ros2_topic_subscriber_callback,
            10)

        self.robot_current_work_ros2_topic_subscriber = self.create_subscription(
            String,
            '/robot_current_work',
            self.robot_current_work_ros2_topic_subscriber_callback,
            10)

        self.timer = self.create_timer(0.1, self.win_pub_listener_callback)

        self.target = {}
        self.joints_list = []
        self.ticks = 0

        self.get_logger().info('Ready for windows communication in ROS2 python.')

    def win_pub_listener_callback(self):
        try:
            message = self.zmq_socket_sub.recv(flags=zmq.NOBLOCK).decode('utf-8')
            if message:
                json_data = json.loads(message)

                topic_name = json_data['topic']
                message_data = json_data['message']['data']

                if topic_name == 'manipulator_connect':
                    self.get_logger().info('Connecting to Doosan manipulator')
                    subprocess.Popen(['ros2', 'launch', 'growth_meter_bringup', 'doosan.launch.py'])

                elif topic_name == '/joy_command':
                    msg = Int8()
                    msg.data = int(message_data)
                    self.create_publisher(Int8, topic_name, 10).publish(msg)
                    self.get_logger().info(f'Received and republished message: {msg.data}')

                elif topic_name == '/joint_request':
                    self.handle_joint_request(message_data)

                elif topic_name == '/request_move_vertical':
                    msg = Twist()
                    msg.linear = Vector3(x=message_data, y=0.0, z=0.0)
                    msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
                    self.create_publisher(Twist, '/cmd_vel', 10).publish(msg)

                elif topic_name == '/request_move_horizontal':
                    msg = Twist()
                    msg.linear = Vector3(x=0.0, y=message_data, z=0.0)
                    msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
                    self.create_publisher(Twist, '/cmd_vel', 10).publish(msg)

                elif topic_name == '/request_move_spin':
                    msg = Twist()
                    msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
                    msg.angular = Vector3(x=0.0, y=0.0, z=message_data)
                    self.create_publisher(Twist, '/cmd_vel', 10).publish(msg)

                elif topic_name == '/save_yaml':
                    area = message_data[0]
                    growth = message_data[1]
                    self.handle_save_yaml(area, growth)

                elif topic_name == '/load_yaml':
                    self.handle_load_yaml()

                elif topic_name == '/confirmation_signal':
                    msg = Bool()
                    msg.data = True
                    self.create_publisher(Bool, '/confirmation_signal', 10).publish(msg)

                elif topic_name == '/auto_drive_signal':
                    msg = Bool()
                    msg.data = True
                    self.create_publisher(Bool, '/pushed1', 10).publish(msg)

                else:
                    self.get_logger().error(f'Undefined topic message received. {message} Check topic name or Add function by topic name')

        except zmq.Again:
            pass
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Error processing message: {e}')

    def handle_joint_request(self, data):
        try:
            pos = [float(value) for value in data]

            future = self.call_service(pos)
            future.add_done_callback(self.joint_request_callback)

        except Exception as e:
            self.get_logger().error(f'Error handling joint_request: {e}')

    def call_service(self, pos):
        client = self.create_client(MoveJoint, 'motion/move_joint')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = MoveJoint.Request()
        request.pos = pos
        request.vel = 5.0
        request.acc = 100.0
        request.time = 0.0
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 1

        return client.call_async(request)

    def joint_request_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Service call successful!')
            else:
                self.get_logger().warn('Service call failed!')
        except Exception as e:
            self.get_logger().error(f'Service call failed with exception: {e}')

    def cylinder_ticks_ros2_topic_subscriber_callback(self, msg):
        json_data = json.dumps({
            'topic': '/cylinder/ticks',
            'message': {
                'data': msg.data
            }
        })
        self.zmq_socket_pub.send_string(json_data)
        self.ticks = msg.data

    def joint_states_ros2_topic_subscriber_callback(self, msg):
        positions = list(msg.position)
        json_data = json.dumps({
            'topic': '/joint_states',
            'message': {
                'positions': positions
            }
        })
        self.zmq_socket_pub.send_string(json_data)
        self.joints_list = msg.position

    def robot_current_work_ros2_topic_subscriber_callback(self, msg):
        json_data = json.dumps({
            'topic': '/robot_current_work',
            'message': {
                'data': msg.data
            }
        })
        self.zmq_socket_pub.send_string(json_data)
        print(json_data)
        self.robot_current_work = msg.data

    def handle_save_yaml(self, area, growth):
        joints_list = self.joints_list
        ticks = self.ticks

        success = self.add2yaml(area, growth, joints_list, ticks)
        if success:
            self.get_logger().info('YAML file updated successfully.')
        else:
            self.get_logger().error('Failed to update YAML file.')

    def add2yaml(self, target_area, shot_area, joints_list, ticks):
        try:
            joints_list = [float(j) for j in joints_list]
        except ValueError as e:
            self.get_logger().error(f'Value error: {e}')
            return False

        try:
            j1, j2, j3, j4, j5, j6 = joints_list
        except ValueError as e:
            self.get_logger().error(f'Unpacking error: {e}')
            return False

        try:
            shot_data = {
                'j1': j1 * 180 / PI,
                'j2': j2 * 180 / PI,
                'j3': j3 * 180 / PI,
                'j4': j4 * 180 / PI,
                'j5': j5 * 180 / PI,
                'j6': j6 * 180 / PI,
                'ticks': ticks
            }

            if target_area not in self.target:
                self.target[target_area] = {}

            if shot_area not in self.target[target_area]:
                self.target[target_area][shot_area] = {}

            shot_id = len(self.target[target_area][shot_area]) + 1
            self.target[target_area][shot_area][shot_id] = shot_data

            with open('config.yaml', 'w') as yaml_file:
                yaml.dump(self.target, yaml_file, default_flow_style=False, sort_keys=False)

            return True
        except AttributeError as e:
            self.get_logger().error(f'Attribute error: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return False

    def handle_load_yaml(self):
        try:
            with open('config.yaml', 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)
            json_data = json.dumps({
                'topic': '/send_yaml_data',
                'message': {
                    'data': yaml_data
                }
            })
            self.zmq_socket_pub.send_string(json_data)
            self.get_logger().info('YAML data sent successfully.')
        except FileNotFoundError:
            self.get_logger().error('YAML file not found.')
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error reading YAML file: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')

def main(args=None):
    rclpy.init(args=args)
    windows_communication = WindowsCommunication()
    try:
        rclpy.spin(windows_communication)
    except KeyboardInterrupt:
        pass
    finally:
        windows_communication.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()