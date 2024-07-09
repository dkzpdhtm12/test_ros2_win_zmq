import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import MoveJoint
import json
import zmq
import subprocess

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

        self.timer = self.create_timer(0.1, self.win_pub_listener_callback)

        self.get_logger().info('Ready for windows communication in ROS2 python.')

    def win_pub_listener_callback(self):
        try:
            message = self.zmq_socket_sub.recv(flags=zmq.NOBLOCK).decode('utf-8')
            print(message)
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

                elif topic_name == '/request_move_forward':
                    msg = Twist()
                    msg.linear = Vector3(x=message_data, y=.0, z=0.0)
                    msg.angular = Vector3(x=.0, y=.0, z=0.0)
                    self.create_publisher(Twist, '/cmd_vel', 10).publish(msg)

                else:
                    pass

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

    def joint_states_ros2_topic_subscriber_callback(self, msg):
        positions = list(msg.position)
        json_data = json.dumps({
            'topic': '/joint_states',
            'message': {
                'positions': positions
            }
        })
        self.zmq_socket_pub.send_string(json_data)

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

