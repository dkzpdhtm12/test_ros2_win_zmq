import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int32
import json
import zmq

class WindowsCommunication(Node):

    def __init__(self):
        super().__init__('windows_communication')
        self.zmq_context = zmq.Context()
        self.zmq_socket_sub = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket_sub.connect("tcp://192.168.0.248:12345")
        self.zmq_socket_sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self.zmq_socket_pub = self.zmq_context.socket(zmq.PUB)
        self.zmq_socket_pub.bind("tcp://*:12346")

        self.specific_ros2_topic_subscriber = self.create_subscription(
            Int32,
            '/cylinder/ticks',
            self.specific_ros2_topic_subscriber_callback,
            10)
       
        self.timer = self.create_timer(0.1, self.win_pub_listener_callback)

        self.get_logger().info('Ready to windows communication in ROS2 python.')

    def win_pub_listener_callback(self):
        try:
            message = self.zmq_socket_sub.recv(flags=zmq.NOBLOCK).decode('utf-8')
            if message:
                json_data = json.loads(message)

                topic_name = json_data['topic']
                message_data = json_data['message']['data']

                msg = Int8()
                msg.data = int(message_data)
                self.create_publisher(Int8, topic_name, 10).publish(msg)

                self.get_logger().info(f'Received and republished message: {msg.data}')
        except zmq.Again:
            pass
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Error processing message: {e}')

    def specific_ros2_topic_subscriber_callback(self, msg):
        json_data = json.dumps({
            'topic': '/cylinder/ticks',
            'message': {
                'data': msg.data
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