import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Bool
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class YamlToArrayPublisher(Node):
    def __init__(self, yaml_file):
        super().__init__('yaml_to_array_publisher')

        self.joint_publisher = self.create_publisher(Float64MultiArray, 'joint_values', 10)
        self.ticks_publisher = self.create_publisher(Int32MultiArray, 'ticks_values', 10)
        self.index_publisher = self.create_publisher(Int32MultiArray, 'index_values', 10)
        self.completion_publisher = self.create_publisher(Int32MultiArray, 'completion_signal', 10)

        self.subscription = self.create_subscription(
            Bool,
            '/confirmation_signal',
            self.confirmation_callback,
            10)

        self.yaml_path = yaml_file

        try:
            with open(self.yaml_path, 'r') as file:
                self.data = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f'YAML file not found at {self.yaml_path}')
            raise

        self.formatted_data = self.format_data(self.data)
        self.current_index = 0
        self.ready_to_publish = True

        self.get_logger().info(f'Loaded YAML data from {self.yaml_path}')

    def format_data(self, data):
        formatted_data = []
        for main_key, sub_dict in data.items():
            for sub_key, sub_sub_dict in sub_dict.items():
                for inner_key, values in sub_sub_dict.items():
                    joint_values = [values[f'j{i+1}'] for i in range(6)]
                    ticks_value = [values['ticks']]
                    origin_index = [int(main_key), int(sub_key), int(inner_key)]
                    index = [int(main_key), -1 ,int(sub_key)]
                    formatted_data.append([index, joint_values, ticks_value])
        return formatted_data

    def confirmation_callback(self, msg):
        if msg.data:
            self.ready_to_publish = True
            self.publish_data()

    def publish_data(self):
        if self.current_index < len(self.formatted_data):
            if self.ready_to_publish:
                data_item = self.formatted_data[self.current_index]
                index, joint_values, ticks_value = data_item

                joint_msg = Float64MultiArray()
                joint_msg.data = joint_values

                ticks_msg = Int32MultiArray()
                ticks_msg.data = ticks_value

                index_msg = Int32MultiArray()
                index_msg.data = index

                self.joint_publisher.publish(joint_msg)
                self.ticks_publisher.publish(ticks_msg)
                self.index_publisher.publish(index_msg)

                self.get_logger().info(f'Published joint values: {joint_values}')
                self.get_logger().info(f'Published ticks values: {ticks_value}')
                self.get_logger().info(f'Published index values: {index}')

                self.current_index += 1
                self.ready_to_publish = False
        else:
            completion_msg = Int32MultiArray()
            completion_msg.data = [0]
            self.completion_publisher.publish(completion_msg)
            self.get_logger().info('Published completion signal')

def main(args=None):
    rclpy.init(args=args)

    yaml_file = os.path.join(
        get_package_share_directory('growth_meter_application'),
        'config', 'target_new.yaml'
    )

    node = YamlToArrayPublisher(yaml_file)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
