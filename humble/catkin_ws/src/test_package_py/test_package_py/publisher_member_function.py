# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from custom_msgs_pkg.msg import Mynum

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_ = self.create_publisher(Mynum, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.declare_parameter('my_parameter', 'world')

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        msg = Mynum()
        msg.num = self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info('Publishing: "%s"' % str(msg.num))
        self.i += 1

        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info('Parameter current value %s' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'pvalue='+str(self.i)
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
