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
import numpy as np

from sensor_msgs.msg import Image
import random

import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(
            Image,
            'topic',
            10)
        timer_period = 3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Image()
        msg.header.frame_id = 'map'
        msg.height = 1000
        msg.width = 1000
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = 3000
        msg.data = [random.randint(0, 255) for _ in range(3000000)]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        print('Publishing an "%s" image' % msg.encoding)
        print('Publishing an image of size "%d" bytes' % len(msg.data))
        print('Publishing an image of size "%d" pixels' % (len(msg.data)/3))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
