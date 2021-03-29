#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Image Subscriber Example.

Revision History:
        2021-03-29 (Animesh): Baseline Software.

Example:
        $ ros2 run ros2_camera_publish execute

"""


#___Import Modules:
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/racecar/camera', 
                                                self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        self.get_logger().info("Image Received")
        
        cv2.imwrite("ani717.png", frame)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""
