#!/bin/python3

# stdlib 

# ROS2 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Program-specific
import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge

class PiCam2IMGSubscribe(Node):

    def __init__(self):
        super().__init__('PiCam2_img')
        self.subscription = self.create_subscription(
            Image, # the message type
            'PiCam2IMG', # the topic name
            self.listener_callback, # function called when getting a message
            10)
        self.CvBridge = CvBridge()

    def listener_callback(self, msg:Image):
        self.get_logger().info(f'I got the message! Attempting to decode...')
        array = self.CvBridge.imgmsg_to_cv2(msg)
        self.get_logger().info(f'Displaying the message...')
        plt.imshow(array)
        plt.axis('off')
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    sub = PiCam2IMGSubscribe()

    rclpy.spin(sub)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting")
    

