#!/bin/python3

# stdlib 
import sys

# ROS2 
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node

# Program-specific
import numpy as np
from picamera2 import Picamera2, Preview
from cv_bridge import CvBridge


class PiCam2IMGPublish(Node):

    def __init__(self):
        """
        Sets up Picam2, publishes images 
        """
        try:
            self.picam2 = Picamera2()
        except RuntimeError as e:
            print(f"Picamera failed to initialize. Check whether another \
                    application is not using it at the moment. \n {e}",
                  file=sys.stderr)
            exit(-1)
        except IndexError:
            print(f"Error: You've most likely inserted the ribbon during bootup. \
                    This will not work. \n {e}", file=sys.stderr)
            traceback_print_last(file=sys.stderr)
            exit(-1)
        self.capture_config = self.picam2.create_still_configuration(
                {
                    "format": "RGB888"
                 }
                )
        self.picam2.start(show_preview=False)
        super().__init__('PiCam2_img')
        # The image is of class numpy.ndarray
        # Optional : add metadata as a tuple in the future
        self.publisher_ = self.create_publisher(Image, 'PiCam2IMG', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        #CvBridge object
        self.CvBridge = CvBridge()

    def timer_callback(self):
        array = self.picam2.capture_array("main")
                # self.capture_config, "main")
                # for whatever reason, cv2 prefers xrgb
                # switch_mode_and_capture with the config does not work properly
                # and the output is bluer than usual
        msg = self.CvBridge.cv2_to_imgmsg(array)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing PiCam2 Image {self.i} : "msg"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        pub = PiCam2IMGPublish()
    except RuntimeError as e:
        print(f"Error: Failed to Initialize. You either hotswapped the camera, or the ribbon was loose.\
                \n Please check and reboot. \n {e}"
              , file=sys.stderr)
        sys.exit(-1)
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n Exiting...")
