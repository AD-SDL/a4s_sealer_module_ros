#! /usr/bin/env python3
"""Camera node"""

import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type


class cameraNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self):

        """
        Class constructor to set up the node
        """

        # Initiate the Node class's constructor and give it a name
        super().__init__("Camera_Node")

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.cameraPub = self.create_publisher(Image, "video_frames", 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.cameraTimer = self.create_timer(timer_period, self.cameraCallback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def cameraCallback(self):
        """Callback function.
        This function gets called every 0.1 seconds.
        """

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.

        ret, frame = self.cap.read()
        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.cameraPub.publish(self.br.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")


def main(args=None):  # noqa: D103

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = cameraNode()

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
