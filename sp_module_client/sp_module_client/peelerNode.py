#! /usr/bin/env python3
"""Peeler node"""

from typing import List, Tuple

import rclpy  # import Rospy
from azenta_driver.peeler_driver import BROOKS_PEELER_CLIENT  # import peeler driver
from rclpy.node import Node  # import Rospy Node
from wei_services.srv import WeiActions, WeiDescription
from std_msgs.msg import String


class peelerNode(Node):
    """
    The peelerNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback,
    based on the executed command and publishes the state of the peeler and a description of the peeler to the respective topics.
    """

    def __init__(self, NODE_NAME="peelerNode"):
        """
        The init function is neccesary for the peelerNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        """

        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()


        self.declare_parameter('peeler_port', '/dev/ttyUSB0')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        PORT = self.get_parameter('peeler_port')    # Renaming parameter to general form so it can be used for other nodes too

        self.peeler = BROOKS_PEELER_CLIENT(PORT.value)
        print("Peeler is online")                   # Wakeup Message

        # self.state = 'ready'
        self.state = self.peeler.get_status() 

        self.description = {
            'name': node_name,
            'type':'',
            'actions':
            {
                'peel':'%d %d'
            }
            }

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, "/peeler_state", 10)       # Publisher for peeler state
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)   # Callback that publishes to peeler state

        self.actionSrv = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback)

        self.descriptionSrv = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback)

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response
        
        
    def actionCallback(self, request, response):
        """The actions the robot can perform, also performs them

        Parameters:
        -----------
        request: str
            Request to the robot to perform an action
        respone: bool
            If action is performed

        Returns
        -------
        None
        """

        action_handle = request.action_handle  # Run commands if manager sends corresponding command
        self.state = "BUSY"
        self.stateCallback()

        if action_handle=="status":
            self.peeler.reset()
            self.peeler.check_version()
            self.peeler.get_status()

            response.action_response = 0
            response.action_msg= "all good peeler"
            self.get_logger().info('Finished Action: ' + request.action_handle)
            return response

        elif action_handle=="peel":
            vars = eval(request.vars)
            print(vars)

            self.peeler.seal_check()
            self.peeler.peel(1, 2.5)

            response.action_response = 0
            response.action_msg= "all good peeler"
            self.get_logger().info('Finished Action: ' + request.action_handle)
            return response

        if "Error:" in self.peeler.peeler_output:
            self.state = self.peeler.error_msg

    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""

        msg = String()

        msg.data = "State: %s" % self.state

        self.statePub.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)


        # self.state = self.peeler.get_status()


def main(args=None):  # noqa: D103

    TEMP_NODE_NAME = "peelerNode"   # Node name for peeler   

    rclpy.init(args=args)       # initialize Ros2 communication

    node = peelerNode(TEMP_NODE_NAME=TEMP_NODE_NAME)

    rclpy.spin(node)            # keep Ros2 communication open for action node

    rclpy.shutdown()            # kill Ros2 communication


if __name__ == "__main__":

    main()
