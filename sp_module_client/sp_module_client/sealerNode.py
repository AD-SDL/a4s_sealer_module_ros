#! /usr/bin/env python3
"""Sealer Node"""

import string
from typing import List, Tuple

import rclpy  # import Rospy
from azenta_driver.sealer_driver import A4S_SEALER_CLIENT  # import sealer driver
from rclpy.node import Node  # import Rospy Node
from wei_services.srv import WeiActions, WeiDescription
from std_msgs.msg import String
from time import sleep

class sealerNode(Node):

    """
    The init function is neccesary for the sealerNode class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    """

    def __init__(self, TEMP_NODE_NAME = "sealerNode"):
        """Setup sealer node"""

        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()


        self.declare_parameter('sealer_port', '/dev/ttyUSB1')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        PORT = self.get_parameter('sealer_port')    # Renaming parameter to general form so it can be used for other nodes too
        self.sealer = A4S_SEALER_CLIENT(PORT.value)
        


        print("Sealer is online")               # Wakeup Message
        self.state = "UNKOWN"



        self.description = {
            'name': node_name,
            'type':'',
            'actions':
            {
                'prepare_sealer':'%d %d'
            }
            }

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, "/sealer_state", 10)       # Publisher for sealer state
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)   # Callback that publishes to sealer state

        self.actionSrv = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback)

        self.descriptionSrv = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback)


    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: Tuple[str, List]
            The actions a robot can do, will be populated during execution

        Returns
        -------
        Tuple[str, List]
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    def actionCallback(self, request: str, response: str) -> None:

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
        
        if request.action_handle=='seal':
            self.state = "BUSY"
            self.stateCallback()
            vars = eval(request.vars)
            print(vars)

            time = vars.get('time',3)
            temp = vars.get('temp',175)
            
            #self.sealer.set_time(3)
            #self.sealer.set_temp(175)
            self.sealer.seal()
            sleep(10)
            response.action_response = 0
            response.action_msg= "all good sealer"
            self.get_logger().info('Finished Action: ' + request.action_handle)
            return response

        self.state = "COMPLETED"

        return response


    def stateCallback(self):
        """The state of the robot, can be ready, completed, busy, error"""

        msg = String()

        msg.data = "State %s" % self.state

        self.statePub.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.state = "READY"


def main(args=None):  # noqa: D103

    TEMP_NODE_NAME = "sealerNode"   # Node name for peeler   

    rclpy.init(args=args)  # initialize Ros2 communication

    node = sealerNode(TEMP_NODE_NAME=TEMP_NODE_NAME)

    rclpy.spin(node)  # keep Ros2 communication open for action node

    rclpy.shutdown()  # kill Ros2 communication


if __name__ == "__main__":

    main()
