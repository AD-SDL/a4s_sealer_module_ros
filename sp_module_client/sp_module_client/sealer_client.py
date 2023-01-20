#! /usr/bin/env python3
"""Sealer Node"""

import string
from typing import List, Tuple

import rclpy  # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String

from wei_services.srv import WeiActions, WeiDescription

from time import sleep

from azenta_driver.sealer_driver import A4S_SEALER_DRIVER  # import sealer driver


class SealerClient(Node):

    """
    The init function is neccesary for the sealerNode class to initialize all variables, parameters, and other functions.
    Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
    """

    def __init__(self, TEMP_NODE_NAME = "sealerNode"):
        """Setup sealer node"""

        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()


        self.declare_parameter('sealer_port', '/dev/ttyUSB1')       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        self.PORT = self.get_parameter('sealer_port').get_parameter_value()._string_value    # Renaming parameter to general form so it can be used for other nodes too
        
        self.get_logger().info("Received Port: " + str(self.PORT))
        self.state = "UNKOWN"
        self.connect_robot()

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

    def connect_robot(self):
        """Connect robot"""
        try:
            self.sealer = A4S_SEALER_DRIVER(self.PORT)
        except Exception as err:
            self.get_logger.error("SEALER CONNECTION ERROR")
            self.state = "SEALER CONNECTION ERROR"
            self.get_logger.warn("Trying to connect againg! PORT: ", str(self.PORT))
            self.connect_robot()
        else:
            self.get_logger.info("Sealer is online")

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
        
        try:
            state = self.pf400.movement_state
            self.pf400.get_overall_state()

        except Exception as err:
            self.get_logger().error("ROBOT IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "SEALER CONNECTION ERROR"

        msg = String()
        msg.data = "State %s" % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.state = "READY"


def main(args=None):  

    TEMP_NODE_NAME = "sealerNode"   

    rclpy.init(args=args)  # initialize Ros2 communication

    node = sealerNode(TEMP_NODE_NAME=TEMP_NODE_NAME)

    rclpy.spin(node)  # keep Ros2 communication open for action node

    rclpy.shutdown()  # kill Ros2 communication

    rclpy.init(args=args)  # initialize Ros2 communication

    try:
        pf400_client = PF400ClientNode()
        executor = MultiThreadedExecutor()
        executor.add_node(pf400_client)

        try:
            pf400_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            pf400_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            pf400_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":

    main()
