import rclpy

from rclpy.node import Node
from rclpy.parameter import AsyncParameterClient

class MyParamNode(Node):
    def __init__(self, node_name, server_name):
        super().__init__(node_name)
        self.param_client = rclpy.pa