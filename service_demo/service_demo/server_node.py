#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from rclpy.executors import MultiThreadedExecutor
from interface_demo.srv import Distance2origin
import numpy as np


class MyServer(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.logger = self.get_logger()

        self.server = self.create_service(Distance2origin, 'distance_server', self.distance_3d)
        self.logger.info(f'创建节点: {node_name}')

    def distance_3d(self, request, response):
        point1 = np.array([request.x, request.y, request.z])
        point2 = np.array([0, 0, 0])
        response.distance = np.linalg.norm(point2 - point1)
        self.logger.info(f"\n输入:{request.x},{request.y},{request.z}\n输出:{response.distance:.4f}")
        return response

def main():
    rclpy.init()
    node1 = None  # 初始化node变量
    # node2 = None
    # executor = MultiThreadedExecutor()  # 多任务执行器

    try:
        node1 = MyServer('server_node')
        # node2 = MyServer('str_helloworld_node2')
        # executor.add_node(node1)
        # executor.add_node(node2)
        # executor.spin()
        rclpy.spin(node1)
    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if node1 is not None:
            node1.destroy_node()
        # if node2 is not None:
            # node2.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()