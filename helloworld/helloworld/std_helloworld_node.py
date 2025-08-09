import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor



class MyNode(Node):
    def __init__(self, node_name:str):
        super().__init__(node_name)
        self.name = node_name
        self.count = 0
        self.logger = self.get_logger()


        print(f'创建节点: {node_name}')
        print("普通日志:")
        self.get_logger().info("Hello world!")
        print("警告日志:")
        self.get_logger().warn("Hello world!")
        print("错误日志:")
        # self.get_logger().error("Hello world!")
        self.logger.error("Hello world")

        print("创建定时器")
        self.timer = self.create_timer(1, self.timercallback)

    def timercallback(self):
        self.count += 1
        self.logger.info(f"节点{self.name}已启动{self.count}秒")
        # print(F'节点{self.name}已启动{self.count}秒')
        
def print_helloworld():
    rclpy.init()
    node = None  # 初始化node变量
    executor = MultiThreadedExecutor()# 多任务执行器

    try:
        node1 = MyNode('str_helloworld_node')
        node2 = MyNode('str_helloworld_node2')
        executor.add_node(node1)
        executor.add_node(node2)
        executor.spin()
    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if node1 is not None:
            node1.destroy_node()
        if node2 is not None:
            node2.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    print_helloworld()