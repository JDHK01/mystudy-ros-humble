import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from interface_demo.msg import Position
import random

class CustomizedPublisher(Node):
    def __init__(self, node_name:str, topic_name:str):
        super().__init__(node_name)
        self.time_period = 1.0
        self.count = 1# 计数器
        self.node_name = node_name
        self._logger = self.get_logger()
        # 创建发布者对象
        self.publisher = self.create_publisher(Position, topic_name, 10)
        self._logger.info(f"话题名{topic_name}, 发布者创建")
        # 创建定时器
        self.timer = self.create_timer(self.time_period, self.timer_callback)

    def timer_callback(self):
        '定期发布数据'
        msg = Position()
        msg.x = random.randint(1, 10)
        msg.y = random.randint(1, 10)
        msg.z = random.randint(1, 10)
        msg.yaw = random.uniform(5, 15)
        self.publisher.publish(msg)
        # 日志 + 迭代
        self._logger.info(f"发布消息:{msg}")
        self.count += 1

def print_position():

    rclpy.init()

    customized_publisherA = None
    customized_publisherB = None
    executor = MultiThreadedExecutor()# 多任务执行器

    try:
        customized_publisherA = CustomizedPublisher(node_name='topic_publisher_node1', topic_name='topic')
        customized_publisherB = CustomizedPublisher(node_name='topic_publisher_node2', topic_name='topic')
        executor.add_node(customized_publisherA)
        executor.add_node(customized_publisherB)
        executor.spin()
    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 销毁节点
        if customized_publisherA is not None:
            customized_publisherA.destroy_node()
        if customized_publisherB is not None:
            customized_publisherB.destroy_node()
        # 关闭rclpy
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    print_position()
