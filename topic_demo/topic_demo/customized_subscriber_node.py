import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interface_demo.msg import Position

class CustomizedSubscriber(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self._logger = self.get_logger()
        # 创建订阅方
        self.subscriber = self.create_subscription(Position, topic_name, self.subscriber_callback, 10)
        self._logger.info(f"话题名{topic_name}, 订阅者创建")

    def subscriber_callback(self, msg):
        self._logger.info(f"收到消息{msg}")

def receive_position():
    rclpy.init()
    try: 
        customized_subscriber = None
        customized_subscriber = CustomizedSubscriber(node_name='topic_subscriber_node', topic_name='topic')
        rclpy.spin(customized_subscriber)

    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 销毁节点
        if customized_subscriber is not None:
            customized_subscriber.destroy_node()
        # 关闭rclpy
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    receive_position()

