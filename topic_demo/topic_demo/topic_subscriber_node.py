import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self._logger = self.get_logger()
        # 创建订阅方
        self.subscriber = self.create_subscription(String, topic_name, self.subscriber_callback, 10)
        self._logger.info(f"话题名{topic_name}, 订阅者创建")

    def subscriber_callback(self, msg):
        self._logger.info(f"收到消息{msg.data}")

def receive_helloworld():
    rclpy.init()
    try: 
        minimal_subscriber = None
        minimal_subscriber = MinimalSubscriber(node_name='topic_subscriber_node', topic_name='topic')
        rclpy.spin(minimal_subscriber)

    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 销毁节点
        if minimal_subscriber is not None:
            minimal_subscriber.destroy_node()
        # 关闭rclpy
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    receive_helloworld()

