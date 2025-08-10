import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("MinimalPublisher")
        self.time_period = 1.0
        self.times = 1
        self.log = self.get_logger()

        self.publisher = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(self.time_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = str(self.times) + ':' + 'Helloworld'
        self.publisher.publish(msg)
        self.log.info(f"发布消息:{msg.data}")
        self.times += 1

def print_helloworld():
    if not rclpy.ok():
        rclpy.init()
    minimal_publisher = None
    try:
        minimal_publisher = MinimalPublisher()
        rclpy.spin(minimal_publisher)

    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 销毁节点
        if minimal_publisher is not None:
            minimal_publisher.destroy_node()
        # 关闭rclpy
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"关闭ROS2时出错: {e}")

if __name__ == '__main__':
    print_helloworld()
