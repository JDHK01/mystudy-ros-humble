import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from interface_demo.action import Process


import time

class MyActionServer(Node):
    def __init__(self, node_name, action_name):
        super().__init__(node_name)
        self.get_logger().info(f"创建节点{node_name}")
        self.action_server = ActionServer(
            self,
            Process,
            action_name,
            self.execute_callback
        )
        self.get_logger().info(f"创建动作通信服务器{action_name}")

    def execute_callback(self, goal_handle):
        duration = goal_handle.request.duration
        self.get_logger().info(f"收到{duration}")

        start = time.time()
        while rclpy.ok():
            now = time.time()
            elapse = now - start 

            if  elapse >= duration :
                goal_handle.succeed()
                self.get_logger().info("动作完成")
                result = Process.Result()
                result.elapse = elapse
                return result
            else:
                feedback = Process.Feedback()
                feedback.process = int((now-start) * 100 / duration)
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(f"动作进度{feedback.process}")

                time.sleep(0.1)


def main():
    rclpy.init()
    action_serve_node = None
    try:
        action_serve_node = MyActionServer(action_name='timing',node_name='action_serve_node')
        rclpy.spin(action_serve_node)
    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"发生错误{e}")
    finally:
        if action_serve_node is not None:
            action_serve_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
