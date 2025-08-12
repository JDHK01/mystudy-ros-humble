import rclpy

from rclpy.node import Node

from rclpy.action import ActionClient
from interface_demo.action import Process
from rclpy.task import Future

import time




class MyActionClient(Node):
    def __init__(self, node_name='default_node', action_name='default_action'):
        super().__init__(node_name)
        self.get_logger().info(f"创建节点{node_name}")
        self.action_client = ActionClient(
            self,
            Process,
            action_name,
        )
        self.completed_future = Future()

    def wait_for_action_server_with_backoff(self, timeout=10, initial_wait=0.3, max_wait=2):
        start = time.time()
        wait = initial_wait
        while rclpy.ok():
            if self.action_client.wait_for_server(wait):
                self.get_logger().info("连接服务器成功")
                return True
            
            elapsed = time.time() - start
            if elapsed > timeout:
                self.get_logger().info("连接服务器超时")
                return False
            
            wait = min(wait*2, max_wait)
            self.get_logger().info(f"尝试连接{elapsed:.1f}秒")
        return False
    
    def send_goal(self, duration):   
        # 检查服务器是否连接正常
        if not self.action_client.server_is_ready():
            self.get_logger().error("服务器未就绪")
            self.completed_future.set_result(False)
            return False
        # 填充信息
        goal = Process.Goal()
        goal.duration = duration
        # 发送
        # 第一个future, 是否被接受
        self.future = self.action_client.send_goal_async(goal, self.fb_callback)
        self.future.add_done_callback(self.goal_response_callback)

    def fb_callback(self, fb_msg):
        feedback = fb_msg.feedback
        self.get_logger().info(f"连续反馈信息:{feedback.process}")    

    def goal_response_callback(self, future):
        # 取出句柄, 判断是否被接受
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("请求被拒绝")
            self.completed_future.set_result(False)
            return
        self.get_logger().info("请求成功")
        # 第二个future, 获取处理结果
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result_handle = future.result()
        result = result_handle.result
        self.completed_future.set_result(True)
        self.get_logger().info(f"处理结果:{result.elapse}")



def main():
    rclpy.init()
    action_client = None

    try:
        action_client = MyActionClient(node_name='action_client_node', action_name='timing')
        if not action_client.wait_for_action_server_with_backoff():
            action_client.get_logger().warning("连接失败")
            return 1
        action_client.get_logger().info("连接成功")
    
        action_client.send_goal(3.0)

        # rclpy.spin(action_client)
        rclpy.spin_until_future_complete(action_client, action_client.completed_future)

    except KeyboardInterrupt:
        print("用户终止")
    except Exception as e:
        print(f"发生意外{e}")        
    finally:
        if action_client is not None:
            action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()