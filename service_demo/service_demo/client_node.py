#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

from rclpy.logging import get_logger
import sys, time

from interface_demo.srv import Distance2origin

class MyClient(Node):
    def __init__(self, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.get_logger().info('客户端创建')
        self.client = self.create_client(Distance2origin, topic_name)
        self.future = None

    def wait_for_service_with_backoff(self, timeout=10, initial_wait=0.3, max_wait=2):
        start = time.time()
        wait = initial_wait
        while rclpy.ok():
            if self.client.wait_for_service(timeout_sec=wait):
                self.get_logger().info("连接服务器成功")
                return True
            
            elapsed = time.time() - start
            if elapsed > timeout:
                self.get_logger().info("连接服务器超时")
                return False
            
            wait = min(wait*2, max_wait)
            self.get_logger().info(f"尝试连接{elapsed:.1f}秒")
        return False
    
    def send_request(self, x, y, z, timeout=10):
        request = Distance2origin.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=timeout)

        if not self.future.done():
            raise TimeoutError("服务调用超时")
        else:
            return self.future.result()

def main():
    if(len(sys.argv)) != 4:
        get_logger('dump').info("参数不是三个, 使用默认参数:x=1,y=2,z=3")
        sys.argv[1] = 1
        sys.argv[2] = 2
        sys.argv[3] = 3
    
    rclpy.init()
    node = None
    try:
        node = MyClient(topic_name='distance_server', node_name='client_node')
        if not node.wait_for_service_with_backoff():
            node.get_logger().info('连接服务器失败')
            return 1
        node.get_logger().info("连接服务器成功")

        try:
            response = node.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
            node.get_logger().info(f"服务器返回结果{response.distance}")
        except TimeoutError:
            node.get_logger().error('服务调用超时')
            return 2
        except Exception as e:
            node.get_logger().error(f'调用失败: {e}')
            return 3
        
    except KeyboardInterrupt:
        print("\n程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        # if node2 is not None:
        #     node2.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()