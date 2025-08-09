import rclpy
def print_helloworld():
    # 初始化ros2
    rclpy.init()

    try:
        # 创建一个节点
        node = rclpy.create_node("helloworld_node")
        # 打印信息
        print("普通日志:")
        node.get_logger().info("Hello world!")
        print("警告日志:")
        node.get_logger().warn("Hello world!")
        print("错误日志:")
        node.get_logger().error("Hello world!")

    except Exception as e:
        print(f"发生异常: {e}")

    finally:
        # 销毁节点
        node.destroy_node()
        # 关闭ros2
        rclpy.shutdown()

if __name__ == '__main__':
    print_helloworld()
