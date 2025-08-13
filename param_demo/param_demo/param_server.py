import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter


class MyParamServer(Node):
    def __init__(self, node_name:str = 'default_node'):
        # 声明允许删除参数
        super().__init__(node_name, allow_undeclared_parameters=True)
        self.get_logger().info(f"创建节点{node_name}")
    
    # 增
    def declare_param(self, name, value=None):
        self.declare_parameter(name=name, value=value)
        self.get_logger().info(f"新增参数   参数名:{name}, 参数值:{value}")

    # 删
    def delete_param(self, name):
        if self.has_parameter(name):
            self.get_logger().info(f"删除参数   参数名:{name}, 参数值:{self.get_parameter(name).value}")
            self.undeclare_parameter(name)
        


    # 改
    def set_param(self,name,value=None):
        if isinstance(name, str) and value is not None:
            self.get_logger().info(f"设置参数   参数名{name}, 参数值:{value}")
            if not self.has_parameter(name):
                self.declare_parameter(name=name, value=value)
            else:
                p = Parameter(name, value)
                self.set_parameters([p])

        if isinstance(name, dict):
            existing_param = []
            for param_name, param_value in name.items():
                if not self.has_parameter(param_name):
                    self.declare_parameter(param_name, param_value)
                    self.get_logger().info(f"设置参数   参数名{param_name}, 参数值:{param_value}")
                else:
                    p = Parameter(param_name, value = param_value)
                    self.get_logger().info(f"设置参数   参数名{param_name}, 参数值:{param_value}")
                    existing_param.append(p)
            self.set_parameters(existing_param)


    # 查
    def get_param(self, name):
        if not isinstance(name, list):
            # 获取单个参数
            if not self.has_parameter(name):
                self.get_logger().info(f"没有{name}参数")
                return
            self.get_logger().info(f"寻找参数   {name}")
            temp_param = self.get_parameter(name)
            self.get_logger().info(f"参数键:{temp_param.name},参数值:{temp_param.value}")
            return temp_param
        else:
            missing_param = []
            existing_param = []
            for p in name:
                if not self.has_parameter(p):
                    missing_param.append(p)
                else:
                    existing_param.append(p)
            if missing_param:
                self.get_logger().info(f"这些参数:{missing_param}不存在")
            if existing_param:
                params = self.get_parameters(existing_param)
                for p in params:
                    self.get_logger().info(f"参数键:{p.name},参数值:{p.value}")
                return existing_param


def main():
    rclpy.init()
    param_server = None
    try:
        # 增
        param_server = MyParamServer(node_name='param_server_node')
        param_server.declare_param(name='J', value='10')
        param_server.declare_param(name='D', value='04')
        param_server.declare_param(name='H', value='08')
        param_server.declare_param(name='K', value='11')

        # 删
        param_server.delete_param('D')

        # 查
        param_server.get_param('J')
        param_server.get_param(['j','J','D','H','K'])

        # 改
        param_server.set_param('Q', '99')
        param_server.set_param(
        {
            'A':'1',
            'B':'2',
            'C':'3',
            'D':'4',
            'E':'5'
        }
        )

        rclpy.spin(param_server)


    except KeyboardInterrupt:
        print("用户中断")
    except Exception as e:
        print(f"发生错误{e}")
    finally:
        if param_server is not None:
            param_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()