#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

import time



class MyParamClient(Node):
    def __init__(self, node_name = "default_node", param_server_name : str = None):
        super().__init__(node_name)
        self.get_logger().info(f"创建节点:{node_name}")
        # 分别创建三个节点
        """
        ros2 service list
            /param_server_node/describe_parameters
            /param_server_node/get_parameter_types
            /param_server_node/get_parameters
            /param_server_node/list_parameters
            /param_server_node/set_parameters
            /param_server_node/set_parameters_atomically
        """
        
        # 列举所有参数
        self.list_param_client = self.create_client(
            ListParameters,
            '/' + param_server_name + '/' + 'list_parameters'
        )
        self.get_logger().info(f"创建客户端:{self.list_param_client.srv_name}")
        self.set_param_client = self.create_client(
            SetParameters,
            '/' + param_server_name + '/' + 'set_parameters'
        )
        self.get_logger().info(f"创建客户端:{self.set_param_client.srv_name}")
        self.get_param_client = self.create_client(
            GetParameters,
            '/' + param_server_name + '/' + 'get_parameters'
        )
        self.get_logger().info(f"创建客户端:{self.get_param_client.srv_name}")

        self.list_param_client.wait_for_service()

    def wait_for_service_with_backoff(self, timeout=10, initial_wait=0.3, max_wait=2):
        start = time.time()
        wait = initial_wait
        services_ready = [
            self.list_param_client.wait_for_service(timeout_sec=wait),
            self.set_param_client.wait_for_service(timeout_sec=wait),
            self.get_param_client.wait_for_service(timeout_sec=wait)
        ]
        while rclpy.ok():
            if all(services_ready):
                self.get_logger().info("连接服务器成功")
                return True
            
            elapsed = time.time() - start
            if elapsed > timeout:
                self.get_logger().info("连接服务器超时")
                return False
            
            wait = min(wait*2, max_wait)
            self.get_logger().info(f"尝试连接{elapsed:.1f}秒")
        return False
    
    def list_param(self):
        """
        ros2 interface show rcl_interfaces/srv/ListParameters --no-comments 
        
        ListParameters
            uint64 DEPTH_RECURSIVE=0
            string[] prefixes
            #
            uint64 depth
            ---
            ListParametersResult result
                    string[] names
                    string[] prefixes
        """
        request = ListParameters.Request()

        future = self.list_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            param_list = future.result().result.names
            self.get_logger().info(f"参数服务器的参数列表:{param_list}")
            return param_list
        else:
            self.get_logger().error("无法获取参数列表")

    def get_param(self, param_name):
        """
        ros2 interface show rcl_interfaces/srv/GetParameters --no-comments 
        GetParameters
            
            string[] names
            ---
            ParameterValue[] values
                    uint8 type
                    bool bool_value
                    int64 integer_value
                    float64 double_value
                    string string_value
                    byte[] byte_array_value
                    bool[] bool_array_value
                    int64[] integer_array_value
                    float64[] double_array_value
                    string[] string_array_value
        """
        type_to_attr = {
            ParameterType.PARAMETER_BOOL: 'bool_value',
            ParameterType.PARAMETER_INTEGER: 'integer_value',
            ParameterType.PARAMETER_DOUBLE: 'double_value',
            ParameterType.PARAMETER_STRING: 'string_value',
            ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array_value',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
            ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value',
        }
        # 这是一个便利函数, 接受单个字符串的传入, 防止单个字符串还用列表影响美观
        if isinstance(param_name, str):
            param_name = [param_name]
        request = GetParameters.Request()
        request.names = param_name
        future = self.get_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not Node:
            param_dict = {}
            response = future.result()
            for i, name in enumerate(param_name):
                value = response.values[i]
                value = getattr(value, type_to_attr[value.type])
                param_dict[name] = value
                self.get_logger().info(f"获取参数    属性名:{name}, 属性值:{value}")
        else:
            self.get_logger().error("无法获取属性值")

    def set_param(self, param_dict:dict):
        """
        ros2 interface show rcl_interfaces/srv/SetParameters --no-comments 
        SetParameters
            Parameter[] parameters
                    string name
                    ParameterValue value
                            uint8 type
                            bool bool_value
                            int64 integer_value
                            float64 double_value
                            string string_value
                            byte[] byte_array_value
                            bool[] bool_array_value
                            int64[] integer_array_value
                            float64[] double_array_value
                            string[] string_array_value
            ---
            SetParametersResult[] results
                    bool successful
                    string reason
        """
        request = SetParameters.Request()
        for name, value in param_dict.items():
            param = Parameter()
            param.name = name
            param_value = ParameterValue()
            if isinstance(value, bool):
                param_value.type = ParameterType.PARAMETER_BOOL
                param_value.bool_value = value
            elif isinstance(value, int):
                param_value.type = ParameterType.PARAMETER_INTEGER
                param_value.integer_value = value
            elif isinstance(value, float):
                param_value.type = ParameterType.PARAMETER_DOUBLE
                param_value.double_value = value
            elif isinstance(value, str):
                param_value.type = ParameterType.PARAMETER_STRING
                param_value.string_value = value
            else:
                self.get_logger().warn(f"不支持的参数类型: {type(value)}")
                continue
            param.value = param_value
            request.parameters.append(param)

        future = self.set_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response:SetParameters.Response = future.result()
            for i,r in enumerate(response.results):
                if r.successful:
                    self.get_logger().info(f"成功设置参数   参数名:{request.parameters[i].name}")
                else:
                    self.get_logger().info(f"设置参数失败   参数名:{request.parameters[i].name}")
                    print(r.reason)
        else:
            self.get_logger().error("无法获取属性值")

def main():
    rclpy.init()
    param_client = None
    try:
        param_client = MyParamClient('param_client', 'param_server_node')
        
        param_client.wait_for_service_with_backoff(timeout=10)

        param_client.list_param()

        param_client.get_param(['J', 'D', 'H', 'K', 'A', 'B'])

        param_client.set_param({
            'A':'9999',
            'B':'9999',
            'Y': 9999,
            # 'Y': '9999',
            'Y':9999,
        })
    except KeyboardInterrupt:
        print("被用户中断")
    except Exception as e:
        print(f"发生错误{e}")
    finally:
        if param_client is not None:
            param_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()