import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from autoware_adapi_v1_msgs.msg import RouteState, LocalizationInitializationState, Heartbeat
import argparse

class AutowareAPISubscriber(Node):
    def __init__(self, topic_name, msg_type, comm_type='notification'):
        super().__init__('autoware_api_subscriber')
        
        # 根据通信类型选择 QoS 配置
        if comm_type == 'notification':
            # Notification: reliable + transient_local
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
            self.get_logger().info(f'使用 Notification QoS (reliable + transient_local) 订阅: {topic_name}, 消息类型: {msg_type}, 通信类型: {comm_type}')
            
        elif comm_type == 'reliable_stream':
            # Reliable Stream: reliable + volatile
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
            self.get_logger().info(f'使用 Reliable Stream QoS (reliable + volatile) 订阅: {topic_name}, 消息类型: {msg_type}, 通信类型: {comm_type}')
            
        elif comm_type == 'realtime_stream':
            # Realtime Stream: best_effort + volatile
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
            self.get_logger().info(f'使用 Realtime Stream QoS (best_effort + volatile) 订阅: {topic_name}, 消息类型: {msg_type}, 通信类型: {comm_type}')
            
        else:
            # 默认使用 Notification 设置
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
            self.get_logger().info(f'使用默认 Notification QoS 订阅: {topic_name}')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.message_callback,
            qos
        )
        
        self.topic_name = topic_name
        self.comm_type = comm_type
        self.msg_count = 0
    
    def message_callback(self, msg):
        """处理 API 消息"""
        self.msg_count += 1
        
        # 打印通用消息信息
        self.get_logger().info(f'收到 {self.comm_type} 类型消息 #{self.msg_count} 来自主题: {self.topic_name}')
        
        # 针对特定消息类型的处理
        if hasattr(msg, 'state'):
            self.get_logger().info(f'状态值: {msg.state}')
            
            # 根据消息类型提供不同的状态描述
            if self.topic_name == '/api/routing/state':
                state_descriptions = {
                    0: "无路线",
                    1: "路线设置中",
                    2: "已设置路线"
                }
                state_desc = state_descriptions.get(msg.state, f"未知状态({msg.state})")
                self.get_logger().info(f'路线状态: {state_desc}')
                
            elif self.topic_name == '/api/localization/initialization_state':
                state_descriptions = {
                    0: "未初始化",
                    1: "初始化中",
                    2: "已初始化"
                }
                state_desc = state_descriptions.get(msg.state, f"未知状态({msg.state})")
                self.get_logger().info(f'定位初始化状态: {state_desc}')
                
            else:
                self.get_logger().info(f'状态值: {msg.state}')
                
        # 打印完整消息
        self.get_logger().info(f'完整消息: {msg}')

def main():
    parser = argparse.ArgumentParser(description='Autoware API 订阅者')
    parser.add_argument('--topic', '-t', type=str, default='/api/routing/state',
                        help='要订阅的 API 主题名称')
    # parser.add_argument('--topic', '-t', type=str, default='/vehicle/status/control_mode',
    #                     help='要订阅的 API 主题名称')
    parser.add_argument('--type', '-m', type=str, default='route_state',
                        help='API 消息类型 (route_state, localization_init_state)')
    parser.add_argument('--comm', '-c', type=str, default='notification',
                        choices=['notification', 'reliable_stream', 'realtime_stream'],
                        help='通信类型')
    args = parser.parse_args()
    
    rclpy.init()
    
    # 根据参数选择消息类型
    msg_type_map = {
        'route_state': RouteState,
        'localization_init_state': LocalizationInitializationState,
        'heartbeat': Heartbeat,  # 示例消息类型
    }
    
    msg_type = msg_type_map.get(args.type)
    if not msg_type:
        print(f"错误: 未知的消息类型 '{args.type}'")
        print(f"可用选项: {', '.join(msg_type_map.keys())}")
        return
    
    # 创建订阅者
    subscriber = AutowareAPISubscriber(args.topic, msg_type, args.comm)
    
    print(f"\n开始订阅 {args.topic}")
    print(f"消息类型: {args.type}")
    print(f"通信类型: {args.comm}")
    print("按 Ctrl+C 停止\n")
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()