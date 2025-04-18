
# import time

# import zenoh


# def main(conf: zenoh.Config, key: str, payload: str, iter: int, interval: int):
#     # initiate logging
#     zenoh.init_log_from_env_or("error")

#     print("Opening session...")
#     with zenoh.open(conf) as session:

#         print(f"Declaring Publisher on '{key}'...")
#         pub = session.declare_publisher(key)

#         print("Press CTRL-C to quit...")
#         for idx in itertools.count() if iter is None else range(iter):
#             time.sleep(interval)
#             buf = f"[{idx:4d}] {payload}"
#             print(f"Putting Data ('{key}': '{buf}')...")
#             pub.put(buf)


# # --- Command line argument parsing --- --- --- --- --- ---
# if __name__ == "__main__":
#     import argparse
#     import itertools
#     import json

#     parser = argparse.ArgumentParser(prog="z_pub", description="zenoh pub example")
#     parser.add_argument(
#         "--mode",
#         "-m",
#         dest="mode",
#         choices=["peer", "client"],
#         type=str,
#         help="The zenoh session mode.",
#     )
#     parser.add_argument(
#         "--connect",
#         "-e",
#         dest="connect",
#         metavar="ENDPOINT",
#         action="append",
#         type=str,
#         help="Endpoints to connect to.",
#     )
#     parser.add_argument(
#         "--listen",
#         "-l",
#         dest="listen",
#         metavar="ENDPOINT",
#         action="append",
#         type=str,
#         help="Endpoints to listen on.",
#     )
#     parser.add_argument(
#         "--key",
#         "-k",
#         dest="key",
#         default="demo/example/zenoh-python-pub",
#         type=str,
#         help="The key expression to publish onto.",
#     )
#     parser.add_argument(
#         "--payload",
#         "-p",
#         dest="payload",
#         default="Pub from Python!",
#         type=str,
#         help="The payload to publish.",
#     )
#     parser.add_argument(
#         "--iter", dest="iter", type=int, help="How many puts to perform"
#     )
#     parser.add_argument(
#         "--interval",
#         dest="interval",
#         type=float,
#         default=1.0,
#         help="Interval between each put",
#     )
#     parser.add_argument(
#         "--config",
#         "-c",
#         dest="config",
#         metavar="FILE",
#         type=str,
#         help="A configuration file.",
#     )

#     args = parser.parse_args()
#     conf = (
#         zenoh.Config.from_file(args.config)
#         if args.config is not None
#         else zenoh.Config()
#     )
#     if args.mode is not None:
#         conf.insert_json5("mode", json.dumps(args.mode))
#     if args.connect is not None:
#         conf.insert_json5("connect/endpoints", json.dumps(args.connect))
#     if args.listen is not None:
#         conf.insert_json5("listen/endpoints", json.dumps(args.listen))

#     main(conf, args.key, args.payload, args.iter, args.interval)



import time
import zenoh
import rclpy
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
from autoware_adapi_v1_msgs.msg import AccelerationCommand
from autoware_adapi_v1_msgs.msg import VehicleKinematics

# 初始化 ROS 2 节点（可选，仅用于创建消息）
rclpy.init()

# 创建 Zenoh 会话
session = zenoh.open(zenoh.Config())

# 创建加速度命令消息
def create_acceleration_command(accel_value):
    msg = AccelerationCommand()
    
    # 设置当前时间戳
    now = Time()
    now.sec = int(time.time())
    now.nanosec = int((time.time() % 1) * 1e9)
    msg.stamp = now
    
    # 设置加速度值
    msg.acceleration = float(accel_value)
    
    # 序列化消息
    serialized_msg = serialize_message(msg)
    return serialized_msg

# 发布消息
def publish_acceleration(accel_value):
    key = "v1/api/vehicle/command/acceleration"
    serialized_msg = create_acceleration_command(accel_value)
    session.put(key, serialized_msg)
    print(f"已发送加速度命令: {accel_value} m/s²")

# 示例：发送加速度命令
publish_acceleration(1.5)  # 发送 1.5 m/s² 的加速命令

# 清理
session.close()
rclpy.shutdown()