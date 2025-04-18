import logging
import random
import time
import signal
import sys
import atexit
import carla

from simulation import config
from simulation.sensors import (
    GnssSensor,
    IMUSensor,
    LidarSensor,
    RgbCamera,
)
import argparse

# 全局变量用于资源清理
_vehicle_actor = None
_sensors = []
_world = None

def check_actor_exists(actor_id):
    """检查指定ID的Actor是否仍存在"""
    global _world
    try:
        actor = _world.get_actor(actor_id)
        if actor:
            return True
        return False
    except:
        return False

def cleanup_resources():
    """清理所有创建的资源"""
    global _vehicle_actor, _sensors
    
    # 清理传感器
    for sensor in _sensors:
        sensor.sensor.destroy()
    time.sleep(0.5)  # 等待传感器销毁完成
    
    # 清理车辆
    if _vehicle_actor:
        _vehicle_actor.destroy()
    time.sleep(1)  # 等待车辆销毁完成

def signal_handler(sig, frame):
    """处理终止信号"""
    logging.info(f"收到信号 {sig}，准备退出...")
    cleanup_resources()
    sys.exit(0)


def main():
    global _vehicle_actor, _sensors, _world
    
    try:
        # 注册信号处理
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # 注册退出清理
        atexit.register(cleanup_resources)

        argparser = argparse.ArgumentParser(description='spawn carla vehicle')
        argparser.add_argument('--rolename', metavar='NAME', default='v1', help='actor role name (default: "v1")')
        argparser.add_argument('--position', metavar='POSITION', default='random', help='Initial position of the vehicle. Format: x,y,z,pitch,yaw,roll. Fill random for randomized position.')
        argparser.add_argument('--host', metavar="HOST", default='172.17.0.1', help="IP of the Carla server (default:127.0.0.1)")#172.17.0.1
        argparser.add_argument('--port', metavar="PORT", default=2000, type=int, help="Port of the Carla server (default:2000)")  
        argparser.add_argument('--vehicle', metavar="VEHICLE", default='vehicle.tesla.model3', help="Vehicle blueprint (default:vehicle.tesla.model3)")
        args = argparser.parse_args()

        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)

        sim_world = client.get_world()
        log_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
        logging.info('begin to spawn vehicles')

        if args.position != 'random':
            position = args.position.split(',')
            vehicle_position = carla.Transform(
                carla.Location(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                carla.Rotation(pitch=float(position[3]), yaw=float(position[4]), roll=float(position[5])),
            )
        
        vehicles = sim_world.get_blueprint_library().filter(args.vehicle)
        blueprint = random.choice(vehicles)
        blueprint.set_attribute('role_name', 'autoware_' + args.rolename)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):  #车辆变得 无敌 
            blueprint.set_attribute('is_invincible', 'true')


        # spawn
        if args.position == 'random':
            spawn_point = random.choice(sim_world.get_map().get_spawn_points())
        else:
            spawn_point = vehicle_position
        vehicle_actor = sim_world.try_spawn_actor(blueprint, spawn_point)
        # 车辆生成失败，重新生成
        attempts = 0
        max_attempts = 10
        while vehicle_actor is None and attempts < max_attempts:
            attempts += 1
            logging.warning(f"车辆生成失败，正在尝试第 {attempts} 次重新生成...")
            # 随机选择一个生成点
            spawn_point = random.choice(sim_world.get_map().get_spawn_points())
            vehicle_actor = sim_world.try_spawn_actor(blueprint, spawn_point)

        
        logging.info("finding vehicle")
        vehicle_actors = sim_world.get_actors().filter('vehicle.*')
        if vehicle_actors is None:
            logging.info("没有找到车辆")
        else:
            logging.info(f"找到车辆数量: {len(vehicle_actors)}")

        # for vehicle in vehicle_actors:
        #     logging.info(f"车辆ID: {vehicle.id}, 角色名称: {vehicle.attributes['role_name']}")
        #     if vehicle.attributes['role_name'] == 'autoware_' + args.rolename:
        #         logging.info(f"找到车辆: {vehicle.attributes['role_name']}")
        #         vehicle_actor = vehicle
        #         break

        
        # Set sweep_wheel_collision
        # 这个是增强的车轮碰撞检测
        physics_control = vehicle_actor.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        vehicle_actor.apply_physics_control(physics_control)

        # Setup sensors
        _gnss_sensor = GnssSensor(vehicle_actor, sensor_name='ublox')
        _imu_sensor = IMUSensor(vehicle_actor, sensor_name='tamagawa')
        _lidar_sensor = LidarSensor(vehicle_actor, sensor_name='top')
        _rgb_camera = RgbCamera(vehicle_actor, sensor_name='traffic_light')

        # 保存全局变量
        _world = sim_world
        _vehicle_actor = vehicle_actor
        _sensors = [_gnss_sensor, _imu_sensor, _lidar_sensor, _rgb_camera]

        while True:
            time.sleep(1)
    finally:
        pass
        # _gnss_sensor.sensor.destroy()
        # _imu_sensor.sensor.destroy()
        # _lidar_sensor.sensor.destroy()
        # _rgb_camera.sensor.destroy()
        # vehicle_actor.destroy()


if __name__ == '__main__':
    main()
