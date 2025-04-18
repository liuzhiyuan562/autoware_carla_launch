import logging
import random
import carla
import signal
import sys

from simulation import config
from simulation.sensors import (
    GnssSensor,
    IMUSensor,
    LidarSensor,
    RgbCamera,
)

vehicle_list = [
    ('v1', '87.687683,145.671295,0.300000,0.000000,90.000053,0.000000'),
    # ('v2', '92.109985,227.220001,0.300000,0.000000,-90.000298,0.000000'),
]

# 全局变量存储所有需要清理的对象
actors_to_destroy = []

def signal_handler(sig, frame):
    """处理退出信号，确保清理资源"""
    print('正在清理资源并退出...')
    cleanup()
    sys.exit(0)

def cleanup():
    """清理所有CARLA资源"""
    print(f"销毁 {len(actors_to_destroy)} 个actors...")
    
    # 首先停止所有传感器
    for actor in actors_to_destroy:
        if isinstance(actor, (GnssSensor, IMUSensor, LidarSensor, RgbCamera)):
            try:
                actor.stop()
            except Exception as e:
                print(f"停止传感器时出错: {e}")
    
    # 然后销毁所有actor
    for actor in actors_to_destroy:
        try:
            if hasattr(actor, 'destroy') and callable(actor.destroy):
                actor.destroy()
        except Exception as e:
            print(f"销毁actor时出错: {e}")
    
    actors_to_destroy.clear()
    print("清理完成")

def main():
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # kill

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(20.0)

    sim_world = client.load_world(config.SIM_WORLD)
    log_level = logging.DEBUG
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('begin to spawn vehicles')

    try:
        for vehicle_name, position in vehicle_list:
            # vehicles settings
            vehicles = sim_world.get_blueprint_library().filter('vehicle.tesla.model3')
            blueprint = random.choice(vehicles)
            blueprint.set_attribute('role_name', 'autoware_' + vehicle_name)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if blueprint.has_attribute('is_invincible'):
                blueprint.set_attribute('is_invincible', 'true')

            # spawn
            position = position.split(',')
            spawn_point = carla.Transform(
                carla.Location(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                carla.Rotation(pitch=float(position[3]), yaw=float(position[4]), roll=float(position[5])),
            )
            vehicle_actor = sim_world.try_spawn_actor(blueprint, spawn_point)
            
            # 添加到清理列表
            if vehicle_actor:
                actors_to_destroy.append(vehicle_actor)
                
                # 设置物理控制
                physics_control = vehicle_actor.get_physics_control()
                physics_control.use_sweep_wheel_collision = True
                vehicle_actor.apply_physics_control(physics_control)

                # 设置传感器
                gnss_sensor = GnssSensor(vehicle_actor, sensor_name='ublox')
                imu_sensor = IMUSensor(vehicle_actor, sensor_name='tamagawa')
                lidar_sensor = LidarSensor(vehicle_actor, sensor_name='top')
                rgb_camera = RgbCamera(vehicle_actor, sensor_name='traffic_light')
                
                # 将传感器添加到清理列表
                actors_to_destroy.extend([gnss_sensor, imu_sensor, lidar_sensor, rgb_camera])

        # 主循环
        try:
            while True:
                sim_world.wait_for_tick()
        except KeyboardInterrupt:
            pass
        
    finally:
        # 确保始终清理资源
        cleanup()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        logging.error(f"程序发生错误: {e}")
        # 即使发生异常也要清理
        cleanup()