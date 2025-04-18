#!/usr/bin/env python3
"""
Python替代版本的CARLA桥接启动脚本
替代原始Bash脚本的功能，启动zenoh_carla_bridge和change_map.py
"""

import os
import sys
import time
import logging
import subprocess
import datetime
import signal
from pathlib import Path
import threading

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# 全局变量以跟踪子进程
bridge_process = None
python_process = None
processes = []
stop_event = threading.Event()

def setup_environment():
    """设置环境变量和路径"""
    # 获取环境变量，如果不存在则使用默认值
    autoware_carla_root = os.environ.get('AUTOWARE_CARLA_ROOT', '')
    if not autoware_carla_root:
        logging.error("AUTOWARE_CARLA_ROOT环境变量未设置")
        sys.exit(1)
    
    # 设置Python代理路径
    python_agent_path = os.path.join(autoware_carla_root, 'external/zenoh_carla_bridge/carla_agent')
    
    # 获取当前日期时间，创建日志文件夹
    current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
    log_path = os.path.join('bridge_log', current_time)
    os.makedirs(log_path, exist_ok=True)
    
    # 获取其他环境变量
    carla_simulator_ip = os.environ.get('CARLA_SIMULATOR_IP', '127.0.0.1:2000')
    zenoh_bridge_config = os.environ.get('ZENOH_CARLA_BRIDGE_CONFIG', '')
    
    return {
        'autoware_carla_root': autoware_carla_root,
        'python_agent_path': python_agent_path,
        'log_path': log_path,
        'carla_simulator_ip': carla_simulator_ip,
        'zenoh_bridge_config': zenoh_bridge_config
    }

def start_bridge(config):
    """启动zenoh_carla_bridge进程"""
    global bridge_process
    
    # 创建日志文件路径
    bridge_log_file = os.path.join(config['log_path'], 'bridge.log')
    
    # 设置环境变量
    env = os.environ.copy()
    env['RUST_LOG'] = 'z=info'
    
    # 构建命令
    bridge_cmd = [
        f"{config['autoware_carla_root']}/external/zenoh_carla_bridge/target/release/zenoh_carla_bridge",
        "--mode", "ros2",
        "--zenoh-listen", "tcp/0.0.0.0:7447",
        "--zenoh-config", config['zenoh_bridge_config'],
        "--carla-address", config['carla_simulator_ip']
    ]
    
    # 打开日志文件
    with open(bridge_log_file, 'w') as log_file:
        logging.info(f"正在启动zenoh_carla_bridge: {' '.join(bridge_cmd)}")
        
        # 启动进程
        bridge_process = subprocess.Popen(
            bridge_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
            bufsize=1  # 行缓冲
        )
        
        processes.append(bridge_process)
        
        # 启动日志记录线程
        def log_output():
            while not stop_event.is_set() and bridge_process.poll() is None:
                line = bridge_process.stdout.readline()
                if not line:
                    break
                print(line.rstrip())
                log_file.write(line)
                log_file.flush()
        
        log_thread = threading.Thread(target=log_output)
        log_thread.daemon = True
        log_thread.start()
        
        logging.info(f"zenoh_carla_bridge已启动，PID: {bridge_process.pid}")
        
        return bridge_process

def start_python_script(config):
    """启动change_map.py Python脚本"""
    global python_process
    
    # 创建日志文件路径
    python_log_file = os.path.join(config['log_path'], 'vehicle.log')
    
    # 构建命令
    python_cmd = [
        "poetry", "-C", config['python_agent_path'],
        "run", "python3", f"{config['python_agent_path']}/change_map.py"
    ]
    
    # 打开日志文件
    with open(python_log_file, 'w') as log_file:
        logging.info(f"正在启动Python脚本: {' '.join(python_cmd)}")
        
        # 启动进程
        python_process = subprocess.Popen(
            python_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1  # 行缓冲
        )
        
        processes.append(python_process)
        
        # 启动日志记录线程
        def log_output():
            while not stop_event.is_set() and python_process.poll() is None:
                line = python_process.stdout.readline()
                if not line:
                    break
                print(line.rstrip())
                log_file.write(line)
                log_file.flush()
        
        log_thread = threading.Thread(target=log_output)
        log_thread.daemon = True
        log_thread.start()
        
        logging.info(f"Python脚本已启动，PID: {python_process.pid}")
        
        return python_process

def cleanup():
    """清理所有子进程"""
    stop_event.set()
    
    logging.info("开始清理子进程...")
    
    for proc in processes:
        if proc and proc.poll() is None:
            try:
                logging.info(f"正在终止进程 PID: {proc.pid}")
                proc.terminate()
                
                # 等待进程优雅终止
                for _ in range(5):
                    if proc.poll() is not None:
                        break
                    time.sleep(0.5)
                
                # 如果进程仍在运行，强制终止
                if proc.poll() is None:
                    logging.warning(f"进程 {proc.pid} 未响应，强制终止")
                    proc.kill()
            except Exception as e:
                logging.error(f"终止进程时出错: {e}")
    
    logging.info("清理完成")

def signal_handler(sig, frame):
    """处理信号以优雅地终止进程"""
    logging.info(f"收到信号 {sig}，开始清理...")
    cleanup()
    sys.exit(0)

def main():
    """主函数"""
    try:
        # 注册信号处理
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # 设置环境和获取配置
        config = setup_environment()
        logging.info(f"日志将保存到: {config['log_path']}")
        
        # 启动Python脚本
        python_proc = start_python_script(config)
        
        # 等待几秒后启动桥接
        logging.info("等待5秒后启动zenoh_carla_bridge...")
        time.sleep(5)
        bridge_proc = start_bridge(config)
        
        # 等待所有进程完成
        while True:
            if python_proc.poll() is not None:
                logging.warning(f"Python脚本已退出，退出码: {python_proc.returncode}")
                break
            
            if bridge_proc.poll() is not None:
                logging.warning(f"桥接进程已退出，退出码: {bridge_proc.returncode}")
                break
            
            time.sleep(1)
        
    except Exception as e:
        logging.error(f"运行时出错: {e}")
        return 1
    finally:
        # 确保资源被清理
        cleanup()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())