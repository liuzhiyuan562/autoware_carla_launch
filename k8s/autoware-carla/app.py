# app.py - 增强版API服务
from flask import Flask, request, jsonify
from kubernetes import client, config
import uuid
import yaml
import os
import time
import threading

app = Flask(__name__)

# 加载 Kubernetes 配置
try:
    config.load_incluster_config()  # 在集群内运行
except:
    config.load_kube_config()  # 本地开发

api = client.CoreV1Api()
api_apps = client.AppsV1Api()

# 存储活跃模拟的状态
active_carla_instances = {}
active_autoware_instances = {}

# 加载模板文件
def load_template(template_name):
    template_path = os.path.join(os.path.dirname(__file__), 'templates', template_name)
    with open(template_path, 'r') as f:
        return f.read()

carla_template = load_template('carla-template.yaml')
autoware_template = load_template('autoware-template.yaml')

def apply_template(template, replacements):
    """应用模板替换"""
    result = template
    for key, value in replacements.items():
        result = result.replace('{{' + key + '}}', str(value))
    return result

def create_from_yaml(yaml_content):
    """从YAML内容创建Kubernetes资源"""
    resources = yaml.safe_load_all(yaml_content)
    results = []
    
    for resource in resources:
        kind = resource.get('kind', '')
        
        if kind == 'Deployment':
            resp = api_apps.create_namespaced_deployment(
                namespace='default',
                body=resource
            )
            results.append(resp)
        elif kind == 'Service':
            resp = api.create_namespaced_service(
                namespace='default',
                body=resource
            )
            results.append(resp)
        elif kind == 'Pod':
            resp = api.create_namespaced_pod(
                namespace='default',
                body=resource
            )
            results.append(resp)
    
    return results

@app.route('/carla', methods=['POST'])
def create_carla():
    """创建新的Carla实例"""
    data = request.json
    carla_id = data.get('carlaId', str(uuid.uuid4()))
    carla_town = data.get('town', 'Town01')
    
    # 检查是否已存在
    if carla_id in active_carla_instances:
        return jsonify({"error": f"Carla instance {carla_id} already exists"}), 400
    
    # 准备替换值
    replacements = {
        'CARLA_ID': carla_id,
        'CARLA_TOWN': carla_town
    }
    
    # 应用模板
    yaml_content = apply_template(carla_template, replacements)
    
    try:
        # 创建资源
        create_from_yaml(yaml_content)
        
        # 记录活跃实例
        active_carla_instances[carla_id] = {
            'created_at': time.time(),
            'town': carla_town,
            'autoware_instances': []
        }
        
        return jsonify({
            "id": carla_id, 
            "status": "creating",
            "town": carla_town
        })
    
    except client.exceptions.ApiException as e:
        return jsonify({"error": str(e)}), 500

@app.route('/carla', methods=['GET'])
def list_carla():
    """列出所有Carla实例"""
    try:
        deployments = api_apps.list_namespaced_deployment(
            namespace='default',
            label_selector='app=carla-simulator'
        )
        
        carla_instances = []
        for deployment in deployments.items:
            carla_id = deployment.metadata.labels.get('carla-id')
            if carla_id:
                status = deployment.status.available_replicas
                carla_instances.append({
                    "id": carla_id,
                    "status": "Running" if status else "Starting",
                    "deployment": deployment.metadata.name,
                    "created_at": deployment.metadata.creation_timestamp.timestamp(),
                    "info": active_carla_instances.get(carla_id, {})
                })
        
        return jsonify({"carla_instances": carla_instances})
    
    except client.exceptions.ApiException as e:
        return jsonify({"error": str(e)}), 500

@app.route('/carla/<carla_id>', methods=['GET'])
def get_carla(carla_id):
    """获取特定Carla实例信息"""
    try:
        deployment = api_apps.read_namespaced_deployment(
            name=f'carla-sim-{carla_id}',
            namespace='default'
        )
        
        # 获取关联的Autoware实例
        associated_autoware = []
        if carla_id in active_carla_instances:
            associated_autoware = active_carla_instances[carla_id].get('autoware_instances', [])
        
        return jsonify({
            "id": carla_id,
            "status": "Running" if deployment.status.available_replicas else "Starting",
            "deployment": deployment.metadata.name,
            "created_at": deployment.metadata.creation_timestamp.timestamp(),
            "autoware_instances": associated_autoware,
            "info": active_carla_instances.get(carla_id, {})
        })
    
    except client.exceptions.ApiException as e:
        if e.status == 404:
            return jsonify({"error": "Carla instance not found"}), 404
        return jsonify({"error": str(e)}), 500

@app.route('/carla/<carla_id>', methods=['DELETE'])
def delete_carla(carla_id):
    """删除Carla实例及其关联的Autoware实例"""
    try:
        # 首先删除关联的Autoware实例
        if carla_id in active_carla_instances:
            for autoware_id in active_carla_instances[carla_id].get('autoware_instances', []):
                try:
                    api.delete_namespaced_pod(
                        name=f'autoware-{autoware_id}',
                        namespace='default'
                    )
                    if autoware_id in active_autoware_instances:
                        del active_autoware_instances[autoware_id]
                except:
                    # 忽略删除错误，继续处理
                    pass
        
        # 删除Deployment
        api_apps.delete_namespaced_deployment(
            name=f'carla-sim-{carla_id}',
            namespace='default'
        )
        
        # 删除Service
        api.delete_namespaced_service(
            name=f'carla-sim-{carla_id}',
            namespace='default'
        )
        
        # 从活跃实例中移除
        if carla_id in active_carla_instances:
            del active_carla_instances[carla_id]
        
        return jsonify({
            "id": carla_id, 
            "status": "deleting",
            "message": "Carla instance and associated Autoware instances are being deleted"
        })
    
    except client.exceptions.ApiException as e:
        if e.status == 404:
            return jsonify({"error": "Carla instance not found"}), 404
        return jsonify({"error": str(e)}), 500

@app.route('/autoware', methods=['POST'])
def create_autoware():
    """创建新的Autoware实例，并连接到指定的Carla实例"""
    data = request.json
    autoware_id = data.get('autowareId', str(uuid.uuid4()))
    carla_id = data.get('carlaId')
    map_name = data.get('mapName', 'default_map')
    vehicle_model = data.get('vehicleModel', 'default_vehicle')
    
    # 验证必要参数
    if not carla_id:
        return jsonify({"error": "carlaId is required"}), 400
    
    # 检查Carla实例是否存在
    try:
        api_apps.read_namespaced_deployment(
            name=f'carla-sim-{carla_id}',
            namespace='default'
        )
    except client.exceptions.ApiException as e:
        if e.status == 404:
            return jsonify({"error": f"Carla instance {carla_id} not found"}), 404
        return jsonify({"error": str(e)}), 500
    
    # 准备替换值
    replacements = {
        'AUTOWARE_ID': autoware_id,
        'CARLA_ID': carla_id,
        'MAP_NAME': map_name,
        'VEHICLE_MODEL': vehicle_model
    }
    
    # 应用模板
    yaml_content = apply_template(autoware_template, replacements)
    
    try:
        # 创建资源
        create_from_yaml(yaml_content)
        
        # 记录活跃实例
        active_autoware_instances[autoware_id] = {
            'created_at': time.time(),
            'carla_id': carla_id,
            'map': map_name,
            'vehicle': vehicle_model
        }
        
        # 更新Carla实例关联
        if carla_id in active_carla_instances:
            if 'autoware_instances' not in active_carla_instances[carla_id]:
                active_carla_instances[carla_id]['autoware_instances'] = []
            active_carla_instances[carla_id]['autoware_instances'].append(autoware_id)
        
        return jsonify({
            "id": autoware_id, 
            "status": "creating",
            "carla_id": carla_id,
            "map": map_name,
            "vehicle": vehicle_model
        })
    
    except client.exceptions.ApiException as e:
        return jsonify({"error": str(e)}), 500

@app.route('/autoware', methods=['GET'])
def list_autoware():
    """列出所有Autoware实例"""
    try:
        pods = api.list_namespaced_pod(
            namespace='default',
            label_selector='app=autoware'
        )
        
        autoware_instances = []
        for pod in pods.items:
            autoware_id = pod.metadata.labels.get('autoware-id')
            carla_id = pod.metadata.labels.get('carla-id')
            if autoware_id:
                status = pod.status.phase
                autoware_instances.append({
                    "id": autoware_id,
                    "status": status,
                    "carla_id": carla_id,
                    "pod_name": pod.metadata.name,
                    "created_at": pod.metadata.creation_timestamp.timestamp(),
                    "info": active_autoware_instances.get(autoware_id, {})
                })
        
        return jsonify({"autoware_instances": autoware_instances})
    
    except client.exceptions.ApiException as e:
        return jsonify({"error": str(e)}), 500

@app.route('/autoware/<autoware_id>', methods=['DELETE'])
def delete_autoware(autoware_id):
    """删除特定的Autoware实例"""
    try:
        # 获取Autoware的Carla关联
        carla_id = None
        if autoware_id in active_autoware_instances:
            carla_id = active_autoware_instances[autoware_id].get('carla_id')
        
        # 删除Pod
        api.delete_namespaced_pod(
            name=f'autoware-{autoware_id}',
            namespace='default'
        )
        
        # 更新关联记录
        if carla_id and carla_id in active_carla_instances:
            if 'autoware_instances' in active_carla_instances[carla_id]:
                if autoware_id in active_carla_instances[carla_id]['autoware_instances']:
                    active_carla_instances[carla_id]['autoware_instances'].remove(autoware_id)
        
        # 从活跃实例中移除
        if autoware_id in active_autoware_instances:
            del active_autoware_instances[autoware_id]
        
        return jsonify({
            "id": autoware_id, 
            "status": "deleting"
        })
    
    except client.exceptions.ApiException as e:
        if e.status == 404:
            return jsonify({"error": "Autoware instance not found"}), 404
        return jsonify({"error": str(e)}), 500

@app.route('/simulations', methods=['GET'])
def list_simulations():
    """列出所有模拟（Carla+Autoware组合）"""
    try:
        # 获取所有Carla实例
        carla_instances = []
        for carla_id, info in active_carla_instances.items():
            # 对每个Carla，获取其关联的Autoware实例
            autoware_instances = []
            for autoware_id in info.get('autoware_instances', []):
                if autoware_id in active_autoware_instances:
                    autoware_instances.append({
                        "id": autoware_id,
                        "info": active_autoware_instances[autoware_id]
                    })
            
            carla_instances.append({
                "carla_id": carla_id,
                "info": info,
                "autoware_instances": autoware_instances
            })
        
        return jsonify({"simulations": carla_instances})
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)