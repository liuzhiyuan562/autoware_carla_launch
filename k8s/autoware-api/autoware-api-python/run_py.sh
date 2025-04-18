#!/bin/bash
# This script is used to run the autoware-api-python container
# . /home/zy/autoware_carla_launch/k8s/autoware-api/autoware-api-python/run_py.sh

# Set up the ROS2 environment
source ~/autoware_carla_launch/env.sh

# Set up ROS2 autoware msgs environment
source ~/autoware_carla_launch/k8s/autoware-api/autoware_adapi_msgs/install/setup.bash

# Run python script
# /bin/python3 ~/autoware_carla_launch/k8s/autoware-api/autoware-api-python/test_msg.py
# /bin/python3 ~/autoware_carla_launch/k8s/autoware-api/autoware-api-python/test_msg.py --topic "/api/system/heartbeat" --type "heartbeat" --comm "realtime_stream"
# /bin/python3 ~/autoware_carla_launch/k8s/autoware-api/autoware-api-python/test_msg.py --topic "/vehicle/status/control_mode"


/bin/python3 ~/autoware_carla_launch/k8s/autoware-api/autoware-api-python/test_zenoh.py