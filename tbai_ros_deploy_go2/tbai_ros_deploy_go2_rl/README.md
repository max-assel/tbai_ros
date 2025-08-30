## Go2 deployment

<img width="2859" height="1701" alt="image" src="https://github.com/user-attachments/assets/e6f5414e-9087-48f7-adfd-0ae20dad446a" />

```bash
## Build project
mkdir -p ros/src && cd ros/src && git clone git@github.com:lnotspotl/tbai_ros.git --recursive && cd tbai_ros && ./tbai_ros.bash --fresh_install go2
pixi shell --environment go2
source $(catkin locate)/devel/setup.bash

## Deploy np3o policy, with mapping running on gpu
roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o.launch publish_pointcloud:=true mapping_device:=gpu

## Deploy np3o policy, with mapping running on cpu
roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o.launch publish_pointcloud:=true mapping_device:=gpu

## Deploy np3o policy, with no mapping
roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o.launch publish_pointcloud:=false mapping_device:=none

## Turn lidar ON or OFF (ROS2 needs to be sourced)
ros2 topic pub /utlidar/switch std_msgs/msg/String "data: 'ON'" --once
ros2 topic pub /utlidar/switch std_msgs/msg/String "data: 'OFF'" --once
```

---

### Showcase

#### Go2 inside
[inside.webm](https://github.com/user-attachments/assets/b9745931-9766-4b03-9376-f104ce9f3f54)

#### Go2 outside
https://github.com/user-attachments/assets/6a4c4f4d-dd03-4de0-9b1f-5339305379f0

---

#### Go2 State Estimation

[exp1.webm](https://github.com/user-attachments/assets/23d6d89a-2e1f-422d-bcd9-e3535efcabc2)

#### Go2 Climbing stairs

https://github.com/user-attachments/assets/11b9b8b0-2cab-49ce-9226-5dde3d2ac029



https://github.com/user-attachments/assets/3ea3c1dd-b1fc-445b-8d06-3ed56743738b

