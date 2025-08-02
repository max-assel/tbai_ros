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

```
