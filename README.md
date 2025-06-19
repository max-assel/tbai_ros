# tbai

## Implemented controllers

```
📦tbai
 ┣ 📂tbai_ros_static               # Static (high gain PD) controller
 ┣ 📂tbai_ros_mpc_perceptive       # Perceptive NMPC controller [1]
 ┣ 📂tbai_ros_mpc_blind            # Blind NMPC controller [1]
 ┣ 📂tbai_ros_rl_perceptive        # Perceptive RL controller [2]
 ┣ 📂tbai_ros_rl_blind             # Blind RL controller [2]
 ┣ 📂tbai_ros_dtc                  # DTC controller (perceptive) [3]
 ┣ 📂tbai_ros_joe                  # Perceptive NMPC controller with NN-based tracking controller [1],[3]

 [1] Perceptive Locomotion through Nonlinear Model Predictive Control
     https://arxiv.org/abs/2208.08373
 [2] Learning robust perceptive locomotion for quadrupedal robots in the wild
     https://arxiv.org/abs/2201.08117
 [3] DTC: Deep Tracking Control
     https://arxiv.org/abs/2309.15462
```

## Perceptive MPC



https://github.com/lnotspotl/tbai/assets/82883398/f451c12d-7525-4606-b722-726f63d852ca




## Blind MPC



https://github.com/lnotspotl/tbai/assets/82883398/1bf86da1-a3d4-44db-88c4-877ec78b06cc




## Perceptive RL



https://github.com/lnotspotl/tbai/assets/82883398/7f6bdefa-4299-454b-a0ef-55e463e0c88d




## Blind RL


https://github.com/lnotspotl/tbai/assets/82883398/ebc2d90d-5c03-4207-a868-2e9436c140d4



## DTC: Deep Tracking Control


https://github.com/lnotspotl/tbai/assets/82883398/6cf672db-b737-4724-a6da-afa0c8dd19d5


## Joe


https://github.com/lnotspotl/tbai/assets/82883398/e3455dd3-10e8-41da-bb02-87fbdf3de041


## System architecture

![overview_01](https://github.com/lnotspotl/tbai/assets/82883398/2c17f08d-6994-4982-8739-2b8246dfcb32)

## Controller architectures

## Mpc 
![mpc_03](https://github.com/lnotspotl/tbai/assets/82883398/daabb2c2-8ced-4ffd-956e-35279b78563b)


## Rl (Bob)

![bob_03](https://github.com/lnotspotl/tbai/assets/82883398/3ea71f1c-b58c-4028-93d3-971592aa364d)


## DTC: Deep Tracking Control

![dtc_03](https://github.com/lnotspotl/tbai/assets/82883398/10b3481d-7782-4a0e-ac31-24e2786c3402)

## Joe

![joe_03](https://github.com/lnotspotl/tbai/assets/82883398/0139df20-d2ce-4de1-884f-ce37e770ee08)

## Installing tbai
```bash
# Install dependencies
sudo apt install libmpfr-dev

# Download project
mkdir -p <your-file>/src && cd <your-file> && catkin init && cd src
git clone git@github.com:lnotspotl/tbai.git --recursive

# Install other dependencies using rosdep
cd .. && rosdep install --from-paths src --ignore-src -r -y && cd src/tbai

# Build tbai
catkin config -DCMAKE_BUILD_TYPE=Release
bash ./tbai.bash --build  # This will only build the necessary packages

# Source tbai
cd ../.. && source devel/setup.bash
```
If any of the steps throws an error for you, please let use know and we will try to extend this guideline with a fix as soon as possible. Thanks 🤗

## Credits
This project stands on the shoulders of giants.
None of this would have been possible were it not for many amazing open-source projects.
Here are a couple that most inspiration was drawn from and that were instrumental during the development:

- https://github.com/leggedrobotics/ocs2
- https://github.com/qiayuanl/legged_control
- https://github.com/leggedrobotics/legged_gym
- https://github.com/leggedrobotics/rsl_rl
- https://github.com/ANYbotics/elevation_mapping
- https://github.com/leggedrobotics/elevation_mapping_cupy
- https://github.com/bernhardpg/quadruped_locomotion
- https://github.com/stack-of-tasks/pinocchio
- https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- https://github.com/mayataka/robotoc
- https://github.com/mayataka/legged_state_estimator
- hundreds of others ...

Thank you all 🤗
