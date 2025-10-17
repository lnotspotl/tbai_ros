# Towards Better Athletic Intelligence - ROS Noetic

## Implemented controllers

```
📦tbai
 ┣ 📂tbai_ros_static               # Static (high gain PD) controller
 ┣ 📂tbai_ros_mpc                  # NMPC controller (both perceptive and blind versions) [1]
 ┣ 📂tbai_ros_bob                  # RL walking controller, based on the wild-Anymal paper (both perceptive and blind versions) [2]
 ┣ 📂tbai_ros_dtc                  # DTC controller (perceptive) [3]
 ┣ 📂tbai_ros_joe                  # Perceptive NMPC controller with NN-based tracking controller [1], [3]

 [1] Perceptive Locomotion through Nonlinear Model Predictive Control
     https://arxiv.org/abs/2208.08373
 [2] Learning robust perceptive locomotion for quadrupedal robots in the wild
     https://arxiv.org/abs/2201.08117
 [3] DTC: Deep Tracking Control
     https://arxiv.org/abs/2309.15462
```

## Installing tbai

To install `tbai_ros`, we recommend using `pixi`, though `tbai_ros` is a full-fledged ROS package and it can be integrated into your projects in using conventional tools and methods. We use `pixi` for reproducibility. Don't worry that ROS is past its end of life, pixi (or micromamba) will install everything for you (even on the newest Ubuntu release) 😮

### Alternative 1: pixi
```bash
# Install pixi
curl -fsSL https://pixi.sh/install.sh | sh # You might have to source your config again

# Install tbai_ros
mkdir -p ros/src && cd ros/src
git clone https://github.com/lnotspotl/tbai_ros.git --recursive && cd tbai_ros
pixi install && pixi shell --environment all-gpu-free
just fresh-install-all-gpu-free
```

### Alternative 2: micromamba
```bash
# Install micromamba
"${SHELL}" <(curl -L micro.mamba.pm/install.sh) # You might have to source your config again

# Clone tbai_ros
mkdir -p ros/src && cd ros/src
git clone https://github.com/lnotspotl/tbai_ros.git --recursive && cd tbai_ros

# Create conda environment
micromamba env create -f .conda/all-gpu-free.yaml
micromamba activate all-gpu-free

# Install tbai_ros
just fresh-install-all-gpu-free
```

Once the installation is complete, you can run one of our many examples, for instance:

```bash
# Activate pixi environment
pixi shell --environment all-gpu-free

# Run NP3O example
source $(catkin locate)/devel/setup.bash && roslaunch tbai_ros_np3o simple_go2.launch gui:=true

# Try out other examples located under tbai_ros_mpc, tbai_ros_bob, tbai_ros_dtc, tbai_ros_joe and tbai_ros_np3o
```

### Go2 deployment

Check out the [**tbai_ros_deploy_go2_rl**](./tbai_ros_deploy_go2/tbai_ros_deploy_go2_rl) folder for deployment-related documentation, pictures and videos 🤗


### Perceptive MPC



https://github.com/lnotspotl/tbai_ros/assets/82883398/f451c12d-7525-4606-b722-726f63d852ca




### Blind MPC



[mpc_go2_blind.webm](https://github.com/user-attachments/assets/28f11d25-4ba0-4b7a-ae3d-e8b1c1a60de6)




### Perceptive Bob



https://github.com/lnotspotl/tbai_ros/assets/82883398/7f6bdefa-4299-454b-a0ef-55e463e0c88d




### Blind Bob


https://github.com/lnotspotl/tbai_ros/assets/82883398/ebc2d90d-5c03-4207-a868-2e9436c140d4



### DTC: Deep Tracking Control


https://github.com/lnotspotl/tbai_ros/assets/82883398/6cf672db-b737-4724-a6da-afa0c8dd19d5


### Joe


https://github.com/lnotspotl/tbai_ros/assets/82883398/e3455dd3-10e8-41da-bb02-87fbdf3de041


### System architecture

![overview_01](https://github.com/lnotspotl/tbai_ros/assets/82883398/2c17f08d-6994-4982-8739-2b8246dfcb32)

## Controller architectures

### Mpc 
![mpc_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/daabb2c2-8ced-4ffd-956e-35279b78563b)


### Bob

![bob_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/3ea71f1c-b58c-4028-93d3-971592aa364d)


### DTC: Deep Tracking Control

![dtc_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/10b3481d-7782-4a0e-ac31-24e2786c3402)

### Joe

![joe_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/0139df20-d2ce-4de1-884f-ce37e770ee08)


### Credits
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
- https://github.com/RossHartley/invariant-ekf
- https://github.com/dfki-ric-underactuated-lab/dfki-quad
- https://github.com/iit-DLSLab/muse
- https://github.com/zeonsunlightyu/LocomotionWithNP3O
- http://www.michaelsebek.cz/cs
- hundreds of others ...

Thank you all 🤗
