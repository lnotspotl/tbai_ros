justfile_path := `realpath justfile`
tbai_build_dir := "/tmp/tbai_build_123"
unitree_mujoco_build_dir := "/tmp/unitree_mujoco_build_123"
current_dir := `realpath justfile`

# Show available commands
help:
    #!/usr/bin/env bash
    just -l

# Fresh install go2 environment
[group("1. fresh")]
fresh-install-go2: clean clone-tbai build-tbai ros-build-go2 install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping elevation_mapping_cupy hesai_ros_driver realsense2_camera
    echo "All good 🤗"

# Fresh install go2-gpu-free environment
[group("1. fresh")]
fresh-install-go2-gpu-free: clean clone-tbai build-tbai ros-build-go2 install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping realsense2_camera hesai_ros_driver
    echo "All good 🤗"

# Fresh install all environment
[group("1. fresh")]
fresh-install-all: clean clone-tbai build-tbai ros-build-all install-tbai-cbf-mppi clone-mujoco-robotic-assets build-mujoco-robotic-assets clone-unitree-mujoco build-unitree-mujoco
    #!/usr/bin/env bash
    catkin build elevation_mapping elevation_mapping_cupy
    echo "All good 🤗"

# Fresh install all-gpu-free environment
[group("1. fresh")]
fresh-install-all-gpu-free: clean clone-tbai build-tbai ros-build-all install-tbai-cbf-mppi clone-mujoco-robotic-assets build-mujoco-robotic-assets clone-unitree-mujoco build-unitree-mujoco
    #!/usr/bin/env bash
    catkin build elevation_mapping
    echo "All good 🤗"

#******************************************************************************************************************#
#******************************************************************************************************************#
#******************************************************************************************************************#

# Go2 blind MPC in Gazebo
[group("2. demos")]
go2_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc go2_blind.launch gui:=true

# Go2 perceptive MPC in Gazebo
[group("2. demos")]
go2_perceptive_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc go2_perceptive.launch gui:=true

# ANYmal B blind MPC in Gazebo
[group("2. demos")]
anymal_b_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc anymal_b_blind.launch gui:=true

# ANYmal C blind MPC in Gazebo
[group("2. demos")]
anymal_c_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc anymal_c_blind.launch gui:=true

# ANYmal D blind MPC in Gazebo
[group("2. demos")]
anymal_d_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc anymal_d_blind.launch gui:=true

# ANYmal D perceptive MPC in Gazebo
[group("2. demos")]
anymal_d_perceptive_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc anymal_d_perceptive.launch gui:=true

# Spot blind MPC in Gazebo
[group("2. demos")]
spot_mpc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc spot_blind.launch gui:=true

# Go2 MPC in Mujoco
[group("2. demos")]
go2_mpc_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_mpc deploy_go2_mpc.launch run_go2_joystick:=false run_virtual_joystick:=true unitree_channel:=1 network_interface:=lo mujoco_simulation:=true run_rviz:=true

# ANYmal D perceptive DTC in Gazebo
[group("2. demos")]
anymal_d_dtc_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_dtc simple_perceptive.launch gui:=true

# ANYmal D blind DTC in Gazebo
[group("2. demos")]
anymal_d_dtc_blind_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_dtc simple_blind.launch gui:=true

# Go2 NP3O in Gazebo
[group("2. demos")]
go2_np3o_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_np3o simple_go2.launch gui:=true

# Go2 NP3O in Mujoco
[group("2. demos")]
go2_np3o_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o.launch run_go2_joystick:=false run_virtual_joystick:=true unitree_channel:=1 network_interface:=lo mujoco_simulation:=true run_rviz:=true

# Go2 NP3O with MPPI in Mujoco
[group("2. demos")]
go2_np3o_mppi_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o_mppi.launch run_go2_joystick:=false run_virtual_joystick:=true network_interface:=lo mujoco_simulation:=true run_rviz:=true

# Go2 Safe in Gazebo
[group("2. demos")]
go2_safe_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_safe simple_go2_safe.launch gui:=true

# Go2 Safe in Mujoco
[group("2. demos")]
go2_safe_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_safe deploy_go2_safe.launch run_go2_joystick:=false run_virtual_joystick:=true unitree_channel:=1 network_interface:=lo mujoco_simulation:=true run_rviz:=true

# ANYmal D blind Bob in Gazebo
[group("2. demos")]
anymal_d_bob_blind_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_bob anymal_d_blind.launch gui:=true

# ANYmal D perceptive Bob in Gazebo
[group("2. demos")]
anymal_d_bob_perceptive_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_bob anymal_d_perceptive.launch gui:=true

# ANYmal D mapping Bob in Gazebo
[group("2. demos")]
anymal_d_bob_mapping_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_bob anymal_d_mapping.launch gui:=true

# Go2 blind Bob in Gazebo
[group("2. demos")]
go2_bob_blind_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_bob go2_blind.launch gui:=true

# G1 humanoid in MuJoCo
[group("2. demos")]
g1_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_g1 g1_mujoco.launch mujoco_simulation:=true run_rviz:=false

# Go2W (wheeled) drive in MuJoCo
[group("2. demos")]
go2w_drive_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_go2w go2w_mujoco_drive.launch run_rviz:=false

# Go2W (wheeled) handstand in MuJoCo
[group("2. demos")]
go2w_handstand_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_go2w go2w_mujoco_handstand.launch run_rviz:=false

# Go2W (wheeled) drive in MuJoCo
[group("2. demos")]
go2w_drive_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    echo "This example requires two terminals:"
    echo "roslaunch tbai_ros_gazebo_unitree go2w_gazebo_unitree.launch"
    echo "roslaunch tbai_ros_go2w go2w_mujoco_drive.launch mujoco_simulation:=false"
    echo "We are working on a fix! 🤗"
    
# Go2W (wheeled) handstand in MuJoCo
[group("2. demos")]
go2w_handstand_gazebo:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    echo "This example requires two terminals:"
    echo "roslaunch tbai_ros_gazebo_unitree go2w_gazebo_unitree.launch"
    echo "roslaunch tbai_ros_go2w go2w_mujoco_handstand.launch mujoco_simulation:=false"
    echo "We are working on a fix! 🤗"

#******************************************************************************************************************#
#******************************************************************************************************************#
#******************************************************************************************************************#

# Deploy go2 np3o policy
[group("3. deployment")]
deploy-go2-np3o:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o.launch publish_pointcloud:=true mapping_device:=cpu

# Deploy go2 np3o policy with an MPPI safety controller
[group("3. deployment")]
deploy-go2-np3o-mppi:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o_mppi.launch publish_pointcloud:=true mapping_device:=cpu

# Deploy go2 np3o policy with a safe controller
[group("3. deployment")]
deploy-go2-np3o-safe:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_rl deploy_go2_np3o_safe.launch publish_pointcloud:=true mapping_device:=cpu

# Deploy go2 MPC policy
[group("3. deployment")]
deploy-go2-mpc:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_deploy_go2_rl deploy_go2_mpc.launch publish_pointcloud:=false mapping_device:=none

#******************************************************************************************************************#
#******************************************************************************************************************#
#******************************************************************************************************************#

# Format C++ code using clang-format, disabled for 'dependencies' folder
[group("4. development")]
format:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v dependencies)
    for folder in $folders; do
        for file in $(find $folder -name "*.hpp" -o -name "*.cpp"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done

# Run cpplint on all folders, disabled for 'dependencies' folder
[group("4. development")]
lint:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v dependencies)
    for folder in $folders; do
        cpplint --recursive $folder
    done

# Build ROS packages (only those related to tbai_ros)
[group("4. development")]
build:
    #!/usr/bin/env bash
    ros_packages=""
    search_dirs=(./ tbai_ros_deploy_go2/)
    for search_dir in ${search_dirs[@]}; do
        for folder in "$search_dir"*/; do
            folder=${folder%/}
            if [[ -f "$folder/CMakeLists.txt" ]] && [[ -f "$folder/package.xml" ]]; then
                package=$(basename "$folder")
                if [[ "$package" != "tbai_ros_dtc" && "$package" != "tbai_ros_joe" && "$package" != tbai_ros_deploy_go2* && "$package" != "tbai_ros_context_aware" ]]; then
                    ros_packages+=" $package"
                fi
            fi
        done
    done
    echo "[TBAI] Building ROS packages:$ros_packages"
    catkin build $ros_packages

# Run tests for ROS packages
[group("4. development")]
test:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v dependencies)
    ros_packages=""
    for folder in $folders; do
        if [[ -d $folder/test ]]; then
            echo "[TBAI] Running tests in $folder"
            package=$(basename $folder)
            ros_packages+=" $package"
        fi
    done
    echo "[TBAI] Running tests for ROS packages:$ros_packages"
    catkin test $ros_packages

# List all pixi environments\
[group("4. development")]
pixi-list-envs:
    #!/usr/bin/env bash
    pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //'

# Generate conda environments for all pixi environments
[group("4. development")]
pixi-generate-conda-envs:
    #!/usr/bin/env bash
    set -euo pipefail
    all_envs=$(pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //')
    for env in $all_envs; do
        pixi workspace export conda-environment -e $env > .conda/$env.yaml
    done

# Open documentation in browser
[group("4. development")]
open-docs:
    #!/usr/bin/env bash
    docs_path={{justfile_path}}/../../build/tbai_ros_docs/output/doxygen/html/index.html
    echo "Opening documentation in browser: $docs_path"
    google-chrome $docs_path

# Rebuild documentation
[group("4. development")]
rebuild-docs:
    #!/usr/bin/env bash
    catkin clean tbai_ros_docs
    catkin build tbai_ros_docs

# Clean ROS workspace and remove tbai
[group("4. development")]
clean:
    #!/usr/bin/env bash
    CURRENT_DIR=$(pwd)
    cd ../..
    catkin init
    catkin config --cmake-args -Wno-dev -DCMAKE_BUILD_TYPE=Release
    echo "Cleaning ROS workspace: $CURRENT_DIR"
    cd $CURRENT_DIR
    catkin clean -y
    rm -rf ${tbai_build_dir}

# Clone tbai repository (skips if already exists)
[group("4. development")]
clone-tbai:
    #!/usr/bin/env bash
    if [[ ! -d dependencies/tbai ]]; then
        git clone https://github.com/lnotspotl/tbai.git --single-branch --branch=lnotspotl/tbai_refactor dependencies/tbai
    else
        echo "[TBAI] dependencies/tbai already exists, skipping clone"
    fi

# Build tbai library
[group("4. development")]
build-tbai:
    #!/usr/bin/env bash
    cmake -B{{tbai_build_dir}} -Sdependencies/tbai -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
    cmake --build {{tbai_build_dir}} --parallel 8
    cmake --build {{tbai_build_dir}} --target install

# Remove tbai dependencies
[group("4. development")]
remove-tbai:
    #!/usr/bin/env bash
    rm -rf dependencies/tbai && rm -rf ${tbai_build_dir}

[group("4. development")]
clone-mujoco-robotic-assets:
    #!/usr/bin/env bash
    if [[ ! -d dependencies/mujoco_robotic_assets ]]; then
        git clone https://github.com/lnotspotl/mujoco_robotic_assets.git --single-branch --branch=main dependencies/mujoco_robotic_assets
    else
        echo "[TBAI] dependencies/mujoco_robotic_assets already exists, skipping clone"
    fi

[group("4. development")]
build-mujoco-robotic-assets:
    #!/usr/bin/env bash
    catkin build mujoco_robotic_assets

[group("4. development")]
remove-mujoco-robotic-assets:
    #!/usr/bin/env bash
    rm -rf dependencies/mujoco_robotic_assets

[group("4. development")]
clone-unitree-mujoco:
    #!/usr/bin/env bash
    if [[ ! -d dependencies/unitree_mujoco ]]; then
        git clone https://github.com/lnotspotl/unitree_mujoco.git --single-branch --branch=main dependencies/unitree_mujoco
    else
        echo "[TBAI] dependencies/unitree_mujoco already exists, skipping clone"
    fi

[group("4. development")]
build-unitree-mujoco:
    #!/usr/bin/env bash
    cmake -B{{unitree_mujoco_build_dir}} -Sdependencies/unitree_mujoco -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
    cmake --build {{unitree_mujoco_build_dir}} --parallel 8
    cmake --build {{unitree_mujoco_build_dir}} --target install

[group("4. development")]
remove-unitree-mujoco:
    #!/usr/bin/env bash
    rm -rf dependencies/unitree_mujoco
    rm -rf {{unitree_mujoco_build_dir}}

[group("4. development")]
install-tbai-cbf-mppi: clone-tbai
    #!/usr/bin/env bash
    pip3 install git+https://github.com/lnotspotl/tbai_cbf_mppi.git

# Build all ROS packages
[group("4. development")]
ros-build-all: build
    #!/usr/bin/env bash
    echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(catkin locate)/devel/lib:$CONDA_PREFIX/lib' >> ../../devel/setup.sh

# Build go2 ROS packages
[group("4. development")]
ros-build-go2: 
    #!/usr/bin/env bash
    catkin build tbai_ros_deploy_go2_rl

