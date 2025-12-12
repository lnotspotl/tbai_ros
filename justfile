justfile_path := `realpath justfile`
tbai_build_dir := "/tmp/tbai_build_123"
current_dir := `realpath justfile`

# Show available commands
help:
    #!/usr/bin/env bash
    just -l

# Format C++ code using clang-format, disabled for 'dependencies' folder
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
lint:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v dependencies)
    for folder in $folders; do
        cpplint --recursive $folder
    done

# Build ROS packages (only those related to tbai_ros)
build:
    #!/usr/bin/env bash
    ros_packages=""
    search_dirs=(./ tbai_ros_deploy_go2/)
    for search_dir in ${search_dirs[@]}; do
        for folder in "$search_dir"*/; do
            folder=${folder%/}
            if [[ -f "$folder/CMakeLists.txt" ]] && [[ -f "$folder/package.xml" ]]; then
                package=$(basename "$folder")
                ros_packages+=" $package"
            fi
        done
    done
    echo "[TBAI] Building ROS packages:$ros_packages"
    catkin build $ros_packages

# Run tests for ROS packages
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

# List all pixi environments
pixi-list-envs:
    #!/usr/bin/env bash
    pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //'

# Generate conda environments for all pixi environments
pixi-generate-conda-envs:
    #!/usr/bin/env bash
    set -euo pipefail
    all_envs=$(pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //')
    for env in $all_envs; do
        pixi workspace export conda-environment -e $env > .conda/$env.yaml
    done

# Open documentation in browser
open-docs:
    #!/usr/bin/env bash
    docs_path={{justfile_path}}/../../build/tbai_ros_docs/output/doxygen/html/index.html
    echo "Opening documentation in browser: $docs_path"
    google-chrome $docs_path

# Rebuild documentation
rebuild-docs:
    #!/usr/bin/env bash
    catkin clean tbai_ros_docs
    catkin build tbai_ros_docs

# Clean ROS workspace and remove tbai
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

# Remove tbai dependencies
remove-tbai:
    #!/usr/bin/env bash
    rm -rf dependencies/tbai && rm -rf ${tbai_build_dir}

# Clone tbai repository (skips if already exists)
clone-tbai:
    #!/usr/bin/env bash
    if [[ ! -d dependencies/tbai ]]; then
        git clone https://github.com/lnotspotl/tbai.git --single-branch --branch=develop dependencies/tbai
    else
        echo "[TBAI] dependencies/tbai already exists, skipping clone"
    fi

# Build tbai library
build-tbai:
    #!/usr/bin/env bash
    cmake -B{{tbai_build_dir}} -Sdependencies/tbai -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
    cmake --build {{tbai_build_dir}} --parallel 8
    cmake --build {{tbai_build_dir}} --target install

install-tbai-cbf-mppi: clone-tbai
    #!/usr/bin/env bash
    pip3 install git+https://github.com/lnotspotl/tbai_cbf_mppi.git

# Build all ROS packages
ros-build-all: build
    #!/usr/bin/env bash
    echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(catkin locate)/devel/lib:$CONDA_PREFIX/lib' >> ../../devel/setup.sh

# Build go2 ROS packages
ros-build-go2: 
    #!/usr/bin/env bash
    catkin build tbai_ros_deploy_go2_rl

# Fresh install go2 environment
fresh-install-go2: clean clone-tbai build-tbai ros-build-go2 install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping elevation_mapping_cupy hesai_ros_driver realsense2_camera
    echo "All good 🤗"

# Fresh install go2-gpu-free environment
fresh-install-go2-gpu-free: clean clone-tbai build-tbai ros-build-go2 install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping realsense2_camera hesai_ros_driver
    echo "All good 🤗"

# Fresh install all environment
fresh-install-all: clean clone-tbai build-tbai ros-build-all install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping elevation_mapping_cupy
    echo "All good 🤗"

# Fresh install all-gpu-free environment
fresh-install-all-gpu-free: clean clone-tbai build-tbai ros-build-all install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping
    echo "All good 🤗"
