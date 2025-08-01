#!/usr/bin/env bash

function print_help() {
	echo "Usage: ./tbai_ros.bash [--format|--lint|--build|--test|--docs|--rebuild_docs|--activate_go2|--activate_go2_gpu_free|--activate_all|--activate_all_gpu_free|--fresh_install [go2|go2-gpu-free|all|all-gpu-free]]"
    echo "With --fresh_install, you can specify the environment to install. Pick from go2, go2-gpu-free, all, all-gpu-free"
    echo "For instance, run \"./tbai_ros.bash --fresh_install go2\" to install the go2 environment"
    echo "For instance, run \"./tbai_ros.bash --fresh_install go2-gpu-free\" to install the go2-gpu-free environment"
    echo "For instance, run \"./tbai_ros.bash --fresh_install all\" to install the all environment"
    echo "For instance, run \"./tbai_ros.bash --fresh_install all-gpu-free\" to install the all-gpu-free environment"
}

function activate_go2() {
	pixi shell -e "go2"
}

function activate_go2_gpu_free() {
	pixi shell -e "go2-gpu-free"
}

function activate_all() {
	pixi shell -e "all"
}

function activate_all_gpu_free() {
	pixi shell -e "all-gpu-free"
}

function lint() {
    folders=$(ls -d */| grep -v dependencies)
    for folder in $folders; do
        cpplint --recursive $folder
    done
}

function fresh_install() {
    env=$1
    echo "Installing $env environment"
    if [ -z "$env" ]; then
        echo "No environment specified. Pick from go2, go2-gpu-free, all, all-gpu-free"
        exit 1
    fi

    if [ "$env" = "go2" ]; then
        echo "Installing go2 environment"
        pixi install
        pixi run --environment go2 fresh_build_go2
    elif [ "$env" = "go2-gpu-free" ]; then
        echo "Installing go2-gpu-free environment"
        pixi install
        pixi run --environment go2-gpu-free fresh_build_go2_gpu_free
    elif [ "$env" = "all" ]; then
        echo "Installing all environment"
        pixi install
        pixi run --environment all fresh_build_all
    elif [ "$env" = "all-gpu-free" ]; then
        echo "Installing all-gpu-free environment"
        pixi install
        pixi run --environment all-gpu-free fresh_build_all_gpu_free
    else
        echo "Invalid environment: $env. Pick from go2, go2-gpu-free, all, all-gpu-free"
        exit 1
    fi

    echo "All good 🤗 You can now activate your environment with \"pixi shell -e $env\""
}

function format() {
    folders=$(ls -d */| grep -v dependencies)
    for folder in $folders; do
        for file in $(find $folder -name "*.hpp" -o -name "*.cpp"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done
}

function build() {
    ros_packages=""
    # Look for ROS packages in the root and tbai_ros_deploy_go2 folders
    search_dirs=(./ tbai_ros_deploy_go2/)
    for search_dir in "${search_dirs[@]}"; do
        # Only look for immediate subdirectories
        for folder in "$search_dir"*/; do
            # Remove trailing slash for consistency
            folder=${folder%/}
            # Check if both CMakeLists.txt and package.xml exist in the folder
            if [[ -f "$folder/CMakeLists.txt" ]] && [[ -f "$folder/package.xml" ]]; then
                package=$(basename "$folder")
                ros_packages+=" $package"
            fi
        done
    done
    echo "[TBAI] Building ROS packages: $ros_packages"
    catkin build $ros_packages
}

function test() {
    folders=$(ls -d */| grep -v dependencies)
    ros_packages=""
    # for each folder, check where test folder exists
    for folder in $folders; do
        if [[ -d $folder/test ]]; then
            echo "[TBAI] Running tests in $folder"
            package=$(basename $folder)
            ros_packages+=" $package"
        fi
    done
    echo "[TBAI] Running tests for ROS packages: $ros_packages"
    catkin test $ros_packages
}

function open_docs() {
    script_dir=$(dirname "$0")
    docs_dir=${script_dir}/../../build/tbai_ros_docs/output/doxygen/html/index.html
    google-chrome $docs_dir
}


if [[ "$1" == "--help" || "$1" == "-h" || -z "$1" ]]; then
	print_help
	exit
fi

if [[ "$1" == "--lint" ]]; then
	lint
	exit
fi

if [[ "$1" == "--format" ]]; then
    format
    exit
fi

if [[ "$1" == "--build" ]]; then
    build
    exit
fi

if [[ "$1" == "--test" ]]; then
    test
    exit
fi

if [[ "$1" == "--docs" ]]; then
    open_docs
    exit
fi

if [[ "$1" == "--activate_all" ]]; then
    activate_all
    exit
fi

if [[ "$1" == "--activate_all_gpu_free" ]]; then
    activate_all_gpu_free
    exit
fi

if [[ "$1" == "--activate_go2" ]]; then
    activate_go2
    exit
fi

if [[ "$1" == "--activate_go2_gpu_free" ]]; then
    activate_go2_gpu_free
    exit
fi

if [[ "$1" == "--fresh_install" ]]; then
    if [[ -z "$2" ]]; then
        print_help
        exit
    fi

    fresh_install "$2"
    exit
fi

if [[ "$1" == "--rebuild_docs" ]]; then
    catkin clean tbai_ros_docs
    catkin build tbai_ros_docs
fi

print_help
