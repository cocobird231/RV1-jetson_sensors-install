#!/usr/bin/bash

ros2_ws_dir="$HOME/ros2_ws"

# Check Ubuntu release
ubuntu_ver=$(lsb_release -r | grep Release | grep -Po '[\d.]+')
if [ "$ubuntu_ver" == "18.04" ]
then
    ros_distro="eloquent"
elif [ "$ubuntu_ver" == "20.04" ]
then
    ros_distro="foxy"
elif [ "$ubuntu_ver" == "22.04" ]
then
    ros_distro="humble"
else
    echo "Ubuntu release not supported."
    exit 1
fi
echo "Ubuntu release: $ubuntu_ver"
echo "Corresponding ROS2 distro: $ros_distro"

# Check ROS2
if source /opt/ros/$ros_distro/setup.bash &> /dev/null
then
    echo "ROS2 distro $ros_distro already installed."
    if ls $ros2_ws_dir/src &> /dev/null
    then
        echo "Found ROS2 workspace: $ros2_ws_dir"
    else
        mkdir -p $ros2_ws_dir/src
        echo "Create ROS2 workspace at $ros2_ws_dir"
    fi
else
    # Check Internet Connection
    printf "%s" "Internet connecting..."
    while ! ping -w 1 -c 1 -n 168.95.1.1 &> /dev/null
    do
        printf "%c" "."
    done
    printf "\n%s\n" "Internet connected."

    # Install ROS2
    echo "ROS2 distro $ros_distro not found. Installing..."
    if [ "$ros_distro" == "eloquent" ]
    then
        sudo apt update && sudo apt install curl gnupg2 lsb-release -y
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
        sudo apt update
        sudo apt install ros-eloquent-ros-base -y
    elif [ "$ros_distro" == "foxy" ]
    then
        sudo apt install software-properties-common
        sudo add-apt-repository universe
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install ros-foxy-ros-base python3-argcomplete
    elif [ "$ros_distro" == "humble" ]
    then
        sudo apt install software-properties-common
        sudo add-apt-repository universe
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install ros-humble-ros-base
    fi
    echo "ROS2 distro $ros_distro installed."

    # Add alias to ~/.bashrc
    echo "alias $ros_distro='source /opt/ros/$ros_distro/setup.bash'" >> $HOME/.bashrc

    # Install dependencies
    echo "Install ROS2 dependencies..."
    sudo apt install python3 python3-dev python3-pip python3-rosdep python3-colcon-common-extensions nano git -y
    echo "ROS2 dependencies installed."

    # Create ROS2 workspace at ~/ros2_ws
    mkdir -p $HOME/ros2_ws/src
    echo "Created ROS2 workspace at $HOME/ros2_ws."

    echo "================================================"
    echo "Summary"
    echo "Ubuntu release: $ubuntu_ver"
    echo "Installed ROS2 distro: $ros_distro"
    echo "Installed dependencies: python3 python3-dev python3-pip python3-rosdep python3-colcon-common-extensions nano git"
    echo "Create ROS2 workspace: $HOME/ros2_ws."
    echo "Create alias $ros_distro under $HOME/.bashrc for ROS2 environment setup."
    echo "================================================"
fi
