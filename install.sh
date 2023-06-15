#!/usr/bin/bash
target_dir="$HOME/jetson_sensors"
ros2_ws_dir="$HOME/ros2_ws"

PARSER_UPDATE="NONE"
PARSER_INSTALL="NONE"
pack_name="NONE"
static_ip="NONE"
interface="eth0"

while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--extension)
      EXTENSION="$2"
      shift # past argument
      shift # past value
      ;;
    -i|--install)
      PARSER_INSTALL="install"
      pack_name="$2"
      shift # past argument
      shift # past value
      ;;
    --reinstall)
      PARSER_INSTALL="reinstall"
      shift # past argument
      ;;
    --forced_update)
      PARSER_UPDATE="forced_update"
      shift # past argument
      ;;
    --preserved_update)
      PARSER_UPDATE="preserved_update"
      shift # past argument
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 0
      ;;
    *)
      shift # past argument
      ;;
  esac
done

CheckCurrentModule ()
{
    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "jetson_sensors path error. Please copy jetson_sensors directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/jetson_sensors

    # Check previous module setting
    if cat .modulename &> /dev/null
    then
        pack_name=$(cat .modulename)
        echo "Found module name: $pack_name"
    else
        echo ".modulename not found. Run install.sh and select number to install module."
        exit 1
    fi

    if cat .moduleinterface &> /dev/null
    then
        interface=$(cat .moduleinterface)
        echo "Found module interface: $interface"
    else
        echo ".moduleinterface not found. Run install.sh and select number to install module."
        exit 1
    fi
    
    if cat .moduleip &> /dev/null
    then
        static_ip=$(cat .moduleip)
        echo "Found module ip: $static_ip"
    else
        echo ".moduleip not found. Run install.sh and select number to install module."
        exit 1
    fi
}

SaveCurrentModule ()
{
    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "jetson_sensors path error. Please copy jetson_sensors directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/jetson_sensors

    # Store selected module name, interface and ip into files
    touch .modulename
    echo $pack_name > .modulename
    touch .moduleinterface
    echo $interface > .moduleinterface
    touch .moduleip
    echo $static_ip > .moduleip
}

PreparePackage ()
{
    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "jetson_sensors path error. Please copy jetson_sensors directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/jetson_sensors

    # Check Ubuntu release
    ubuntu_ver=$(lsb_release -a | grep Release | grep -Po '[\d.]+')
    if [ "$ubuntu_ver" == "18.04" ]
    then
        ros_distro="eloquent"
    elif [ "$ubuntu_ver" == "20.04" ]
    then
        ros_distro="foxy"
    elif [ "$ubuntu_ver" == "22.04" ]
    then
        ros_distro="humble"
    fi

    # Check ROS2
    if source /opt/ros/$ros_distro/setup.bash &> /dev/null
    then
        mkdir -p $ros2_ws_dir/src
        echo "Found ROS2 distro: $ros_distro."
    else
        echo "Source ROS2 distro $ros_distro error. Installing ROS2..."
        sudo chmod a+x ./install_ros2.sh
        ./install_ros2.sh
    fi

    # Network Interface Selection
    echo "Enter network interface (default eth0):"
    read interface
    if [ $interface ]
    then
        echo "Interface: $interface"
    else
        interface="eth0"
        echo "Default interface: $interface"
    fi

    # Network IP Selection
    echo "Use DHCP? (y/n):"
    read static_ip
    if [[ "$static_ip" == "y" || "$static_ip" == "Y" ]]
    then
        static_ip="NONE"
    else
        echo "Enter static ip (ex 192.168.3.100/16):"
        read static_ip
        if [ ! $static_ip ]
        then
            static_ip="NONE"
        fi
    fi
    echo "Static IP: $static_ip"

    # Save module info
    SaveCurrentModule
}

InstallPackages ()
{
    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "jetson_sensors path error. Please copy jetson_sensors directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/jetson_sensors

    # Install package dependencies
    sudo chmod a+x ./codePack/$pack_name/install_dependencies.sh
    . ./codePack/$pack_name/install_dependencies.sh

    # Recover run.sh if .tmp exist
    if cat run.sh.tmp &> /dev/null
    then
        cp run.sh.tmp run.sh
        echo "run.sh recovered"
    else
        cp run.sh run.sh.tmp
        echo "Backup run.sh: run.sh.tmp"
    fi
    
    # Modify run.sh by adding specific $pack_name source_env.txt and docker run process
    cat ./codePack/$pack_name/source_env.txt >> run.sh
    echo "cd $ros2_ws_dir" >> run.sh
    echo "source $ros2_ws_dir/install/setup.bash" >> run.sh
    if [ "$ros_distro" == "eloquent" ]
    then
        echo "ros2 launch $pack_name launch_eloquent.py" >> run.sh
    else
        echo "ros2 launch $pack_name launch.py" >> run.sh
    fi
    sudo chmod a+x run.sh

    # Check ROS2 workspace
    if ls $ros2_ws_dir/src &> /dev/null
    then
        echo "Found ROS2 workspace: $ros2_ws_dir"
    else
        mkdir -p $ros2_ws_dir/src
        echo "Create ROS2 workspace at $ros2_ws_dir/src"
    fi

    # Copy packages into src under ros2 workspace
    rm -rf $ros2_ws_dir/src/$pack_name
    rm -rf $ros2_ws_dir/src/vehicle_interfaces
    cp -rv codePack/$pack_name $ros2_ws_dir/src
    cp -rv codePack/vehicle_interfaces $ros2_ws_dir/src
    rm -rf $ros2_ws_dir/run.sh && cp run.sh $ros2_ws_dir/run.sh

    # Link ros2 workspace common.yaml file to ~/jetson_sensors for convenient modifying
    rm -rf common.yaml && ln $ros2_ws_dir/src/$pack_name/launch/common.yaml common.yaml

    # Change directory to ROS2 workspace
    cd $ros2_ws_dir
    echo "Change directory: $PWD"

    # Install package
    source /opt/ros/$ros_distro/setup.bash
    colcon build --packages-select $pack_name vehicle_interfaces --symlink-install
}

EnvSetting ()
{
    # Create ros2_startup.desktop file
    rm -rf ros2_startup.desktop.tmp && touch ros2_startup.desktop.tmp
    echo "[Desktop Entry]" >> ros2_startup.desktop.tmp
    echo "Type=Application" >> ros2_startup.desktop.tmp
    echo "Exec=gnome-terminal --command '$ros2_ws_dir/run.sh'" >> ros2_startup.desktop.tmp
    echo "Hidden=false" >> ros2_startup.desktop.tmp
    echo "NoDisplay=false" >> ros2_startup.desktop.tmp
    echo "X-GNOME-Autostart-enabled=true" >> ros2_startup.desktop.tmp
    echo "Name[en_NG]=ros2_startup" >> ros2_startup.desktop.tmp
    echo "Name=ros2_startup" >> ros2_startup.desktop.tmp
    echo "Comment[en_NG]=Start ROS2 Application" >> ros2_startup.desktop.tmp
    echo "Comment=Start ROS2 Application" >> ros2_startup.desktop.tmp

    # Copy ros2_startup.desktop to autostart directory
    sudo cp ros2_startup.desktop.tmp /etc/xdg/autostart/ros2_startup.desktop
    rm -rf ros2_startup.desktop.tmp
}

UpdateCodePack ()
{
    # Check Internet Connection
    printf "%s" "Internet connecting..."
    while ! ping -w 1 -c 1 -n 168.95.1.1 &> /dev/null
    do
        printf "%c" "."
    done
    printf "\n%s\n" "Internet connected."

    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "jetson_sensors path error. Please copy jetson_sensors directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/jetson_sensors

    # Check git
    if [ -x "$(command -v git)" ]; then
        echo "Found git." && git --version
    else
        echo "No git. Installing git..."
        # sudo apt install git -y
    fi

    # Check git control
    if git status &> /dev/null
    then
        echo "git control checked."
    else
        echo "git control not found. \
Delete jetson_sensors directory and run \
'cd ~ && git clone https://github.com/cocobird231/RobotVehicle-1.0-ROS2-jetson_sensors.git jetson_sensors' \
to grab git controlled directory."
        exit 1
    fi

    # Ask if preserve common.yaml file
    echo "Preserve current common.yaml file ?(y/n):"
    read selectNum
    if [ "$selectNum" == "y" ]
    then
        # Check previous module setting
        if cat .modulename &> /dev/null
        then
            pack_name=$(cat .modulename)
            cp codePack/$pack_name/launch/common.yaml common.yaml.tmp
        else
            echo ".modulename not found. common.yaml will not preserved."
            selectNum="n"
        fi
    fi

    # Update submodules
    git submodule update --remote --recursive --force

    # Recovering common.yaml
    if [ "$selectNum" == "y" ]
    then
        mv common.yaml.tmp codePack/$pack_name/launch/common.yaml
        echo "common.yaml recovered."
    fi
}

## Install Menu
echo "################################################"
printf "\t%s\n\n" "Jetson Sensor Package Installer"
echo "1) ZED camera"
echo "u) Update module (git control required)"
echo "q) Exit"
echo "################################################"
echo "Enter number for module installation. Enter 'u' for module update or 'q' to exit:"
read selectNum

if [ "$selectNum" == "1" ]
then
    echo "Install ZED camera module..."
    pack_name="cpp_zedcam"
elif [ "$selectNum" == "u" ]
then
    echo "Updating module..."
    pack_name="NONE"
    UpdateCodePack
    CheckCurrentModule
    InstallPackages
    pack_name="NONE"
else
    pack_name="NONE"
fi

if [ "$pack_name" != "NONE" ]
then
    echo "Preparing package..."
    PreparePackage
    InstallPackages
    EnvSetting
else
    echo "Process ended."
fi