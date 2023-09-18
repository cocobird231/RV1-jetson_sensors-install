#!/usr/bin/bash
target_dir="$HOME/jetson_sensors"
ros2_ws_dir="$HOME/ros2_ws"

PARSER_USED="FALSE" # Is parser used
PARSER_REMOVE="FALSE" # Remove current program and settings
PARSER_UPDATE="FALSE" # Update codePack, no install
PARSER_INSTALL="FALSE" # Install program from codePack

preserve_conf="FALSE" # Preserve current common.yaml file while installing
pack_name="NONE"
static_ip="NONE"
interface="eth0"


# Parser process orders
# 1.    PARSER_REMOVE       remove program under ros2_ws and environment settings
# 2.    PARSER_UPDATE       update codePack without installation
# 3.    PARSER_INSTALL      install program from codePack to ros2_ws
#       --preserve          preserve common.yaml under ros2_ws while installing
#       --interface         set network interface
#       --ip                set static ip (not supported)

while [[ $# -gt 0 ]]; do
    case $1 in
        -rm|--remove) # Can be worked independently
            PARSER_REMOVE="TRUE"
            PARSER_USED="TRUE"
            shift # past argument
            ;;
        -u|--update) # Can be worked independently
            PARSER_UPDATE="TRUE"
            PARSER_USED="TRUE"
            shift # past argument
            ;;
        -i|--install) # Can be worked independently
            PARSER_INSTALL="TRUE"
            PARSER_USED="TRUE"
            pack_name="$2" # <pack_name> or auto
            shift # past argument
            shift # past value
            ;;
        -p|--preserve)
            preserve_conf="TRUE"
            shift # past argument
            ;;
        --interface)
            interface="$2"
            shift # past argument
            shift # past value
            ;;
        --ip)
            static_ip="$2"
            shift # past argument
            shift # past value
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

function PrintError () # Red
{
    error_color="\033[1;91m"
    reset_color="\033[0m"
    printf "${error_color}%s${reset_color}\n" "$1"
}

function PrintSuccess () # Green
{
    success_color="\033[1;92m"
    reset_color="\033[0m"
    printf "${success_color}%s${reset_color}\n" "$1"
}

function PrintWarning () # Yellow
{
    warn_color="\033[1;93m"
    reset_color="\033[0m"
    printf "${warn_color}%s${reset_color}\n" "$1"
}

vercomp ()
{
    if [[ $1 == $2 ]]
    then
        return 0
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++))
    do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++))
    do
        if [[ -z ${ver2[i]} ]]
        then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]}))
        then
            return 1
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]}))
        then
            return 2
        fi
    done
    return 0
}

CheckParser ()
{
    # Check pwd
    CheckTargetPath

    # Check remove
    if [ "$PARSER_REMOVE" == "TRUE" ]
    then
        Remove
    fi

    # Check update
    if [ "$PARSER_UPDATE" == "TRUE" ]
    then
        UpdateCodePack
    fi

    # Check install
    if [ "$PARSER_INSTALL" == "TRUE" ]
    then
        if [ "$pack_name" == "auto" ]
        then
            CheckCurrentModule
            if [[ $? -eq 1 ]]
            then
                Remove
                return 1
            fi
        fi

        CheckRequirements
        InstallPackage # Return 0 if succeed
        if [[ $? -eq 0 ]]
        then
            EnvSetting
            SaveCurrentModule
        else
            Remove
            return 1
        fi
    fi
}

# cd into $target_dir
CheckTargetPath ()
{
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            PrintError "Target path error: $target_dir"
            exit 1
        fi
    fi
}

# Check cmake version and ros2
CheckRequirements ()
{
    # Check cmake version
    req_cmake_ver=3.16
    install_cmake=0
    if cmake --version &> /dev/null
    then
        cur_cmake_ver=$(cmake --version | grep -Po '(\d+.)+\d+')
        vercomp $cur_cmake_ver $req_cmake_ver
        case $? in
            0) op='=';;
            1) op='>';;
            2) op='<';;
        esac
        if [[ $op == '<' ]]
        then
            echo "cmake version does not fit the minimum required version: $req_cmake_ver"
            install_cmake=1
        fi
        echo "cmake version: $cur_cmake_ver $op $req_cmake_ver"
    else
        echo "cmake not found."
        install_cmake=1
    fi

    # Install cmake if needed
    if [[ $install_cmake != 0 ]]
    then
        echo "Installing cmake..."
        sudo apt update
        sudo apt install -y apt-transport-https ca-certificates gnupg software-properties-common wget
        wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo apt-key add -
        if [ "$ubuntu_ver" == "18.04" ]
        then
            sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' -y
        fi
        sudo apt update && sudo apt install -y cmake
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
}

# Will set $pack_name, $interface and $static_ip
CheckCurrentModule ()
{
    # Check pwd
    CheckTargetPath

    # Check previous module setting
    if cat .modulename &> /dev/null
    then
        pack_name=$(cat .modulename)
        echo "Found module name: $pack_name"
    else
        PrintError ".modulename not found. Run install.sh and select number to install module."
        return 1
    fi

    if cat .moduleinterface &> /dev/null
    then
        interface=$(cat .moduleinterface)
        echo "Found module interface: $interface"
    else
        PrintError ".moduleinterface not found. Run install.sh and select number to install module."
        return 1
    fi
    
    if cat .moduleip &> /dev/null
    then
        static_ip=$(cat .moduleip)
        echo "Found module ip: $static_ip"
    else
        PrintError ".moduleip not found. Run install.sh and select number to install module."
        return 1
    fi
    return 0
}

# Will create .modulename, .moduleinterface and .moduleip
SaveCurrentModule ()
{
    # Check pwd
    CheckTargetPath

    # Store selected module name, interface and ip into files
    touch .modulename
    echo $pack_name > .modulename
    touch .moduleinterface
    echo $interface > .moduleinterface
    touch .moduleip
    echo $static_ip > .moduleip
}

# Install package from codePack to $ros2_ws_dir
InstallPackage ()
{
    echo "===Install Process==="

    # Check pwd
    CheckTargetPath

    # Check $pack_name available
    if ls codePack/$pack_name &> /dev/null
    then
        echo "Package found: $pack_name"
    else
        return 1
    fi

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

    # Modify run.sh by adding specific $pack_name and source_env.txt
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

    # Preserve common.yaml under $ros2_ws_dir/src/launch if needed
    if [ "$preserve_conf" == "TRUE" ]
    then
        if [ "$ros_distro" == "eloquent" ]
        then
            if ls $ros2_ws_dir/src/$pack_name/launch/common_eloquent.yaml &> /dev/null
            then
                cp $ros2_ws_dir/src/$pack_name/launch/common_eloquent.yaml common.yaml.tmp
            else
                preserve_conf="FALSE"
            fi
        else
            if ls $ros2_ws_dir/src/$pack_name/launch/common.yaml &> /dev/null
            then
                cp $ros2_ws_dir/src/$pack_name/launch/common.yaml common.yaml.tmp
            else
                preserve_conf="FALSE"
            fi
        fi
    fi

    # Re-create $ros2_ws_dir/src and copy packages to $ros2_ws_dir/src
    rm -rf $ros2_ws_dir && mkdir -p $ros2_ws_dir/src
    cp -r codePack/$pack_name codePack/vehicle_interfaces $ros2_ws_dir/src

    # Preserve common.yaml under $ros2_ws_dir/src/launch if needed
    if [ "$preserve_conf" == "TRUE" ]
    then
        if [ "$ros_distro" == "eloquent" ]
        then
            mv common.yaml.tmp $ros2_ws_dir/src/$pack_name/launch/common_eloquent.yaml
        else
            mv common.yaml.tmp $ros2_ws_dir/src/$pack_name/launch/common.yaml
        fi
    fi

    # Link common.yaml to $target_dir
    if [ "$ros_distro" == "eloquent" ]
    then
        ln $ros2_ws_dir/src/$pack_name/launch/common_eloquent.yaml common.yaml
    else
        ln $ros2_ws_dir/src/$pack_name/launch/common.yaml common.yaml
    fi

    # Change directory to ROS2 workspace
    cd $ros2_ws_dir
    echo "Change directory: $PWD"

    # Install package
    source /opt/ros/$ros_distro/setup.bash
    colcon build --symlink-install
    return 0
}

# Create ros2_startup.desktop under /etc/xdg/autostart
EnvSetting ()
{
    echo "===Environment Setting==="

    # Create ros2_startup.desktop file
    rm -rf ros2_startup.desktop.tmp && touch ros2_startup.desktop.tmp
    echo "[Desktop Entry]" >> ros2_startup.desktop.tmp
    echo "Type=Application" >> ros2_startup.desktop.tmp
    echo "Exec=gnome-terminal --command '$target_dir/run.sh $interface'" >> ros2_startup.desktop.tmp
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

# Update codePack
UpdateCodePack ()
{
    echo "===Update Process==="

    # Check pwd
    CheckTargetPath

    # Check Internet Connection
    printf "%s" "Internet connecting..."
    while ! ping -w 1 -c 1 -n 168.95.1.1 &> /dev/null
    do
        printf "%c" "."
    done
    printf "\n%s\n" "Internet connected."

    # Check git
    if [ -x "$(command -v git)" ]; then
        echo "Found git." && git --version
    else
        echo "No git. Installing git..."
        sudo apt install git -y
    fi

    # Check git control
    if git status &> /dev/null
    then
        echo "git control checked."
    else
        PrintError "git control not found. \
    Delete jetson_sensors directory and run \
    'cd ~ && git clone https://github.com/davidweitaiwan/RV-1.0-jetson_sensors-install.git jetson_sensors' \
    to grab git controlled directory."
        return 1
    fi

    # Update submodules
    git submodule update --remote --recursive --force
}

# Remove common.yaml, .tmp files, $ros2_ws_dir and /etc/xdg/autostart/ros2_startup.desktop
Remove ()
{
    echo "===Remove Process==="

    # Check pwd
    CheckTargetPath
    
    # Target files
    rm -rf common.yaml
    rm -rf ros2_startup.desktop.tmp
    rm -rf .module*
    
    # Recover run.sh if .tmp exist
    if cat run.sh.tmp &> /dev/null
    then
        mv run.sh.tmp run.sh
        echo "run.sh recovered"
    fi

    # ROS2 workspace
    rm -rf $ros2_ws_dir
    
    # System files
    sudo rm -rf /etc/xdg/autostart/ros2_startup.desktop
}

# Manual installation
PreparePackage ()
{
    echo "===Prepare Packages==="

    # Check pwd
    CheckTargetPath

    ## Install Menu
    echo "################################################"
    printf "\t%s\n\n" "Jetson Sensor Package Installer"
    echo "1) ZED camera"
    echo "u) Update package (git control required)"
    echo "r) Remove package"
    echo "q) Exit"
    echo "################################################"
    read -p "Enter number for module installation. Enter 'u' for package update, 'r' for package removal or 'q' to exit:" selectNum

    if [ "$selectNum" == "q" ]
    then
        return 0
    elif [ "$selectNum" == "r" ]
    then
        Remove
        return 0
    elif [ "$selectNum" == "u" ]
    then
        CheckCurrentModule
        if [[ $? -eq 1 ]]
        then
            PrintError "[PreparePackage] CheckCurrentModule failed. Exiting..."
            return 1
        fi
        UpdateCodePack
        
        # Check Ubuntu ver, CMake ver and ROS2 distro
        CheckRequirements
        InstallPackage
        if [[ $? -eq 0 ]]
        then
            EnvSetting
            SaveCurrentModule
        else
            Remove
            return 1
        fi
    elif [ "$selectNum" == "1" ]
    then
        echo "[PreparePackage] Install ZED camera module..."
        pack_name="cpp_zedcam"
    else
        PrintError "[PreparePackage] Unknown input number. Exiting..."
        return 1
    fi

    # Check Ubuntu ver, CMake ver and ROS2 distro
    CheckRequirements

    # Network Interface Selection
    read -p "Enter network interface (default eth0):" interface
    if [ $interface ]
    then
        echo "Interface: $interface"
    else
        interface="eth0"
        echo "Default interface: $interface"
    fi

    # Network IP selection
    read -p "Use DHCP? (y/n):" static_ip
    if [[ "$static_ip" != "n" && "$static_ip" != "N" ]]
    then
        static_ip="NONE"
    else
        read -p "Enter static ip (ex 192.168.3.100/16):" static_ip
        if [ ! $static_ip ]
        then
            static_ip="NONE"
        fi
    fi
    echo "Static IP: $static_ip"

    # Install package
    InstallPackage
    if [[ $? -eq 0 ]]
    then
        EnvSetting
        SaveCurrentModule
    else
        Remove
        return 1
    fi
}

## Entry
################################################################

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
    PrintError "Ubuntu release version not supported: $ubuntu_ver"
    exit 1
fi

entry_pwd="$PWD"

if [ "$PARSER_USED" == "TRUE" ]
then
    CheckParser
else
    PreparePackage
fi

if [ "$PWD" != "$entry_pwd" ]
then
    if ls $entry_pwd &> /dev/null
    then
        cd $entry_pwd
        echo "Change directory: $PWD"
    fi
fi
