# Jetson Sensors Installation

Web version: https://hackmd.io/@cocobird231/BkmIsPa5n

*`Updated: 2023/07/25`*

The sensor installation pack for NVIDIA Jetson. The installation will automatically detect and install dependencies.

**System requirements**:
- OS: Ubuntu 18.04 or higher (64-bit recommend)
- RAM: 4G or higher

**Now supported sensor types**:
- ZED stereo camera

---
## Usage

### For Newly Install (No `jetson_sensors` Under Home Path)
Run the pre-install script `get-jetson-sensors-install.sh` to grab git controlled SensorPack directory (renamed as `jetson_sensors`). **Make sure Jetson is connected to the internet before installation.** There are **two** ways to run the script: 

1. **Run the pre-install script `get-jetson-sensors-install.sh` manually**
    ```bash
    . get-jetson-sensors-install.sh
    ```
2. **Run the pre-install script using `curl`**
    ```bash
    curl -fsSL ftp://61.220.23.239/rv-10/get-jetson-sensors-install.sh | bash
    ```
The new directory `jetson_sensors` will be created under `$HOME`.

### Module Installation
:::warning
The `static_ip` option not support yet. Always set to `dhcp` or ignore it.
:::
1. Run the script `install.sh` under `jetson_sensors` to install program for selected module.
    ```bash
    cd ~/jetson_sensors
    ```
    
    ```bash
    . install.sh
    ```
    Select a number for module installation
2. Determine the network interface and IP for program to execute.
3. Reboot while installation finished. The program will be running at startup automatically.

:::warning
**Effects After Installation**
- Files Creation:
    - Under `$HOME/jetson_sensors`
        - `run.sh.tmp`
        - `.modulename` (selected module pack name)
        - `.moduleinterface` (interface setting)
        - `.modulename` (IP setting)
    - Under System Environment
        - `/etc/xdg/autostart/ros2_startup.desktop` (startup)
- Dependencies Installation
    - Determined by `install_dependencies.sh`
:::


### Module Update
1. Run the script `install.sh` under `jetson_sensors` directory and enter `u` for update process.
    ```bash
    . install.sh
    
    # Enter 'u' for update process
    ```
2. The update process will update `codePack` under `jetson_sensors` by pulling repositories from git server.
3. After pulling, the module program will start rebuilding if module program had been installed before.

---
### Parameters Setting for ROS2
Settings may be varient in different sensors, but there are some common parameters need to be changed:
1. Device node name (under `generic_prop` tag)
2. Device ID (under `generic_prop` tag)
3. Topic name (may be one or more)
4. Publish interval (Need to be float, e.g. not `1` but `1.0`)

Modify the settings under `~/jetson_sensors/common.yaml` and reboot device.


### Module Removal
:::success
The installer now supports remove option. (2023/07/25)
:::
- **Run `install.sh` with `--remove` option**
    ```bash
    . install.sh --remove
    ```
The files which describes at **Effects After Installation** section will be removed except the files installed from `install_dependencies.sh`.

### One Line Command
:::success
The `install.sh` now supported parser installation. (2023/07/25)
:::
The package installation, updating and removal functions can be done by adding some arguments while running `install.sh`.
- **Install new package**
    ```bash
    . install.sh -i <package_name> --interface <network_interface> --ip [<static_ip> | dhcp]
    ```
    Variable descriptions: 
    - **package_name**: real packages name, for now, just `cpp_zedcam`.
    - **network_interface**: the network interface to detect network or internet connection, e.g. `eth0` or `wlan0`.
    - **static_ip**: the format should be like `ip/mask`, e.g. `192.168.1.10/16`.

- **Update existing package**
    - Preserve common.yaml file
        ```bash
        . install.sh --preserve-update
        ```
    - New common.yaml file
        ```bash
        . install.sh --force-update
        ```
    The process returns 1 if current package not found (.modulename file empty or not found)
- **Remove installed package**
    ```bash
    . install.sh --remove
    ```
