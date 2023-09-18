# Jetson Sensors Installation

Web version: https://hackmd.io/@cocobird231/BkmIsPa5n

*`Updated: 2023/09/18`*

The package installer for sensor depends on NVIDIA Jetson.

**System requirements**:
- OS: Ubuntu 18.04 or higher (64-bit recommend)
- RAM: 4G or higher

**Now supported sensor types**:
- ZED stereo camera (the ZED and ZED X were tested)

---
## Usage

### For Newly Install (No `jetson_sensors` Under Home Path)
Run the pre-install script `get-jetson-sensors-install.sh` to grab git controlled directory (renamed as `jetson_sensors`). **Make sure Jetson is connected to the internet before installation start.** There are **two** ways to run the pre-install script: 

1. **Run the pre-install script `get-jetson-sensors-install.sh` manually**
    ```bash
    . get-jetson-sensors-install.sh
    ```
2. **Run the pre-install script using `curl`**
    ```bash
    curl -fsSL ftp://61.220.23.239/rv-11/get-jetson-sensors-install.sh | bash
    ```
The new directory `jetson_sensors` will be created under `$HOME`.

### Module Installation
:::warning
The `static_ip` option not support yet. Always set to `dhcp` or ignore it.
:::
1. Run the script `install.sh` under `jetson_sensors` to install package for selected module.
    ```bash
    . ~/jetson_sensors/install.sh
    ```
    Select a number for module installation
2. Determine the network interface and IP for program to execute.
3. Reboot while installation finished. The program will be running at startup automatically.

- **The installation will automatically detect version and install ROS2, then create ROS2 workspace under `$HOME/ros2_ws`.**
- **The packages will be built and installed under `$HOME/ros2_ws`.**

:::warning
**Effects After Installation**
- Files Creation:
    - Under `$HOME/jetson_sensors`
        - `run.sh.tmp`
        - `.modulename` (selected module pack name)
        - `.moduleinterface` (interface setting)
        - `.modulename` (IP setting)
        - `ros2_startup.desktop.tmp` (may be deleted)
        - `common.yaml` (linked from module package under ROS2 workspace)
    - Under `$HOME/ros2_ws`
        - `build`
        - `install`
        - `log`
        - `src`
    - Under System Environment
        - `/etc/xdg/autostart/ros2_startup.desktop` (startup file)
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
3. After pulling, the module package will start rebuilding if module package had been installed before.

### Module Removal
1. Run the script `install.sh` under `jetson_sensors` directory and enter `r` for package remove process.
    ```bash
    . install.sh
    
    # Enter 'r' for update process
    ```
The files which describes under **Effects After Installation** section will be removed except the files installed from `install_dependencies.sh`.

### Parameters Setting for ROS2
Settings may be varient in different sensors, but there are some common parameters need to be changed:
1. Device node name (under `generic_prop` tag)
2. Device ID (under `generic_prop` tag)
3. Topic name (may be one or more)
4. Publish interval (Need to be float, e.g. not `1` but `1.0`)

Modify the settings under `$HOME/jetson_sensors/common.yaml` and reboot device.
![](https://hackmd.io/_uploads/Hy4Efgk1a.png)

---

### One Line Command
:::success
The `install.sh` now supported parser installation. (2023/07/25)
:::
The package installation, updating and removal functions can be done by adding some arguments while running `install.sh`.

#### **Overall arguments**
```bash!
. install.sh [[-i|--install] <package_name>] [--interface <network_interface>] [--ip [<static_ip>|dhcp]] [-rm|--remove] [-u|--update] [-p|--preserve]
```
- Command explanation
    - **`-i|--install`**: install specific package from codePack to ROS2 workspace.
    - **`-rm|--remove`**: remove installed packages and environment settings.
    - **`-u|--update`**: update codePack without install packages.
    - **`-p|--preserve`**: preserve `common.yaml` file during installation.
    - **`--interface`**: determine the network interface during installation.
    - **`--ip`**: determine the network ip during installation.
- Variable explanation
    - **package_name**: real package name under codePack. If variable set to `auto`, the process will automatically detect current installed module settings, then install the packages.
    - **network_interface**: the network interface to detect network or internet connection, e.g. `eth0` or `wlan0`.
    - **static_ip**: the format should be like `ip/mask`, e.g. `192.168.1.10/16`.

:::info
- **For Commands**

The three commands `-i`, `-u` and `-rm` can be work independently. The priority order of the three commands from high to low are: `-rm` > `-u` > `-i`. That is, if three commands exists at the same time, the process will be:
1. Remove installed packages, ROS2 workspace and environment settings.
2. Update codePack without install any packages.
3. Install packages from codePack to ROS2 workspace.

The flag `-p` tells the installer to keep old `common.yaml` file. If `-p` set but `-i` not set, the `-p` will be ignored. If `-p` set but the `common.yaml` file not found, the preservation will be ignored.

The `--interface` determines the network interface for network detection or internet detection while installed program startup. The `--interface` will be ignored if `-i` not set. The `--interface` is not necessary and will be set to default `eth0`.

The `--ip` currently not support and will always set to `dhcp`. The `--ip` is not necessary and will be set to default `dhcp`.
:::

:::warning

- **For Variables**

The default value of the variables describes as following:
- `package_name`: `NONE`
- `network_interface`: `eth0`
- `static_ip`: `NONE`

The `package_name` is necessary if `-i` set. The valid name of `package_name` were shown under codePack. If the package had installed before, set `package_name` to `auto` is valid for process to auto detect the package name.

If `package_name` set to `auto`, the process will ignored `--interface` and `--ip` settings.

The `network_interface` is necessary if `--interface` set. The `network_interface` do not have any valid check mechanism, be careful of the mistyping.

The `static_ip` is necessary if `--ip` not set to `dhcp`. The `static_ip` currently not support.
:::

#### **Examples**
- Install package
    ```bash!
    . install.sh -i <package_name> [--interface <network_interface>] [--ip [<static_ip> | dhcp]]
    ```
- Update codePack
    ```bash!
    . install.sh -u
    ```
- Update codePack then install package
    ```bash!
    . install.sh -i <package_name> [--interface <network_interface>] [--ip [<static_ip> | dhcp]] -u
    ```
- Update current package while preserve `common.yaml`
    ```bash!
    . install.sh -u -i auto -p
    ```
- Remove current package
    ```bash
    . install.sh -rm
    ```
- Remove current package, update codePack and install package
    ```bash!
    . install.sh -rm -u -i <package_name> [--interface <network_interface>] [--ip [<static_ip> | dhcp]]
    ```