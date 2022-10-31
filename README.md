# DCA1000EVM CLI

![galactic_build](https://github.com/3473f/DCA1000EVM_CLI_ROS2/actions/workflows/galactic_build.yaml/badge.svg)
![humble_build](https://github.com/3473f/DCA1000EVM_CLI_ROS2/actions/workflows/humble_build.yaml/badge.svg)


## Overview
The DCA1000EVM CLI package is A ROS2 wrapper for the DCA1000EVM CLI provided by TI to control and configure the DCA1000EVM capture board.

The package was tested under ROS2 Galactic and Humble on Ubuntu 20.04 and Ubuntu 22.04 respectively.

## Installation
1. Install ROS Galactic or Humble.
2. Make sure that `colcon` is installed:

    ```
    sudo apt install python3-colcon-common-extensions
    ```

3. Clone this repo into your workspace:

    ```
    git clone https://github.com/3473f/DCA1000EVM_CLI_ROS2
    ```

4. Install dependencies and build the workspace:

    ```
    rosdep update
    rosdep install --ignore-src --from-paths src -y -r
    colcon build
    source install/setup.bash
    ```

5. Set the `LD_LIBRARY_PATH` variable to point to the CLI's library file. In order to do so, add the following to the `~/.bashrc` file:

    ```
    LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path_to_workspace>/install/dca1000evm_cli/share/dca1000evm_cli/cli/
    export LD_LIBRARY_PATH
    ```

## Included packages

`dca1000evm_cli` - provides the interface for communication with the capture card.

`dca1000evm_cli_interface` - defines the services used by the interface.

## Usage
To launch the node using the parameters defined in `config/params.yaml` use the following command:

```
ros2 launch dca1000evm_cli dca1000evm_cli.launch.py
```

## Config files
 - **params.yaml** Contains the parameters for the dca1000evm_cli node.
 - **dca1000evm.json** The configuration file to be sent to the DCA1000EVM board as documented by TI.
 
## Nodes
### dca1000evm_cli_node

#### Parameters

- **`config_file`** (string, default: "None")
    
    The path of the board configuration (JSON) file.

- **`cli_timeout`** (int, default: 10)

    The timeout for the response from the CLI interface.
#### Services
- **`start_record`** ([dca1000evm_interface/srv/StartRecord](https://github.com/3473f/DCA1000EVM_CLI_ROS2/blob/main/dca1000evm_interface/srv/StartRecord.srv))

    Start recording.

- **`stop_record`** ([dca1000evm_interface/srv/StopRecord](https://github.com/3473f/DCA1000EVM_CLI_ROS2/blob/main/dca1000evm_interface/srv/StopRecord.srv))

    Stop recording.

- **`fpga_config`** ([dca1000evm_interface/srv/FpgaConfig](https://github.com/3473f/DCA1000EVM_CLI_ROS2/blob/main/dca1000evm_interface/srv/FpgaConfig.srv))

    Send FPGA configuration to the board.

- **`record_config`** ([dca1000evm_interface/srv/RecordConfig](https://github.com/3473f/DCA1000EVM_CLI_ROS2/blob/main/dca1000evm_interface/srv/RecordConfig.srv))

    Send record configuration to the board.

- **`reset_ar_device`** ([dca1000evm_interface/srv/ResetArDevice](https://github.com/3473f/DCA1000EVM_CLI_ROS2/blob/main/dca1000evm_interface/srv/ResetArDevice.srv))

    Reset the sensor attached to the DCA1000EVM board.
