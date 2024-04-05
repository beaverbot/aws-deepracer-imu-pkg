# This fork of the robofoundry fork implements the following features:
1. Fix for conversion to g when calculating the linear acceleration.

2. Documentation of our wiring for a Jetson Nano

3. Documentation for a fix to the BMI160-i2c library that adds missing constants, enabling usage of the gyroscope calibration

4. Autocalibration for the gyroscope

## Fix to BMI160-i2c library
Some constants are missing in definitions.py. A fix has been made, but not implemented.
We will implement this manually.
See the files to be fixed [here](https://github.com/lefuturiste/BMI160-i2c/pull/12/commits/006d53002f381236a8ed9de33f903a395677978f).

If the package was installed via pip, the files to be adjusted can be found in `~/.local/lib/python3.6/site-packages/BMI160_i2c`.
Adjust `__init__.py` and `definitions.py` according to the pull request above.

This actually just equates to pasting the following lines at the bottom of definitions.py.
The changes in `__init__.py` don't affect functionality.
```
# No Motion
NOMOTION_EN_BIT     = (0)
NOMOTION_EN_LEN     = (3)
NOMOTION_INT_BIT    = (7)
NOMOTION_DUR_BIT    = (2)
NOMOTION_DUR_LEN    = (6)
NOMOTION_SEL_BIT    = (0)
NOMOTION_SEL_LEN    = (1)

# Frequency
ACCEL_RATE_SEL_BIT  = (0)
ACCEL_RATE_SEL_LEN  = (4)
```

Additionally, the following line should be added on line 5.
This fix is necessary for calibrating the gyroscope, but wasn't fixed in the above fix.

```
GYR_OFFSET_EN       = (7)	# Added as part of a manual fix
```

## Wiring
BME160 pinouts are visible on the bottom of the chip.
Pins are referred to using the following system, looking at the top of the sensor (the side with the chips on it) with the orientation diagram of the sensor on the bottom right.

|Pinout | |
|--------|----------|
| 1- VIN |          |
| 2- 3V3 | 8-  OCS  |
| 3- GND | 9-  INT2 |
| 4- SCL | 10- INT1 |
| 5- SDA | 11- SCX  |
| 6- CS  | 12- SDX  |
| 7- SAO |          |

[Jetson Nano Pinout](https://developer.nvidia.com/embedded/learn/jetson-nano-2gb-devkit-user-guide)

The pins should then be connected from the BME160 to the Jetson Nano.

| BMI160 pin | Jetson Nano pin | Purpose |
|------------|-----------------|---------|
| 2          | 1               | 3.3V    |
| 3          | 9               | GND     |
| 4          | 28              | I2C CLK |
| 5          | 27              | I2C SDA |

Once it is wired, the following command should show a device on I2C address 69.
```
sudo i2cdetect -r -y 1
```

Note that on a new Jetson, the Jetson.GPIO package must be installed to enable GPIO and I2C usage first.
```
sudo pip install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio admetal
```

## Build

Recommended to build with `--symlink-install`, which makes symlinks to original Python files to install, so when changing Python code, don't have to rebuild.

```
colcon build --packages-select imu_pkg --symlink-install
```

## Run

We are using ROS2 Humble, not Foxy as in the original. If it is not already done in ~/.bashrc, the ros2_ws should be sourced.
The node can then be run in one terminal, and output checked by echoing the topic in another. The ros2_ws must be sourced in both.

```
# Launch the node
source ~/ros2_ws/install/setup.bash
ros2 launch imu_pkg imu_pkg_launch.py
```

```
# Check the messages published by the node
source ~/ros2_ws/install/setup.bash
ros2 topic echo imu/data_raw
```

### Original RoboFoundry AWS DeepRacer ReadMe below


# This fork adds following capabilities:
1. Ability to set the config params for device_address, publication topic, frame_id, publishing rate etc.

2. Simulation of IMU movements in RVIZ2

For step-by-step details follow this article:
https://robofoundry.medium.com/using-bmi160-imu-with-ros2-ecb550851efa


### Original AWS DeepRacer ReadMe below

# AWS DeepRacer Inertial Measurement Unit

## Overview

The AWS DeepRacer sensor fusion ROS package creates the `imu_node`, which is an additional package for AWS DeepRacer that can be used to provide Acceleration and Gyroscope readings from the built in BMI160 IMU.

This node is responsible for collecting the messages from the IMU and publishing the resulting sensor message. 

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer sensor fusion package.

### Prerequisites

The `imu_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `geometry_msgs`: This package contains the messages for geometric messages.
1. `sensor_msgs`: This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.

Additionally the following Python Packages are needed:

1. `smbus2` which allows communication via the i2c bus.
1. `BMI160-i2c` which is a driver for the Bosch BMI160 IMU.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Install the Python packages:

        pip install BMI160-i2c smbus2

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `imu_pkg` on the AWS DeepRacer device:

        git clone https://github.com/larsll/aws-deepracer-imu-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/aws-deepracer-imu-pkg
        rosws update

1. Resolve the `imu_pkg` dependencies:

        cd ~/deepracer_ws/aws-deepracer-imu-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `imu_pkg`:

        cd ~/deepracer_ws/aws-deepracer-imu-pkg && colcon build --packages-select imu_pkg

## Usage

The `imu_node` provides the core functionality to combine the sensor data from various sensors connected to the AWS DeepRacer vehicle. Although the node is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `imu_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/aws-deepracer-imu-pkg/install/setup.bash

1. Launch the `imu_node` using the launch script:

        ros2 launch imu_pkg imu_pkg_launch.py

## Launch files

The `imu_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the nodes independently from the core application.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='imu_pkg',
                namespace='imu_pkg',
                executable='imu_node',
                name='imu_node'
            )
        ])

## Node details

### `imu_node`

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`imu_pkg`/`imu_msg`/`raw`|Imu|Publisher that publishes the readings from the IMU in 6 dimensions.|
