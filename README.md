# arducam_ros2
This package interfaces Arducam cameras that support v4l2 to ROS 2

**NOTE** This package is tested on Jetosn Xavier NX with Jetpack `L4T_VERSION=35.1.0`
At the time of writing this package (June 2023), ROS 2 `humble` does not run directly on the host Jetson. It requires a docker container. You can use docker images from [jetson-containers](https://github.com/dusty-nv/jetson-containers)

# Installation
* Use a suitable Docker image according to your Jetpack version. See [jetson-containers](https://github.com/dusty-nv/jetson-containers)
* Install the Arducam drivers in your Jetson
    ```bash
    cd ~
    wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
    chmod +x install_full.sh
    ./install_full.sh -m arducam
    ```
    **Reference**: [arducam website](https://docs.arducam.com/Nvidia-Jetson-Camera/Jetvariety-Camera/Quick-Start-Guide/#1check-and-validate-the-camera-connection)
* **Inside the Docker container**, install the v4l2 tools
    ```bash
    sudo apt-get install v4l-utils
    sudo pip3 install v4l2-fix  
    sudo pip3 install jetson-stats
    ```

* Create a workspace, for example called `ros2_ws` and clone this package to `ros2_ws/src`
    ```bash
    cd ros2_ws/src
    git clone https://github.com/mzahana/arducam_ros2.git
    ```
* Build and source the workspace
    ```bash
    cd ros2_ws
    colcon build
    source install/setup.bash
    ```

# Run the ROS node
**NOTE All of the following step are done inside the Docker container**

* Adjust the camera `.yaml` files located inside the `arducam_ros2/config` directory, according to you camera. This is usually the result of your camera calibration process. You can use
* Build and source the workspace after any modifications
* Modify the node parameters inside the `launch/stereo.launch.py`
* Inside the terminal of the docker container, run the `launch/stereo.launch.py`
    ```bash
    ros2 launch arducam_ros2 stereo.launch.py
    ```
* In a new terminal, you can list the topic
    ```bash
    `ros2 topic list
    ```
# Camera calibration
* Reference: [tutorial](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
* [Checker board generator](https://calib.io/pages/camera-calibration-pattern-generator)

# Teste Cameras
* [Dual OV9281](https://www.uctronics.com/arducam-1mp-stereo-camera-for-raspberry-pi-nvidia-jetson-nano-and-xavier-nx-dual-ov9281-monochrome-global-shutter-camera-module.html)