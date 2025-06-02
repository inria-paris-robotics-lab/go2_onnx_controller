Go2 ONNX Controller
===

## How to install
1. Follow [go2_control_interface](https://github.com/inria-paris-robotics-lab/go2_control_interface) install procedure.
2. Then clone this repo in the cyclonedds_ws/src/ directory (next to the go2_control_interface repo for instance)
    ```bash
    cd src  # cd unitree_ros2/cyclonedds_ws/src
    git clone --recurse-submodules git@github.com:inria-paris-robotics-lab/go2_onnx_controller.git
    ```
3. Recompile
    ```bash
    cd ../
    colcon build --packages-skip unitree_sdk2_python
    source install/setup.bash
    ```
