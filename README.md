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

## How to run
0. A joystick needs to be plugged in the computer
1. See [Go2_control_interface: How to control the robot](https://github.com/inria-paris-robotics-lab/go2_control_interface?tab=readme-ov-file#how-to-control-the-robot). Most likely you will run:
    ```bash
    # mamba activate go2_control_interface
    # source install/setup.bash
    # source <(ros2 run go2_control_interface autoset_environment_dds.py SIMULATION)
    ros2 launch go2_simulation launch_sim.launch.py
    ```
    to start the simulation.
1. In another terminal:
    ```bash
    # mamba activate go2_control_interface
    # source install/setup.bash
    # source <(ros2 run go2_control_interface autoset_environment_dds.py SIMULATION)
    ros2 launch onnx_controller controller_and_joystick.launch.py
    ```
