# Nakama Franka Bag tools

## Usage

### Building

It is recommended to use the `colcon build --symlink-install` flag for recording data. This way, any changes made to the `.yaml` configuration files or `.launch.py` launch files inside the workspace `src` directory are symlinked to the `install` folder. This means that you do not have to re-run `colcon build` every time you change a configuration.

### Recording

To record motions, you need two terminals.

- In terminal 1, execute:

  ```bash
  cd ~/nakama_ws
  source install/setup.bash

  # Move robot to neutral starting position, interrupt with ctrl-C once the motion is done
  ros2 launch franka_bringup move_to_start_example_controller.launch.py arm_id:=fr3 robot_ip:=172.16.0.2

  # Start the recorder, do not interrupt until the recording is stopped in the second terminal
  ros2 launch nakama_franka_bringup franka_bag_recorder.launch.py
  ```

- In terminal 2, execute:

  ```bash
  cd ~/nakama_ws
  source install/setup.bash

  # Start recording, look at file nakama_franka_interfaces/srv/StartBagRecording.srv for help on the fields
  ros2 service call /start_bag_recording nakama_franka_interfaces/srv/StartBagRecording "{bag_path: '/home/fr3/nakama_ws/bags/', bag_name: 'recorded_bag', append_timestamp: false}"

  # Perform motion

  # Stop recording
  ros2 service call /stop_bag_recording nakama_franka_interfaces/srv/StopBagRecording 
  ```

### Playback

- Ensure that the controller configuration file ([nakama_franka_controllers/config/nakama_bag_playback_joint_impedance_controller.yaml](../nakama_franka_controllers/config/nakama_bag_playback_joint_impedance_controller.yaml)) is updated to use the correct bag URI.
  - If you changed the controller configuration file, run `colcon build` again to reflect the changes. This is *not* necessary if you built using `colcon build --symlink-install` flag before.

- Open a terminal and execute the following commands

  ```bash
  cd ~/nakama_ws
  source install/setup.bash

  # Move robot to neutral starting position, interrupt with ctrl-C once the motion is done
  ros2 launch franka_bringup move_to_start_example_controller.launch.py arm_id:=fr3 robot_ip:=172.16.0.2

  # Start the recorder, do not interrupt until the recording is stopped in the second terminal
  ros2 launch nakama_franka_bringup bag_playback_joint_impedance_controller.launch.py
  ```
