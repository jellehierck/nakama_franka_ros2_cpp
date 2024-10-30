# Nakama Franka Bag tools


## Usage

### Recording

Terminal 1:

```bash
cd ~/nakama_ws
source install/setup.bash
ros2 launch nakama_franka_bringup franka_bag_recorder.launch.py
```

Terminal 2:

```bash
cd ~/nakama_ws
source install/setup.bash

# Start recording
ros2 service call /start_bag_recording nakama_franka_interfaces/srv/StartBagRecording "{bag_path: '/home/fr3/nakama_ws/bags/', bag_name: 'recorded_bag', append_timestamp: false}"

# Perform motion

# Stop recording
ros2 service call /stop_bag_recording nakama_franka_interfaces/srv/StopBagRecording 
```

### Playback

Move robot very close to the same starting position as the movement (automatic homing is not implemented yet).

```bash
cd ~/nakama_ws
source install/setup.bash
ros2 launch nakama_franka_bringup bag_playback_joint_impedance_controller.launch.py
```