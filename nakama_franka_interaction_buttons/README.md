
## HMI experiment starting point:

```bash
cd ~/nakama_ws
source install/setup.bash
ros2 launch nakama_franka_bringup interaction_button_manager.launch.py participant_nr:=1
```

### Re-recording tasks

1. Delete the task folders for the participant who wants to re-record their tasks.
   - The task folders are located under `~/nakama_ws/bags/hmi`.
   - E.g. for participant 1, you need to delete all folders called `~/nakama_ws/bags/hmi/participant_001/task_*`.
   - (recommended) It is safest to do so from the file explorer, as the files can be restored from the bin afterwards.
   - (not recommended) `rm -rf ~/nakama_ws/bags/hmi/participant_001/task_*` **Make sure that you have selected the correct participant, there is no undo**.
2. Move the robot pose into the `training` state.
3. Let the robot do its `homing` sequence from the `training` state.
4. Now the participant can re-record their tasks.


### Opening / closing gripper

Opening:

```bash
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.05, speed: 0.03, force: 50}"
```

Closing:

```bash
ros2 action send_goal -f /fr3_gripper/grasp franka_msgs/action/Grasp "{width: 0.00, speed: 0.03, force: 50}"
```

### Manual button press commands from CLI

Single press:

```bash
ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'PRESSED'  }"
```

Double press:

```bash
ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }"
```

Long press:

```bash
ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'LONG_PRESS'  }"
```

### State machine

Detailed on the left (with all transition states), simple on the right (without transition states).

![](./state%20machine.drawio.png)

---

## Manual launch files


### Gravity compensation controller

```bash
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py load_gripper:=true robot_ip:=172.16.0.2
```

### Move to start controller

```bash
ros2 launch franka_bringup move_to_start_example_controller.launch.py load_gripper:=true robot_ip:=172.16.0.2
```

### Playback controller

Launch all nodes (terminal 1):

```bash
cd ~/nakama_ws
source install/setup.bash
ros2 launch nakama_franka_bringup bag_playback_joint_impedance_controller.launch.py load_gripper:=true
```

Set bag URI to play back (terminal 2):
 
```bash
ros2 param set /nakama_bag_playback_joint_impedance_controller bag_uri "/home/fr3/nakama_ws/bags/hmi/participant_001/task_000"
```

Activate playback controller (terminal 2):
 
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{ activate_controllers: ['nakama_bag_playback_joint_impedance_controller'] , strictness: 2 }"
```

Deactivate playback controller (terminal 2):
 
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{ deactivate_controllers: ['nakama_bag_playback_joint_impedance_controller'], strictness: 2 }"
```