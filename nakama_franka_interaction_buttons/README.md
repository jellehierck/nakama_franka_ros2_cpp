
## HMI experiment starting point:

```bash
cd ~/nakama_ws
source install/setup.bash
ros2 launch nakama_franka_bringup interaction_button_manager.launch.py participant_nr:=1
```

---

## Manual launch files

### Move to start controller

```bash
ros2 launch franka_bringup move_to_start_example_controller.launch.py load_gripper:=false robot_ip:=172.16.0.2
```

### Playback controller

Launch all nodes (terminal 1):

```bash
cd ~/nakama_ws
source install/setup.bash
ros2 launch nakama_franka_bringup bag_playback_joint_impedance_controller.launch.py load_gripper:=false
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