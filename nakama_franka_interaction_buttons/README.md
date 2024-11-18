# LfD Social Aspects study

HMI experiment in November/December 2024 which is operated by Jelle Hierck.

## Experiment procedure

### Start of day preparation

1. Set up FCI Desk:
   1. Log in as NakamaFrankaAdmin (pwd: regular password + first half of serial number of robot).
   2. Go to `SETTINGS > End Effector` and select `HMI-Basket` from dropdown list.
   3. Go to `DESK` and enable FCI.
2. Ensure camera tripods, trays and baskets are in correct place.
3. Start up HMI laptop with forms.
4. Check enough consent forms.

### Session 1 (training phase)

1. Set up equipment:

   1. Check that all fruits and vegetables are in the correct place.
   2. Open robot joints.
   3. Start launch file:

      ```bash
      cd ~/nakama_ws
      source install/setup.bash
      ros2 launch nakama_franka_bringup interaction_button_manager.launch.py participant_nr:=  # Insert number here
      ```

   4. Put robot in Automatic Mode (enable button DOWN)
   5. Home the robot (in 2nd terminal)

      ```bash
      cd ~/nakama_ws
      source install/setup.bash
      ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }" --once
      ```

2. Welcome for participant (5 minutes):

   1. Explain procedure.
      1. Fill out consent form.
      2. In session 1 you will mess around with the robot, then teach some tasks, and finally we will check if the tasks you taught are okay.
      3. In session 2 you will perform the task with the robot as you taught it youself and as someone else taught it.
      4. You will fill out some Google Forms during the sessions to capture your experience.
   2. Safety information.
      1. Participant can stop experiment at any time.
      2. Operator controls safety button.
      3. Operator will notify when robot will move automatically.
      4. Everything is under operator supervision.
   3. Let participant fill out consent form **2 times**.

3. Let user get familiar with training mode (10-15 minutes):

   1. Put robot in User Mode (enable button UP)
   2. Let the user play around with the robot (~5 minutes).
      - If participant wants to home again, repeat homing steps:
        - Enable User Mode (button DOWN)
        - Start homing: `ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }" --once`
        - Disable User Mode (button DOWN)
   3. If playing is over, record two segments of a task and stop recording in between.
      - The user can then move the robot as they want.
      - Once the interaction button is pressed, the recording start immediately. So waiting before starting the robot will also reflect in the recording.
      - Pausing a sequence can be done by pressing the interaction button again.
      - It is **important to only move the robot when recording**. Moving it in between recordings will cause a position difference that cannot be played back.
   4. Play back recording to let the user understand the "pause" they created when they stopped the recording in between the segments.
      - User Mode button DOWN
      - Move to Operating state: `ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'LONG_PRESS'  }" --once`
      - Let user press interaction button to start playback and experience the pause.
   5. Let user put their hand in front of robot while playing back to experience compliance.
   6. Delete the recorded tasks according to [Re-recording tasks](#re-recording-tasks).

4. Once the user is ready to record the task, start actual recording procedure (~5 minutes):

   1. Check if vegetables are put back.
   2. Home the robot using the homing steps.
   3. Let the user press the interaction button to start recording a sequence.

5. When user is done recording, show the playback (5-10 minutes, + 5-10 if re-recording):
   1. Move to OPERATING mode: `ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'LONG_PRESS'  }" --once`
   2. Put robot in Automatic Mode (enable button DOWN)
   3. Home the robot: `ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }" --once`
   4. The participant can now play back motions sequences by pressing the interaction button once.

      - If a user is unhappy with the playback of the tasks, remove the task sequences with instructions in [Re-recording tasks](#re-recording-tasks) (max 10 minutes).
      - To repeat playback sequence:
        - Move into TRAINING state: `ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'LONG_PRESS'  }" --once`
        - Repeat all steps above:

6. Finalize experiment for participant (5 minutes):
   1. Let them fill out form for session 1.
   2. Plan a session 2.

7. Shut down equipment:
   1. Stop the terminal with the launch file.
   2. Lock the robot joints.

8. Final steps:
   1. Store recorded task bags and session logging bags to Teams as backup.

### Session 2 (tasking phase)

1. Welcome for participant

   1. Explain procedure... TODO

2. Set up equipment

   1. Start both cameras, clap twice for audio and video synchronization.
   2. In FCI desk:
      1. log in as NakamaFrankaAdmin (pwd: regular password + first half of serial number of robot).
      2. Go to `SETTINGS > End Effector` and select `HMI-Basket` from dropdown list.
      3. Go to `DESK` and enable FCI.

   3. Start launch file:

      ```bash
      cd ~/nakama_ws
      source install/setup.bash
      ros2 launch nakama_franka_bringup interaction_button_manager.launch.py participant_nr:=  # Insert number here
      ```

   4. Put robot in Automatic Mode (enable button DOWN)
   5. Home the robot (in 2nd terminal)

      ```bash
      cd ~/nakama_ws
      source install/setup.bash
      ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }" --once
      ```

3. Show the playback procedure
   1. Move to OPERATING mode

      ```bash
      ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'LONG_PRESS'  }" --once
      ```

   2. Put robot in Automatic Mode (enable button DOWN)
   3. Home the robot

      ```bash
      ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }" --once
      ```

   4. The participant can now play back motions sequences by pressing the interaction button once.
   5. DO NOT ACTUALLY DO TASK

4. Shut down equipment
   1. Stop both cameras
   2. Stop the terminal with the launch file
   3. Lock the robot joints

5. Finalize experiment for participant
   1. Let them fill out form for session 1
   2. Plan a session 2
   3. Ask for feedback points

6. Final steps
   1. Remove recordings from camera and store on disk
   2. Store recorded task bags and session logging bags to disk

## HMI experiment starting point

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
ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'PRESSED'  }" --once
```

Double press:

```bash
ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'DOUBLE_PRESSED'  }" --once
```

Long press:

```bash
ros2 topic pub /button_press bee_buttons_interfaces/msg/BeeButtonPress "{ press_type: 'LONG_PRESS'  }" --once
```

### State machine

Detailed on the left (with all transition states), simple on the right (without transition states).

![state machine](./state%20machine.drawio.png)

---

## Manual launch files

### Gravity compensation controller

```bash
ros2 launch franka_bringup gravity_compensation_example_controller.launch.py load_gripper:=false robot_ip:=172.16.0.2
```

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
