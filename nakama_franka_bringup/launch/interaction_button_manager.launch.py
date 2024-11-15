from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():  # noqa: ANN201, D103
    robot_ip_parameter_name = "robot_ip"
    arm_id_parameter_name = "arm_id"
    load_gripper_parameter_name = "load_gripper"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    use_rviz_parameter_name = "use_rviz"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    arm_id = LaunchConfiguration(arm_id_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    # HMI experiment launch arguments
    participant_nr_parameter_name = "participant_nr"

    return LaunchDescription(
        [
            # Declare all launch arguments
            DeclareLaunchArgument(
                robot_ip_parameter_name,
                default_value="172.16.0.2",
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                arm_id_parameter_name,
                default_value="fr3",
                description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            ),
            DeclareLaunchArgument(
                use_rviz_parameter_name,
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            DeclareLaunchArgument(
                use_fake_hardware_parameter_name,
                default_value="false",
                description="Use fake hardware",
            ),
            DeclareLaunchArgument(
                fake_sensor_commands_parameter_name,
                default_value="false",
                description=f"Fake sensor commands. Only valid when '{use_fake_hardware_parameter_name}' is true",
            ),
            DeclareLaunchArgument(
                load_gripper_parameter_name,
                default_value="false",
                description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
                "without an end-effector.",
            ),
            # HMI experiment launch arguments
            DeclareLaunchArgument(
                participant_nr_parameter_name,
                description="Which participant number to use.",
            ),
            # Franka bringup
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("nakama_franka_bringup"),
                                "launch",
                                "franka.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    robot_ip_parameter_name: robot_ip,
                    arm_id_parameter_name: arm_id,
                    load_gripper_parameter_name: load_gripper,
                    use_fake_hardware_parameter_name: use_fake_hardware,
                    fake_sensor_commands_parameter_name: fake_sensor_commands,
                    use_rviz_parameter_name: use_rviz,
                }.items(),
            ),
            # Load the playback controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "nakama_bag_playback_joint_impedance_controller",
                    "--inactive",  # Important: load and configure the controller but do not activate
                    "--param-file",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nakama_franka_controllers"),
                            "config",
                            "nakama_bag_playback_joint_impedance_controller.yaml",
                        ]
                    ),
                ],
                output="screen",
            ),
            # Load the Franka gravity compensation example controller (for compliant robot in training phase)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "franka_gravity_compensation_example_controller",
                    "--inactive",  # Important: load and configure the controller but do not activate
                    "--param-file",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nakama_franka_bringup"),
                            "config",
                            "franka_gravity_compensation_example_controller.yaml",
                        ]
                    ),
                ],
                output="screen",
            ),
            # Load the Franka move to start example controller (for homing to hardcoded start position)
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "franka_move_to_start_example_controller",
                    "--inactive",  # Important: load and configure the controller but do not activate
                    "--param-file",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nakama_franka_bringup"),
                            "config",
                            "franka_move_to_start_example_controller.yaml",
                        ]
                    ),
                ],
                output="screen",
            ),
            # Load the joint state bag recorder
            Node(
                package="nakama_franka_bag",
                executable="joint_state_bag_recorder",
                output="screen",
            ),
            # Interaction button node
            Node(
                package="bee_buttons",
                executable="bee_buttons",
                parameters=[
                    {"update_rate": 50.0},
                    {"battery_info_on_startup": False},
                ],
                # Limit log level so not all messages are shown
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            # Bag recorder manager node
            Node(
                package="nakama_franka_interaction_buttons",
                executable="bag_recorder_manager",
                parameters=[
                    {"wait_for_services": False},
                    {"participant_nr": LaunchConfiguration(participant_nr_parameter_name)},
                    {"training_controller": "franka_gravity_compensation_example_controller"},
                    {"playback_controller": "nakama_bag_playback_joint_impedance_controller"},
                    {"homing_controller": "franka_move_to_start_example_controller"},
                ],
            ),
        ]
    )
