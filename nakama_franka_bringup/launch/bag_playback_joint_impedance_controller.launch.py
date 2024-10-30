from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
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
                default_value="true",
                description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
                "without an end-effector.",
            ),
            # Franka bringup
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("nakama_franka_bringup"), "launch", "franka.launch.py"])]
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
            # Spawn the controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "nakama_bag_playback_joint_impedance_controller",
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
        ]
    )
