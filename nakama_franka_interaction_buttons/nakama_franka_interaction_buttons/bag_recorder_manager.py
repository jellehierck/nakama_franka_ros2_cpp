from enum import auto
from pathlib import Path
import time

import rclpy
import rclpy.parameter
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from example_interfaces.msg import String

from controller_manager_msgs.srv import SwitchController

from bee_buttons_interfaces.msg import BeeButtonColor, BeeButtonPress
from bee_buttons_interfaces.srv import BeeButtonSetColor

from nakama_franka_interfaces.srv import StartBagRecording, StopBagRecording

from .client_wrapper import AsyncClientWrapper
from .strenum import StrEnum


class BagRecorderManagerState(StrEnum):
    """Possible states for the BagRecorderManager."""

    IDLE = auto()

    ERROR = auto()

    OPERATING_IDLE = auto()
    STOPPING_OPERATING = auto()
    STARTING_OPERATING = auto()

    OPERATING_HOMING = auto()
    STOPPING_OPERATING_HOMING = auto()
    STARTING_OPERATING_HOMING = auto()

    REPLAYING = auto()
    STOPPING_REPLAYING = auto()
    STARTING_REPLAYING = auto()

    TRAINING_IDLE = auto()
    STARTING_TRAINING = auto()
    STOPPING_TRAINING = auto()

    TRAINING_HOMING = auto()
    STOPPING_TRAINING_HOMING = auto()
    STARTING_TRAINING_HOMING = auto()

    RECORDING = auto()
    STOPPING_RECORDING = auto()
    STARTING_RECORDING = auto()


class BagRecorderManager(Node):
    """Bag recorder manager node.

    This node will manage controllers and buttons record and play back franka motions.
    """

    def __init__(self) -> None:
        """Initialize the BagRecoderManager node."""
        super().__init__("bag_recorder_manager")

        # Initialize the state
        self._state = BagRecorderManagerState.IDLE

        # Store the recorded bag URIs during the training phase
        # self.recorded_bag_uris: list[str] = []

        # Declare parameters for keeping track of participants and tasks
        self.base_bag_path_param = self.declare_parameter(
            name="base_bag_path",
            value="/home/fr3/nakama_ws/bags/hmi/",
            descriptor=ParameterDescriptor(
                description="Full path to the base of where the recorded bags are stored",
            ),
        )
        self.participant_nr_param = self.declare_parameter(
            name="participant_nr",
            value=1,
            descriptor=ParameterDescriptor(
                description="Participant number",
            ),
        )
        self.curr_task_nr_param = self.declare_parameter(
            name="curr_task_nr",
            value=0,
            descriptor=ParameterDescriptor(
                description="Current task number, used to select a recording and playback bag",
            ),
        )

        # Declare parameters for used controller names
        self.playback_controller_param = self.declare_parameter(
            name="playback_controller",
            value="nakama_bag_playback_joint_impedance_controller",
            descriptor=ParameterDescriptor(
                description="Name of the controller used for playback of recorded tasks",
                read_only=True,  # Only takes effect when the node is started
            ),
        )
        self.homing_controller_param = self.declare_parameter(
            name="homing_controller",
            value="franka_move_to_start_example_controller",
            descriptor=ParameterDescriptor(
                description="Name of the homing controller",
                read_only=True,  # Only takes effect when the node is started
            ),
        )

        # Declare other parameters
        self.wait_for_services_param = self.declare_parameter(
            name="wait_for_services",
            value=True,
            descriptor=ParameterDescriptor(
                description="Whether to wait for required services to become available on startup.",
                read_only=True,  # Only takes effect when the node is started
            ),
        )

        # Instantiate another callback group which will be run on a separate executor, to allow service calls to happen
        # on anothe thread and therefore be synchronous without causing a deadlock
        self.async_service_callback_group = MutuallyExclusiveCallbackGroup()

        # Create service clients to be able to make service requests
        def wait_for_service(client_wrapper: AsyncClientWrapper) -> None:
            """Helper function to wait on the service to become availabe if `wait_for_services` parameter is True."""
            if self.get_parameter(self.wait_for_services_param.name).get_parameter_value().bool_value:
                # Wait for some seconds before logging a new message that the service is unavailable
                while not client_wrapper.client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().info(f"Service {client_wrapper.client.srv_name} not available, waiting...")

        # Relevant parameter services
        self.get_playback_controller_parameters_service = AsyncClientWrapper(
            self.create_client(
                GetParameters,
                srv_name=f"{self.playback_controller}/get_parameters",
                callback_group=self.async_service_callback_group,
            )
        )

        self.set_playback_controller_parameters_service = AsyncClientWrapper(
            self.create_client(
                SetParameters,
                srv_name=f"{self.playback_controller}/set_parameters",
                callback_group=self.async_service_callback_group,
            )
        )

        self.get_homing_controller_parameters_service = AsyncClientWrapper(
            self.create_client(
                GetParameters,
                srv_name=f"{self.homing_controller}/get_parameters",
                callback_group=self.async_service_callback_group,
            )
        )

        self.set_homing_controller_parameters_service = AsyncClientWrapper(
            self.create_client(
                SetParameters,
                srv_name=f"{self.homing_controller}/set_parameters",
                callback_group=self.async_service_callback_group,
            )
        )

        # Controller manager services
        self.switch_controller_service = AsyncClientWrapper(
            self.create_client(
                SwitchController,
                srv_name="controller_manager/switch_controller",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.switch_controller_service)

        # Bag recorder services
        self.start_recording_service = AsyncClientWrapper(
            self.create_client(
                StartBagRecording,
                srv_name="joint_state_bag_recorder/start_recording",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.start_recording_service)

        self.stop_recording_service = AsyncClientWrapper(
            self.create_client(
                StopBagRecording,
                srv_name="joint_state_bag_recorder/stop_recording",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.stop_recording_service)

        # Button color services
        self.set_button_color_service = AsyncClientWrapper(
            self.create_client(
                BeeButtonSetColor,
                srv_name="bee_buttons/set_button_color",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.set_button_color_service)

        # Create subscriptions
        self.button_press_subscription = self.create_subscription(
            BeeButtonPress, "button_press", self.handle_button_press, qos_profile=qos_profile_system_default
        )

        # Create publishers
        self.action_log_publisher = self.create_publisher(String, "/action_log", qos_profile=qos_profile_system_default)

    @property
    def state(self) -> BagRecorderManagerState:
        """Get the current state."""
        return self._state

    @state.setter
    def state(self, new_state: BagRecorderManagerState) -> None:
        """Set a new state. This will also change the interaction button color depending on the new state."""
        self._state = new_state
        self.set_button_color_from_state(self._state)
        self.action_log_publisher.publish(String(data=f"Changed to state: {self._state}"))
        self.get_logger().info(f"Changed to state: {self._state}")

    @property
    def playback_controller(self) -> str:
        return self.get_parameter(self.playback_controller_param.name).get_parameter_value().string_value

    @property
    def homing_controller(self) -> str:
        return self.get_parameter(self.homing_controller_param.name).get_parameter_value().string_value

    @property
    def participant_nr(self) -> int:
        return self.get_parameter(self.participant_nr_param.name).get_parameter_value().integer_value

    @property
    def curr_task_nr(self) -> int:
        return self.get_parameter(self.curr_task_nr_param.name).get_parameter_value().integer_value

    @curr_task_nr.setter
    def curr_task_nr(self, new_nr: int) -> None:
        self.set_parameters(
            [rclpy.parameter.Parameter(self.curr_task_nr_param.name, self.curr_task_nr_param.type_, new_nr)]
        )
        self.action_log_publisher.publish(String(data=f"Set current task to {self.curr_task_nr}"))
        self.get_logger().info(f"Set current task to {self.curr_task_nr}")

    @property
    def base_bag_path(self) -> str:
        return self.get_parameter(self.base_bag_path_param.name).get_parameter_value().string_value

    def curr_participant_task_to_bag_name(self) -> str:
        return f"participant_{self.participant_nr:03}/task_{self.curr_task_nr:03}"

    def set_button_color_from_state(self, state: BagRecorderManagerState) -> None:
        """Send a button color effect request based on a state."""
        if state == BagRecorderManagerState.ERROR:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_FULL,
                    color=BeeButtonColor(value=BeeButtonColor.RED),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif state == BagRecorderManagerState.IDLE:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_FULL,
                    color=BeeButtonColor(value=BeeButtonColor.GREEN),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif state == BagRecorderManagerState.TRAINING_IDLE:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_FULL,
                    color=BeeButtonColor(value=BeeButtonColor.BLUE),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif state == BagRecorderManagerState.RECORDING:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_FULL,
                    color=BeeButtonColor(value=BeeButtonColor.PURPLE),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif state == BagRecorderManagerState.OPERATING_IDLE:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_FULL,
                    color=BeeButtonColor(value=BeeButtonColor.YELLOW),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif state == BagRecorderManagerState.REPLAYING:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_BLINK,
                    color=BeeButtonColor(value=BeeButtonColor.ORANGE),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif state == BagRecorderManagerState.TRAINING_HOMING or state == BagRecorderManagerState.OPERATING_HOMING:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_BLINK,
                    color=BeeButtonColor(value=BeeButtonColor.PINK),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif str(state).lower().startswith("starting") or str(state).lower().startswith("stopping"):
            # All transition states will show the same color effect
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    effect=BeeButtonSetColor.Request.EFFECT_BLINK,
                    color=BeeButtonColor(value=BeeButtonColor.CYAN),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        else:
            self.get_logger().warn(f"Cannot set button color for state {state}")

    def handle_button_press(self, msg: BeeButtonPress) -> None:
        """Handle incoming button press messages.

        This will trigger state transitions based on the press type and current state.
        """
        self.action_log_publisher.publish(String(data=f"Received button press: {msg.press_type}"))

        # Define which specific button press types are handled in specific states
        if self.state == BagRecorderManagerState.IDLE:
            if msg.press_type == BeeButtonPress.LONG_PRESS:
                self.start_training()
                return

            if msg.press_type == BeeButtonPress.DOUBLE_PRESSED:
                self.start_operating()
                return

        # Training states
        elif self.state == BagRecorderManagerState.TRAINING_IDLE:
            if msg.press_type == BeeButtonPress.PRESSED:
                self.start_recording()
                return

            elif msg.press_type == BeeButtonPress.LONG_PRESS:  # noqa: RET505
                self.stop_training()
                return

            elif msg.press_type == BeeButtonPress.DOUBLE_PRESSED:  # noqa: RET505
                self.start_training_homing()
                return

        elif self.state == BagRecorderManagerState.RECORDING:  # noqa: SIM102
            if msg.press_type == BeeButtonPress.PRESSED:
                self.stop_recording()
                return

        # Operating states
        elif self.state == BagRecorderManagerState.OPERATING_IDLE:
            if msg.press_type == BeeButtonPress.PRESSED:
                self.start_replaying()
                return

            elif msg.press_type == BeeButtonPress.LONG_PRESS:  # noqa: RET505
                self.stop_operating()
                return

            elif msg.press_type == BeeButtonPress.DOUBLE_PRESSED:  # noqa: RET505
                self.start_operating_homing()
                return

        # The current combination of state and button press type has no effect
        self.get_logger().warn(f"Button press type {msg.press_type} has no effect in state {self._state}")

    def switch_controllers_sync_and_wait(
        self, *, activate_controllers: list[str] | None = None, deactivate_controllers: list[str] | None = None
    ) -> SwitchController.Response | None:
        """Switch controllers.

        It is possible to only activate or deactivate controllers by providing only a single input list. If both are
        provided, the controllers are switched.

        :return: `SwitchController.Response` Response of the service request.
        :return: `None` if the service request was cancelled or timed out.
        """
        if activate_controllers is None:
            activate_controllers = []
        if deactivate_controllers is None:
            deactivate_controllers = []

        return self.switch_controller_service.call_sync_and_await(
            SwitchController.Request(
                activate_controllers=activate_controllers,
                deactivate_controllers=deactivate_controllers,
                strictness=SwitchController.Request.STRICT,
                activate_asap=True,
                timeout=Duration(sec=1),
            )
        )

    def start_training(self) -> None:
        """Enter the `STARTING_TRAINING` transition state.

        Possible next states:
        - Always transitions to `TRAINING_IDLE`
        """
        self.state = BagRecorderManagerState.STARTING_TRAINING

        # Always moves into TRAINING_IDLE immediately
        self.state = BagRecorderManagerState.TRAINING_IDLE


    def stop_training(self) -> None:
        """Enter the `STOPPING_TRAINING` transition state.

        Possible next states:
        - Always transitions to `IDLE`
        """
        self.state = BagRecorderManagerState.STOPPING_TRAINING

        # Reset the current task number to zero as we might want to play back or record from the beginning
        self.curr_task_nr = 0

        # Always move into IDLE immediately
        self.state = BagRecorderManagerState.IDLE

    def start_recording(self) -> None:
        """Enter the `STARTING_RECORDING` transition state.

        Possible next states:
        - `StartBagRecording` response indicated success: `RECORDING`
        - `StartBagRecording` response indicated failure: `ERROR`
        - A bag already exists for this task and participant: `TRAINING_IDLE`
        """
        self.state = BagRecorderManagerState.STARTING_RECORDING

        # First check if the path already exists
        full_bag_path = Path(self.base_bag_path + self.curr_participant_task_to_bag_name())
        if full_bag_path.exists():
            self.get_logger().warn(f"Cannot start recording as task bag already exists ({full_bag_path})")
            self.state = BagRecorderManagerState.TRAINING_IDLE
            return

        response: StartBagRecording.Response | None = self.start_recording_service.call_sync_and_await(
            StartBagRecording.Request(
                bag_path=self.base_bag_path,
                bag_name=self.curr_participant_task_to_bag_name(),
                append_timestamp=False,
            )
        )

        if response is None or not response.started:
            self.get_logger().error("Could not start recording")
            self.state = BagRecorderManagerState.ERROR

        else:
            # Store the recorded bag URI in the list of trainings
            # TODO: Remove this if not needed anymore
            # self.recorded_bag_uris.append(response.opened_bag_uri)
            self.state = BagRecorderManagerState.RECORDING

    def stop_recording(self) -> None:
        """Enter the `STOPPING_RECORDING` transition state.

        Possible next states:
        - `StopBagRecording` response indicated success: `TRAINING_IDLE`
        - `StopBagRecording` response indicated failure: `ERROR`
        """
        self.state = BagRecorderManagerState.STOPPING_RECORDING

        response: StopBagRecording.Response | None = self.stop_recording_service.call_sync_and_await(
            StopBagRecording.Request()
        )

        if response is None:
            self.get_logger().error("Could not stop recording")
            self.state = BagRecorderManagerState.ERROR

        else:
            # Increment the current task number to keep track of how many tasks are recorded
            self.curr_task_nr += 1
            self.state = BagRecorderManagerState.TRAINING_IDLE

    def start_operating_homing(self) -> None:
        """Enter the `OPERATING_HOMING` timed state.

        This will move through the `STARTING_OPERATING_HOMING`, `OPERATING_HOMING` and `STOPPING_OPERATING_HOMING`
        states.

        Possible next states:
        - Homing is finished and successful: `OPERATING_IDLE`
        - Any error occurred: `ERROR`
        """
        # Activate the homing controller
        self.state = BagRecorderManagerState.STARTING_OPERATING_HOMING
        activate_homing_controller_response = self.switch_controllers_sync_and_wait(
            activate_controllers=[self.homing_controller],
        )

        # If an error occurred, go into the error state and return early
        if activate_homing_controller_response is None or not activate_homing_controller_response.ok:
            self.get_logger().error("Error during homing: could not activate homing controller")
            self.state = BagRecorderManagerState.ERROR
            return

        # Wait for the homing to finish
        self.state = BagRecorderManagerState.OPERATING_HOMING
        process_finished = self.wait_for_controller_process_finished(self.get_homing_controller_parameters_service)
        if not process_finished:
            self.get_logger().error("Error during homing: homing was not successful")
            self.state = BagRecorderManagerState.ERROR

        # Create a service request to reset the process finished flag
        # TODO: This is currently a bug in `franka_example_controllers/MoveToStartExampleController`, where parameter
        # `"process_finished"` is not reset when the controller is reactivated
        self.set_homing_controller_parameters_service.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="process_finished",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_BOOL,
                            bool_value=False,
                        ),
                    )
                ]
            ),
            done_callback=None,  # We do not wait for a response
        )

        # Deactivate the homing controller and activate the training controller
        self.state = BagRecorderManagerState.STOPPING_OPERATING_HOMING
        deactivate_homing_controller_response = self.switch_controllers_sync_and_wait(
            deactivate_controllers=[self.homing_controller],
        )

        # If an error occurred, go into the error state and return early
        if deactivate_homing_controller_response is None or not deactivate_homing_controller_response.ok:
            self.get_logger().error("Error during homing: could not deactivate homing controller")
            self.state = BagRecorderManagerState.ERROR
            return

        # All is OK, we can now move back into the operating state
        self.state = BagRecorderManagerState.OPERATING_IDLE

    def start_training_homing(self) -> None:
        """Enter the `TRAINING_HOMING` timed state.

        This will move through the `STARTING_TRAINING_HOMING`, `TRAINING_HOMING` and `STOPPING_TRAINING_HOMING` states.

        Possible next states:
        - Homing is finished and successful: `TRAINING_IDLE`
        - Any error occurred: `ERROR`
        """
        # Switch the homing controller in
        self.state = BagRecorderManagerState.STARTING_TRAINING_HOMING
        activate_homing_controller_response = self.switch_controllers_sync_and_wait(
            activate_controllers=[self.homing_controller],
        )

        # If an error occurred, go into the error state and return early
        if activate_homing_controller_response is None or not activate_homing_controller_response.ok:
            self.get_logger().error("Error during homing: could not activate homing controller")
            self.state = BagRecorderManagerState.ERROR
            return

        # Wait for the homing to finish
        self.state = BagRecorderManagerState.TRAINING_HOMING
        process_finished = self.wait_for_controller_process_finished(self.get_homing_controller_parameters_service)
        if not process_finished:
            self.get_logger().error("Error during homing: homing was not successful")
            self.state = BagRecorderManagerState.ERROR

        # Create a service request to reset the process finished flag
        # TODO: This is currently a bug in `franka_example_controllers/MoveToStartExampleController`, where parameter
        # `"process_finished"` is not reset when the controller is reactivated
        self.set_homing_controller_parameters_service.call_async(
            SetParameters.Request(
                parameters=[
                    Parameter(
                        name="process_finished",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_BOOL,
                            bool_value=False,
                        ),
                    )
                ]
            ),
            done_callback=None,  # We do not wait for a response
        )

        # Switch the homing controller out
        self.state = BagRecorderManagerState.STOPPING_TRAINING_HOMING
        deactivate_homing_controller_response = self.switch_controllers_sync_and_wait(
            deactivate_controllers=[self.homing_controller],
        )

        # If an error occurred, go into the error state and return early
        if deactivate_homing_controller_response is None or not deactivate_homing_controller_response.ok:
            self.get_logger().error("Error during homing: could not deactivate homing controller")
            self.state = BagRecorderManagerState.ERROR
            return

        # All is OK, we can now move back into the training state
        self.state = BagRecorderManagerState.TRAINING_IDLE

    @staticmethod
    def wait_for_controller_process_finished(get_controller_parameters_service: AsyncClientWrapper) -> bool:
        """Wait for a controller to set its `"process_finished"` parameter.

        :param get_controller_parameters_service: The `GetParameters` service to use to make the parameter request.
        :return: `True` if the homing is finished.
        :return: `False` if an error occurred.
        """
        # Check whether the homing process is finished by polling the parameter flag on the move to start controller
        while True:
            homing_finished_response: GetParameters.Response | None = (
                get_controller_parameters_service.call_sync_and_await(
                    GetParameters.Request(
                        names=["process_finished"],
                    )
                )
            )
            # Check if the response is OK
            if (
                homing_finished_response is None
                or not homing_finished_response.values
                or homing_finished_response.values[0].type != ParameterType.PARAMETER_BOOL
            ):
                return False  # Indicate failure

            # Check if the response indicated that the controller is finished
            if homing_finished_response.values[0].bool_value is True:
                return True  # Indicate success

            # If not finished, sleep for a short time before trying again
            time.sleep(0.2)

    def start_operating(self) -> None:
        """Enter the `STARTING_OPERATING` transition state.

        Possible next states:
        - Always transitions to: `OPERATING_IDLE`
        """
        self.state = BagRecorderManagerState.STARTING_OPERATING
        # Transition to the next state immediately
        self.state = BagRecorderManagerState.OPERATING_IDLE

    def stop_operating(self) -> None:
        """Enter the `STOPPING_OPERATING` transition state.

        Possible next states:
        - Always transitions to: `IDLE`
        """
        self.state = BagRecorderManagerState.STOPPING_OPERATING

        # Reset the current task number to zero as we might want to play back or record from the beginning
        self.curr_task_nr = 0

        # Transition to the next state immediately
        self.state = BagRecorderManagerState.IDLE

    def start_replaying(self) -> None:
        """Enter the `REPLAYING` timed state.

        This will move through the `STARTING_REPLAYING`, `REPLAYING` and `STOPPING_REPLAYING` states.

        Possible next states:
        - Playback is finished and successful: `OPERATING_IDLE`
        - Any error occurred: `ERROR`
        - A bag does not exist for this task and participant: `OPERATING_IDLE`
        """
        self.state = BagRecorderManagerState.STARTING_REPLAYING

        # Check if we can play back the next recording
        # TODO: Remove this if not needed anymore
        # if self.nr_recordings_played_back >= len(self.recorded_bag_uris):
        #     self.get_logger().error("There is no new recording to play back!")
        #     self.state = BagRecorderManagerState.ERROR
        #     return

        # First check if the path already exists
        full_bag_path = Path(self.base_bag_path + self.curr_participant_task_to_bag_name())
        if not full_bag_path.exists():
            self.get_logger().warn(f"Cannot start playback as task bag does not exist ({full_bag_path})")
            self.state = BagRecorderManagerState.OPERATING_IDLE
            return

        # Create a service request to change the used bag URI for the playback controller before the controller starts
        bag_uri_response: SetParameters.Response | None = (
            self.set_playback_controller_parameters_service.call_sync_and_await(
                SetParameters.Request(
                    parameters=[
                        Parameter(
                            name="bag_uri",
                            value=ParameterValue(
                                type=ParameterType.PARAMETER_STRING,
                                # TODO: Remove this if not needed anymore
                                # string_value=self.recorded_bag_uris[self.nr_recordings_played_back],
                                string_value=self.base_bag_path + self.curr_participant_task_to_bag_name(),
                            ),
                        )
                    ]
                )
            )
        )

        # Check that the new bag URI is set successfully
        if bag_uri_response is None or not all(result.successful for result in bag_uri_response.results):
            self.get_logger().error("Could not update the bag URI parameter.")
            self.state = BagRecorderManagerState.ERROR
            return

        # Now we can activate the playback controller
        activate_playback_controller_response: SwitchController.Response | None = self.switch_controllers_sync_and_wait(
            activate_controllers=[self.playback_controller],
        )

        # If an error occurred, go into the error state and return early
        if activate_playback_controller_response is None or not activate_playback_controller_response.ok:
            self.get_logger().error("Error during playback: could not activate playback controller")
            self.state = BagRecorderManagerState.ERROR
            return

        # Wait for the playback to finish
        self.state = BagRecorderManagerState.REPLAYING
        playback_finished = self.wait_for_controller_process_finished(self.get_playback_controller_parameters_service)
        if not playback_finished:
            self.get_logger().error("Error during playback: playback was not successful")
            self.state = BagRecorderManagerState.ERROR

        # Deactivate the playback controller
        self.state = BagRecorderManagerState.STOPPING_REPLAYING
        deactivate_playback_controller_response = self.switch_controllers_sync_and_wait(
            deactivate_controllers=[self.playback_controller],
        )

        # If an error occurred, go into the error state and return early
        if deactivate_playback_controller_response is None or not deactivate_playback_controller_response.ok:
            self.get_logger().error("Error during homing: could not deactivate homing controller")
            self.state = BagRecorderManagerState.ERROR
            return

        # Increment the current task number to keep track of how many tasks are played back
        self.curr_task_nr += 1

        # All is OK, we can now move back into the operating state
        self.start_operating()


def main(args: list[str] | None = None) -> None:
    """Start the node."""
    try:
        rclpy.init(args=args)
        bag_recorder_manager_node = BagRecorderManager()

        # Define a multithreaded executor to perform some callbacks in parallel.
        # In particular, we want to execute service calls parallel to all other callbacks.
        # We use the principle that any service call should be parallel to all other callbacks, but never to itself.
        # The other callbacks should also not be parallel to each other.
        # To achieve this, we add service calls to one mutually exclusive callback group and all other callbacks to
        # another mutually exclusive callback group.
        mt_executor = MultiThreadedExecutor(num_threads=2)

        rclpy.spin(bag_recorder_manager_node, executor=mt_executor)

    except KeyboardInterrupt:
        # Keyboard interrupts cause the program to stop gracefully
        pass

    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
