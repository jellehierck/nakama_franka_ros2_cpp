from enum import auto

import rclpy
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.task import Future

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

    REPLAYING = auto()
    STOPPING_REPLAYING = auto()
    STARTING_REPLAYING = auto()

    TRAINING_IDLE = auto()
    STARTING_TRAINING = auto()
    STOPPING_TRAINING = auto()

    RECORDING = auto()
    STOPPING_RECORDING = auto()
    STARTING_RECORDING = auto()

    TRAINING_HOMING = auto()
    STOPPING_HOMING = auto()
    STARTING_HOMING = auto()


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
        self.recorded_bag_uris: list[str] = []

        # Keep track of how many recordings were played back
        self.nr_recordings_played_back = 0

        # Declare parameters
        # TODO: Add parameter for playback controller name
        # self.recorder_controller_param = self.declare_parameter(
        #     name="recorder_controller",
        #     value="nakama_bag_recorder_controller",
        #     descriptor=ParameterDescriptor(
        #         description="Name of the recorder controller to be loaded into the controller manager",
        #     ),
        # )

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
            if self.wait_for_services_param.get_parameter_value().bool_value:
                # Wait for some seconds before logging a new message that the service is unavailable
                while not client_wrapper.client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().info(f"Service {client_wrapper.client.srv_name} not available, waiting...")

        # Set parameter service
        # TODO: Update this to use a synchronous call instead? Or remove the hardcoded controller name
        self.set_parameters_service = AsyncClientWrapper(
            self.create_client(
                SetParameters,
                srv_name="/nakama_bag_playback_joint_impedance_controller/set_parameters",
                callback_group=self.async_service_callback_group,
            )
        )
        # We do not wait for the set_parameters_service

        # Controller manager services
        self.switch_controller_service = AsyncClientWrapper(
            self.create_client(
                SwitchController,
                srv_name="/controller_manager/switch_controller",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.switch_controller_service)

        # Bag recorder services
        self.start_recording_service = AsyncClientWrapper(
            self.create_client(
                StartBagRecording,
                srv_name="/joint_state_bag_recorder/start_recording",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.start_recording_service)

        self.stop_recording_service = AsyncClientWrapper(
            self.create_client(
                StopBagRecording,
                srv_name="/joint_state_bag_recorder/stop_recording",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.stop_recording_service)

        # Button color services
        self.set_button_color_service = AsyncClientWrapper(
            self.create_client(
                BeeButtonSetColor,
                srv_name="/bee_buttons/set_button_color",
                callback_group=self.async_service_callback_group,
            )
        )
        wait_for_service(self.set_button_color_service)

        # Create subscriptions
        self.button_press_subscription = self.create_subscription(
            BeeButtonPress, "button_press", self.handle_button_press, qos_profile=qos_profile_system_default
        )

    @property
    def state(self) -> BagRecorderManagerState:
        """Get the current state."""
        return self._state

    @state.setter
    def state(self, new_state: BagRecorderManagerState) -> None:
        """Set a new state. This will also change the interaction button color depending on the new state."""
        self._state = new_state
        self.get_logger().info(f"Changed to state: {self._state}")

    def set_button_color_from_state(self, state: BagRecorderManagerState) -> None:
        """Send a button color effect request based on a state."""
        if state == BagRecorderManagerState.ERROR:
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    node_ids=[],  # TODO: Target specific button?
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
                    node_ids=[],  # TODO: Target specific button?
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
                    node_ids=[],  # TODO: Target specific button?
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
                    node_ids=[],  # TODO: Target specific button?
                    effect=BeeButtonSetColor.Request.EFFECT_FULL,
                    color=BeeButtonColor(value=BeeButtonColor.PURPLE),
                    brightness=BeeButtonSetColor.Request.BRIGHTNESS_MAX,
                ),
                done_callback=None,  # We do not care about the result of setting the color
            )

        elif str(state).startswith("starting") or str(state).startswith("stopping"):
            # All transition states will show the same color effect
            self.set_button_color_service.call_async(
                request=BeeButtonSetColor.Request(
                    broadcast=True,
                    node_ids=[],  # TODO: Target specific button?
                    effect=BeeButtonSetColor.Request.EFFECT_BLINK,
                    color=BeeButtonColor(value=BeeButtonColor.YELLOW),
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
        # Check that the button press comes from the correct button
        # TODO: Make node ID configurable
        # if msg.node_id != "443":
        #     self.get_logger().warn(f"Got button press from button {msg.node_id} which this node does not listen to")
        #     return

        # Define which specific button press types are handled in specific states
        if self.state == BagRecorderManagerState.IDLE:
            if msg.press_type == BeeButtonPress.LONG_PRESS:
                self.start_training()
                return

            if msg.press_type == BeeButtonPress.DOUBLE_PRESSED:
                self.start_operating()
                return

        # Training states
        if self.state == BagRecorderManagerState.TRAINING_IDLE:
            if msg.press_type == BeeButtonPress.PRESSED:
                self.start_recording()
                return

            elif msg.press_type == BeeButtonPress.LONG_PRESS:  # noqa: RET505
                self.stop_training()
                return

        if self.state == BagRecorderManagerState.RECORDING:  # noqa: SIM102
            if msg.press_type == BeeButtonPress.PRESSED:
                self.stop_recording()
                return

        # Operating states
        if self.state == BagRecorderManagerState.OPERATING_IDLE:
            if msg.press_type == BeeButtonPress.PRESSED:
                self.start_replaying()
                return

            elif msg.press_type == BeeButtonPress.LONG_PRESS:  # noqa: RET505
                self.stop_operating()
                return

        if self.state == BagRecorderManagerState.REPLAYING:  # noqa: SIM102
            if msg.press_type == BeeButtonPress.PRESSED:
                self.stop_replaying()
                return

        # The current combination of state and button press type has no effect
        self.get_logger().warn(f"Button press type {msg.press_type} has no effect in state {self._state}")

    def start_training(self) -> None:
        """Enter the `STARTING_TRAINING` transition state.

        Possible next states:
        - `SwitchController` response indicated success: `TRAINING_IDLE`
        - `SwitchController` response indicated failure: `ERROR`
        """
        self.state = BagRecorderManagerState.STARTING_TRAINING
        request = SwitchController.Request(
            # TODO: Make controller name configurable
            activate_controllers=["franka_gravity_compensation_example_controller"],
            deactivate_controllers=[],
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(sec=1),
        )

        response = self.switch_controller_service.call_sync_and_await(request)

        if response is None or not response.ok:
            self.get_logger().error(f"Error while transitioning to state {BagRecorderManagerState.TRAINING_IDLE}")
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.TRAINING_IDLE

    def stop_training(self) -> None:
        """Enter the `STOPPING_TRAINING` transition state.

        Possible next states:
        - `SwitchController` response indicated success: `IDLE`
        - `SwitchController` response indicated failure: `ERROR`
        """
        self.state = BagRecorderManagerState.STOPPING_TRAINING
        request = SwitchController.Request(
            # TODO: Make controller name configurable
            activate_controllers=[],
            deactivate_controllers=["franka_gravity_compensation_example_controller"],
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(sec=1),
        )

        response: SwitchController.Response | None = self.switch_controller_service.call_sync_and_await(request)

        if response is None or not response.ok:
            self.get_logger().error(f"Error while transitioning to state {BagRecorderManagerState.TRAINING_IDLE}")
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.IDLE

    def start_recording(self) -> None:
        """Enter the `STARTING_RECORDING` transition state.

        Possible next states:
        - `StartBagRecording` response indicated success: `RECORDING`
        - `StartBagRecording` response indicated failure: `ERROR`
        """
        self.state = BagRecorderManagerState.STARTING_RECORDING
        request = StartBagRecording.Request(
            # TODO: Create better bag path and name, perhaps based on participant etc
            bag_path="bags/recordings/",
            bag_name=f"recording_{len(self.recorded_bag_uris):03}",
            append_timestamp=False,
        )

        response: StartBagRecording.Response | None = self.start_recording_service.call_sync_and_await(request)

        if response is None or not response.started:
            self.get_logger().error("Could not start recording")
            self.state = BagRecorderManagerState.ERROR

        else:
            # Store the recorded bag URI in the list of trainings
            self.recorded_bag_uris.append(response.opened_bag_uri)
            self.state = BagRecorderManagerState.RECORDING

    def stop_recording(self) -> None:
        """Enter the `STOPPING_RECORDING` transition state.

        Possible next states:
        - `StopBagRecording` response indicated success: `TRAINING_IDLE`
        - `StopBagRecording` response indicated failure: `ERROR`
        """
        self.state = BagRecorderManagerState.STOPPING_RECORDING
        request = StopBagRecording.Request()

        response: StopBagRecording.Response | None = self.stop_recording_service.call_sync_and_await(request)

        if response is None:
            self.get_logger().error("Could not stop recording")
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.TRAINING_IDLE

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
        # Transition to the next state immediately
        self.state = BagRecorderManagerState.IDLE

    def start_replaying(self) -> None:
        """Enter the `STARTING_REPLAYING` transition state.

        Possible next states:
        - `SwitchController` response indicated success: `REPLAYING`
        - `SwitchController` response indicated failure, or the bag URI was not set on the playback controller: `ERROR`
        """
        self.state = BagRecorderManagerState.STARTING_REPLAYING

        # Check if we can play back the next recording
        # TODO: Make a better way to influence which recording is selected
        if self.nr_recordings_played_back >= len(self.recorded_bag_uris):
            self.get_logger().error("There is no new recording to play back!")
            self.state = BagRecorderManagerState.ERROR
            return

        # Create a service request to change the used bag URI for the playback controller before the controller starts
        bag_uri_parameter_request = SetParameters.Request(
            parameters=[
                Parameter(
                    name="bag_uri",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING,
                        # TODO: Make a better way to influence which recording is selected
                        string_value=self.recorded_bag_uris[self.nr_recordings_played_back],
                    ),
                )
            ]
        )

        bag_uri_response: SetParameters.Response | None = self.set_parameters_service.call_sync_and_await(
            bag_uri_parameter_request
        )

        # Check that the new bag URI is set successfully
        if bag_uri_response is None or not all(result.successful for result in bag_uri_response.results):
            self.get_logger().error("Could not update the bag URI parameter.")
            self.state = BagRecorderManagerState.ERROR
            return

        # Now we can activate the playback controller
        request = SwitchController.Request(
            # TODO: Make controller name configurable
            activate_controllers=["nakama_bag_playback_joint_impedance_controller"],
            deactivate_controllers=[],
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(sec=1),
        )
        response: SwitchController.Response | None = self.switch_controller_service.call_sync_and_await(request)

        if response is None or not response.ok:
            self.get_logger().error(f"Error while transitioning to state {BagRecorderManagerState.REPLAYING}")
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.REPLAYING

    def stop_replaying(self) -> None:
        """Enter the `STOPPING_REPLAYING` transition state.

        Possible next states:
        - `SwitchController` response indicated success: `OPERATING_IDLE`
        - `SwitchController` response indicated failure: `ERROR`
        """
        self.state = BagRecorderManagerState.STOPPING_REPLAYING

        self.nr_recordings_played_back += 1  # Update the number of recordings which are played back

        request = SwitchController.Request(
            # TODO: Make controller name configurable
            activate_controllers=[],
            deactivate_controllers=["nakama_bag_playback_joint_impedance_controller"],
            strictness=SwitchController.Request.STRICT,
            activate_asap=True,
            timeout=Duration(sec=1),
        )
        response: SwitchController.Response | None = self.switch_controller_service.call_sync_and_await(request)

        if response is None or not response.ok:
            self.get_logger().error(f"Error while transitioning to state {BagRecorderManagerState.OPERATING_IDLE}")
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.OPERATING_IDLE


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
