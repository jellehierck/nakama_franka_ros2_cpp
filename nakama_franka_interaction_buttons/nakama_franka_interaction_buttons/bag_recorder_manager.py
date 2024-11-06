from enum import StrEnum, auto

import rclpy
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.task import Future

from controller_manager_msgs.srv import LoadController, SwitchController, UnloadController

from bee_buttons_interfaces.msg import BeeButtonColor, BeeButtonPress
from bee_buttons_interfaces.srv import BeeButtonSetColor

from nakama_franka_interaction_buttons.client_wrapper import AsyncClientWrapper
from nakama_franka_interfaces.srv import StartBagRecording, StopBagRecording


class BagRecorderManagerState(StrEnum):
    """Possible states for the BagRecorderManager."""

    IDLE = auto()

    ERROR = auto()

    PLAYING_BACK = auto()
    STOPPING_PLAYBACK = auto()
    STARTING_PLAYBACK = auto()

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

        # Declare parameters
        self.recorder_controller_param = self.declare_parameter(
            name="recorder_controller",
            value="nakama_bag_recorder_controller",
            descriptor=ParameterDescriptor(
                description="Name of the recorder controller to be loaded into the controller manager",
            ),
        )

        self.wait_for_services_param = self.declare_parameter(
            name="wait_for_services",
            value=True,
            descriptor=ParameterDescriptor(
                description="Whether to wait for required services to become available on startup.",
                read_only=True,  # Only takes effect when the node is started
            ),
        )

        # Create service clients to be able to make service requests
        def wait_for_service(client_wrapper: AsyncClientWrapper) -> None:
            """Helper function to wait on the service to become availabe if `wait_for_services` parameter is True."""
            if self.wait_for_services_param.get_parameter_value().bool_value:
                # Wait for some seconds before logging a new message that the service is unavailable
                while not client_wrapper.client.wait_for_service(timeout_sec=2.0):
                    self.get_logger().info(f"Service {client_wrapper.client.srv_name} not available, waiting...")

        # Controller manager services
        self.load_controller_service = AsyncClientWrapper(
            self.create_client(LoadController, srv_name="controller_manager/load_controller")
        )
        wait_for_service(self.load_controller_service)

        self.unload_controller_service = AsyncClientWrapper(
            self.create_client(UnloadController, srv_name="controller_manager/unload_controller")
        )
        wait_for_service(self.unload_controller_service)

        # Bag recorder services
        self.start_recording_service = AsyncClientWrapper(
            self.create_client(StartBagRecording, srv_name="start_recording")
        )
        wait_for_service(self.start_recording_service)

        self.stop_recording_service = AsyncClientWrapper(
            self.create_client(StartBagRecording, srv_name="stop_recording")
        )
        wait_for_service(self.stop_recording_service)

        # Button color services
        self.set_button_color_service = AsyncClientWrapper(
            self.create_client(BeeButtonSetColor, srv_name="set_button_color")
        )
        wait_for_service(self.set_button_color_service)

        # Create subscriptions
        self.button_press_subscription = self.create_subscription(
            BeeButtonPress,
            "button_press",
            self.handle_button_press,
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
                set_color_request=BeeButtonSetColor.Request(
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
                set_color_request=BeeButtonSetColor.Request(
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
                set_color_request=BeeButtonSetColor.Request(
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
                set_color_request=BeeButtonSetColor.Request(
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
                set_color_request=BeeButtonSetColor.Request(
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
        if self.state == BagRecorderManagerState.IDLE:  # noqa: SIM102
            if msg.press_type == BeeButtonPress.LONG_PRESS:
                self.start_training()
                return

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

        # The current combination of state and button press type has no effect
        self.get_logger().warn(f"Button press type {msg.press_type} has no effect in state {self._state}")

    def start_training(self) -> None:
        """Enter the `STARTING_TRAINING` transition state."""
        self.state = BagRecorderManagerState.STARTING_TRAINING
        request = LoadController.Request(name=self.recorder_controller_param.get_parameter_value().string_value)
        self.load_controller_service.call_async(request, done_callback=self.start_training_finished)

    def start_training_finished(self, resolved_future: Future) -> None:
        """End of the `STARTING_TRAINING` transition phase.

        Possible next states:
        - `LoadController` response indicated success: `TRAINING_IDLE`
        - `LoadController` response indicated failure: `ERROR`

        :param resolved_future: Resolved future of the service request to load the recorder controller.
        """
        response: LoadController.Response | None = resolved_future.result()

        if response is None or not response.ok:
            self.get_logger().error(
                f"Could not load {self.recorder_controller_param.get_parameter_value().string_value}"
            )
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.TRAINING_IDLE

    def stop_training(self) -> None:
        """Enter the `STOPPING_TRAINING` transition state."""
        self.state = BagRecorderManagerState.STOPPING_TRAINING
        request = UnloadController.Request(self.recorder_controller_param.get_parameter_value().string_value)
        self.unload_controller_service.call_async(request, done_callback=self.stop_training_finished)

    def stop_training_finished(self, resolved_future: Future) -> None:
        """End of the `STOPPING_TRAINING` transition phase.

        Possible next states:
        - `UnloadController` response indicated success: `IDLE`
        - `UnloadController` response indicated failure: `ERROR`

        :param resolved_future: Resolved future of the service request to unload the recorder controller.
        """
        response: UnloadController.Response | None = resolved_future.result()

        if response is None or not response.ok:
            self.get_logger().error(
                f"Could not unload {self.recorder_controller_param.get_parameter_value().string_value}"
            )
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.IDLE

    def start_recording(self) -> None:
        """Enter the `STARTING_RECORDING` transition state."""
        self.state = BagRecorderManagerState.STARTING_RECORDING
        request = StartBagRecording.Request(
            bag_path="recordings/",
            bag_name="recording",  # TODO: Create better bag name, perhaps based on participant etc
            append_timestamp=False,
        )
        self.start_recording_service.call_async(request, done_callback=self.start_recording_finished)

    def start_recording_finished(self, resolved_future: Future) -> None:
        """End of the `STARTING_RECORDING` transition phase.

        Possible next states:
        - `StartBagRecording` response indicated success: `RECORDING`
        - `StartBagRecording` response indicated failure: `ERROR`

        :param resolved_future: Resolved future of the service request to start recording.
        """
        response: StartBagRecording.Response | None = resolved_future.result()

        if response is None or not response.started:
            self.get_logger().error("Could not start recording")
            self.state = BagRecorderManagerState.ERROR

        else:
            # TODO: Store opened bag URI
            todo = response.opened_bag_uri
            self.state = BagRecorderManagerState.RECORDING

    def stop_recording(self) -> None:
        """Enter the `STOPPING_RECORDING` transition state."""
        self.state = BagRecorderManagerState.STOPPING_RECORDING
        request = StopBagRecording.Request()
        self.stop_recording_service.call_async(request, done_callback=self.stop_recording_finished)

    def stop_recording_finished(self, resolved_future: Future) -> None:
        """End of the `STOPPING_RECORDING` transition phase.

        Possible next states:
        - `StopBagRecording` response indicated success: `TRAINING_IDLE`
        - `StopBagRecording` response indicated failure: `ERROR`

        :param resolved_future: Resolved future of the service request to start recording.
        """
        response: StopBagRecording.Response | None = resolved_future.result()

        if response is None:
            self.get_logger().error("Could not stop recording")
            self.state = BagRecorderManagerState.ERROR

        else:
            self.state = BagRecorderManagerState.TRAINING_IDLE


def main(args: list[str] | None = None) -> None:
    """Start the node."""
    try:
        rclpy.init(args=args)
        bag_recorder_manager_node = BagRecorderManager()
        rclpy.spin(bag_recorder_manager_node)

    except KeyboardInterrupt:
        # Keyboard interrupts cause the program to stop gracefully
        pass

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
