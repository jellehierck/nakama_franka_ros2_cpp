from enum import StrEnum, auto

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

from builtin_interfaces.msg import Duration

from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController

from bee_buttons_interfaces.msg import BeeButtonPress
from bee_buttons_interfaces.srv import BeeButtonSetColor


class BagRecorderManagerStates(StrEnum):
    IDLE = auto()

    PLAYING_BACK = auto()
    STOPPING_PLAYBACK = auto()
    STARTING_PLAYBACK = auto()

    RECORDING = auto()
    STOPPING_RECORDING = auto()
    STARTING_RECORDING = auto()

    HOMING = auto()
    STOPPING_HOMING = auto()
    STARTING_HOMING = auto()


class BagRecorderManager(Node):
    def __init__(self):
        super().__init__("bag_recorder_manager")

        self.state = BagRecorderManagerStates.IDLE

        # Create subscriptions
        self.button_press_subscription = self.create_subscription(
            BeeButtonPress,
            "button_press",
            self.handle_button_press,
        )

        # Create service clients to be able to make service requests
        # TODO: Make controller manager node name configurable with parameter? With namespacing perhaps
        self.controller_manager_load_controller_service_client = self.create_client(
            LoadController,
            "/controller_manager/load_controller",
        )
        self.controller_manager_load_controller_future: Future | None = None

        # Check that the services are available before continuing
        # TODO: Remove or make configurable?
        while not self.controller_manager_switch_controller_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def handle_finished_controller_manager_service_call(self) -> None:
        pass

    def handle_button_press(self, msg: BeeButtonPress) -> None:
        # Check that the button press comes from the correct button
        # TODO: Make node ID configurable
        if msg.node_id != "443":
            self.get_logger().warn(f"Got button press from button {msg.node_id} which this node does not listen to")
            return

        # Take action depending on how the button is pressed
        if msg.press_type == BeeButtonPress.PRESSED:
            pass

        elif msg.press_type == BeeButtonPress.DOUBLE_PRESSED:
            pass

        elif msg.press_type == BeeButtonPress.LONG_PRESS:
            if self.state == BagRecorderManagerStates.IDLE:
                # Move STARTING_RECORDING state
                self.start_recording()
                pass

            if self.state == BagRecorderManagerStates.RECORDING:
                # Move into STOPPING_RECORDING state
                pass

        else:
            self.get_logger().warn(f"Got unknown press type {msg.press_type}")
            return

    def start_recording(self) -> None:
        self.state = BagRecorderManagerStates.STARTING_RECORDING
        self.controller_manager_load_controller_future = self.load_recording_controller()
        self.controller_manager_load_controller_future.add_done_callback(self.start_recording_finished)

    def load_recording_controller(self) -> Future:
        request = LoadController.Request(
            # TODO: Make controller name a parameter
            name="nakama_bag_recorder_controller"
        )

        # Return the future created by calling the service
        return self.controller_manager_load_controller_service_client.call_async(request)

    def start_recording_finished(self, future: Future) -> None:
        response: LoadController.Response | None = future.result()

        if response is None or not response.ok:
            # TODO: Add controller names to the log statement
            self.get_logger().error("Could not switch the recorder controller in and the playback recorder out")

            # TODO: Perhaps trigger button colors to switch?

        else:
            self.state = BagRecorderManagerStates.RECORDING
            # TODO: Add controller names to the log statement
            self.get_logger().info("Recorder controller switched in, playback recorder switched out")

            # TODO: Trigger button colors to switch

        # Return the future created by calling the service
        future = self.controller_manager_load_controller_service_client.call_async(request)
        future.add_done_callback(self.start_recording_finished)


def main(args=None):
    """Start the node."""
    try:
        rclpy.init(args=args)
        bag_recorder_manager_node = BagRecorderManager()
        rclpy.spin(bag_recorder_manager_node)

    except KeyboardInterrupt:
        # Keyboard interrupts cause the program to stop gracefully
        pass

    except Exception as error:
        # All other errors must be reported
        print(error)

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
