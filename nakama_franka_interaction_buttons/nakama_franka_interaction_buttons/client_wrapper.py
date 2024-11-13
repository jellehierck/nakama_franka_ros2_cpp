from collections.abc import Callable
from typing import TypeVar

import rclpy
from rclpy.client import Client
from rclpy.executors import Executor
from rclpy.node import Node
from rclpy.task import Future

SrvTypeRequest = TypeVar("SrvTypeRequest")
"""Service request type, for documentation only."""

SrvTypeResponse = TypeVar("SrvTypeResponse")
"""Service request type, for documentation only."""

FutureDoneCallback = Callable[[Future], None]
"""Function signature of `Future.add_done_callback()`."""


class AsyncClientWrapper:
    """Asynchronous service client wrapper which includes the client and a future obtained by sending a request to the
    client asynchronously.
    """

    def __init__(self, client: Client) -> None:
        """Instantiate the AsyncClientWrapper.

        :param client: Client obect to bind. The client should be created with `Node.create_client()`.
        """
        self.client = client
        self.future: Future | None = None

    def call_async(self, request: SrvTypeRequest, done_callback: FutureDoneCallback | None = None) -> Future:
        """Send the service request to the client asynchronously. A done callback can be registered which is called when
        the future resolves.

        Note: this function behaviour is undefined if called before the any previous future is resolved.

        :param request: Request to send.
        :param done_callback: (optional) Done callback to add to the future when it resolves. If not given or set to
        None, no callback fill be fired when the future resolves. Defaults to None.
        :return: The created future of the asynchronous request.
        """
        self.future = self.client.call_async(request=request)

        if done_callback is not None:
            self.future.add_done_callback(done_callback)

        return self.future

    def call_sync_and_await(self, request: SrvTypeRequest) -> SrvTypeResponse | None:
        """Send the service request to the client and block until it resolves.

        Note: this should never be called from another callback, UNLESS you have taken precautions to prevent a silent
        deadlock. One option is to run the node using a MultiThreadedExecutor and using a different callback group for
        the service client and the callback which calls this function. See
        <https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html#avoiding-deadlocks> for more details.

        :param request: Request to send.
        :return: Result of the service call. This is usually the response, but can be `None` if the request timed out.
        """
        return self.client.call(request)
