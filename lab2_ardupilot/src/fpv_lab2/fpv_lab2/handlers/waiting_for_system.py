import time
from ..flight_state import FlightState
from .handler import FlightStateHandler


class WaitingForSystemHandler(FlightStateHandler):
    def handle(self, node):
        now = time.monotonic()
        ready = (node.status is not None and node.odometry is not None
                 and now - node.last_status < 2 and now - node.last_odometry < 2
                 and node.services_ready()
                 and node.velocity_publisher.get_subscription_count() > 0)
        if not ready:
            node.ready_since = None
            return
        if node.ready_since is None:
            node.ready_since = now
        if now - node.ready_since < 2.0:
            return
        if node.status.armed or node.status.flying:
            node.fail('Start a fresh simulation with the drone on the ground')
            return
        node.capture_start()
        node.state = FlightState.PREARM_CHECK
