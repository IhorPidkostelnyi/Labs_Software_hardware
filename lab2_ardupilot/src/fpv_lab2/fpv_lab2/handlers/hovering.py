import time
from ..flight_state import FlightState
from .handler import FlightStateHandler


class HoveringHandler(FlightStateHandler):
    def handle(self, node):
        if not node.status.armed:
            node.fail('Drone disarmed during takeoff')
            return
        p = node.odometry.pose.pose.position
        altitude = p.z - node.start[2]
        now = time.monotonic()
        if abs(altitude - node.TAKEOFF_ALTITUDE) > 0.2 or not node.status.flying:
            node.altitude_stable_since = node.hover_started_at = None
            return
        if node.altitude_stable_since is None:
            node.altitude_stable_since = now
        if now - node.altitude_stable_since < 2.0:
            return
        if node.hover_started_at is None:
            node.hover_started_at = now
            node.get_logger().info(f'Altitude reached: {altitude:.2f} m; hovering')
        if now - node.hover_started_at >= node.HOVER_DURATION:
            node.get_logger().info('Hover completed; flying directly to the target')
            node.state = FlightState.MOVING_TO_TARGET
