import time
from ..flight_state import FlightState
from ..navigation import velocity
from .handler import FlightStateHandler


class MovingToTargetHandler(FlightStateHandler):
    def handle(self, node):
        if not node.status.armed or not node.status.flying:
            node.fail('Unexpected flight status while moving')
            return
        p = node.odometry.pose.pose.position
        altitude = p.z - node.start[2]
        vz = max(-0.4, min(0.4, 0.6 * (node.TAKEOFF_ALTITUDE - altitude)))
        if altitude < node.TAKEOFF_ALTITUDE - 0.35:
            node.publish_velocity(0.0, 0.0, vz)
            node.target_stable_since = None
            return
        vx, vy, distance = velocity(p.x, p.y, *node.target, max_speed=node.MAX_SPEED)
        node.publish_velocity(vx, vy, vz)
        if distance > node.POSITION_TOLERANCE or node.horizontal_speed > 0.12:
            node.target_stable_since = None
            return
        now = time.monotonic()
        if node.target_stable_since is None:
            node.target_stable_since = now
        if now - node.target_stable_since >= 5.0:
            node.before_landing = (p.x, p.y, p.z)
            node.error_m = distance
            node.get_logger().info(
                f'Target reached: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}; '
                f'error={distance:.4f} m')
            node.publish_velocity()
            node.state = FlightState.LANDING
