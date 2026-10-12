from ..flight_state import FlightState
from .handler import FlightStateHandler


class WaitingForLandingHandler(FlightStateHandler):
    def handle(self, node):
        p = node.odometry.pose.pose.position
        if (node.saw_flying and not node.status.flying and not node.status.armed
                and p.z - node.start[2] < 0.35):
            node.get_logger().info('Landing completed; motors disarmed')
            node.get_logger().info(f'Final position: x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}')
            node.state = FlightState.FINISHED
