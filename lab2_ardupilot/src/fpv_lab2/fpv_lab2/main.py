import time
import rclpy
from .flight_state import FlightState
from .flight_test_node import FlightTestNode


def main(args=None):
    rclpy.init(args=args)
    node = FlightTestNode()
    try:
        while rclpy.ok() and node.state not in (FlightState.FINISHED, FlightState.ERROR):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.fail('Interrupted; requesting landing')
        deadline = time.monotonic() + 120
        try:
            while (rclpy.ok() and time.monotonic() < deadline
                   and node.state not in (FlightState.FINISHED, FlightState.ERROR)):
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            node.state = FlightState.ERROR
    finally:
        success = node.save_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)
