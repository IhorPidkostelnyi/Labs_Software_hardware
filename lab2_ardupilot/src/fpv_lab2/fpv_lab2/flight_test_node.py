import json
import math
import time
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from .flight_state import FlightState
from .handlers import HANDLERS
from .navigation import target15


class FlightTestNode(Node):
    GUIDED_MODE = 4
    LAND_MODE = 9
    TAKEOFF_ALTITUDE = 4.5
    HOVER_DURATION = 30.0
    POSITION_TOLERANCE = 0.25
    MAX_SPEED = 0.65

    def __init__(self):
        super().__init__('flight_test')
        self.declare_parameter('ap_prefix', '/ap/v1')
        self.declare_parameter('result_file', '')
        prefix = str(self.get_parameter('ap_prefix').value).rstrip('/')
        self.state = FlightState.WAITING_FOR_SYSTEM
        self.request_in_progress = False
        self.status = self.odometry = None
        self.start = self.target = self.before_landing = None
        self.error_m = self.failure = None
        self.hover_started_at = self.altitude_stable_since = None
        self.target_stable_since = self.ready_since = None
        self.next_prearm_request = 0.0
        self.last_status = self.last_odometry = -math.inf
        self.previous_position = None
        self.horizontal_speed = math.inf
        self.saw_flying = False
        self._last_state = self.state
        self._phase_started = self._started = time.monotonic()
        self._request_since = None
        self._handlers = HANDLERS
        self.status_subscription = self.create_subscription(
            Status, prefix + '/status', self.status_callback, qos_profile_sensor_data)
        self.odometry_subscription = self.create_subscription(
            Odometry, '/odometry', self.odometry_callback, qos_profile_sensor_data)
        self.velocity_publisher = self.create_publisher(TwistStamped, prefix + '/cmd_vel', 10)
        self.prearm_client = self.create_client(Trigger, prefix + '/prearm_check')
        self.mode_client = self.create_client(ModeSwitch, prefix + '/mode_switch')
        self.arm_client = self.create_client(ArmMotors, prefix + '/arm_motors')
        self.takeoff_client = self.create_client(Takeoff, prefix + '/experimental/takeoff')
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Variant 15: direct flight above the maze at 4.5 m')

    def status_callback(self, message):
        self.status = message
        self.last_status = time.monotonic()
        self.saw_flying = self.saw_flying or bool(message.flying)

    def odometry_callback(self, message):
        now = time.monotonic()
        p = message.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.fail('Invalid odometry')
            return
        if self.previous_position is not None:
            old_x, old_y, previous_time = self.previous_position
            dt = now - previous_time
            if dt > 0.01:
                self.horizontal_speed = math.hypot(p.x - old_x, p.y - old_y) / dt
        self.previous_position = (p.x, p.y, now)
        self.odometry = message
        self.last_odometry = now

    def services_ready(self):
        return all(client.service_is_ready() for client in (
            self.prearm_client, self.mode_client, self.arm_client, self.takeoff_client))

    def capture_start(self):
        p = self.odometry.pose.pose.position
        self.start = (p.x, p.y, p.z)
        self.target = target15(p.x, p.y)
        self.get_logger().info(f'Start: {self.start}; target: {self.target}')

    def publish_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.linear.z = float(vz)
        self.velocity_publisher.publish(message)

    def control_loop(self):
        now = time.monotonic()
        if self.state != self._last_state:
            self.get_logger().info('State: ' + self.state.name)
            self._last_state = self.state
            self._phase_started = now
            self._request_since = None
        if self.state in (FlightState.FINISHED, FlightState.ERROR):
            return
        if now - self._phase_started > 240:
            self.fail('Flight stage timed out')
            return
        if self.start is not None and (
                now - self.last_status > 4 or now - self.last_odometry > 4):
            if self.failure is not None:
                if now - self._phase_started > 30:
                    self.state = FlightState.ERROR
                return
            self.fail('Telemetry timeout; requesting landing')
            return
        if self.request_in_progress:
            if self._request_since is None:
                self._request_since = now
            elif now - self._request_since > 12:
                self.fail('Service response timeout')
                return
        else:
            self._request_since = None
        handler = self._handlers.get(self.state)
        if handler is not None:
            handler.handle(self)

    def call_mode_switch(self, mode, callback):
        self.request_in_progress = True
        request = ModeSwitch.Request()
        request.mode = mode
        future = self.mode_client.call_async(request)
        future.add_done_callback(callback)
        self.get_logger().info(f'Requesting flight mode {mode}')

    def fail(self, message):
        self.get_logger().error(message)
        if self.failure is None:
            self.failure = message
        self.publish_velocity()
        self.request_in_progress = False
        if (self.status is not None and self.status.armed
                and self.state not in (FlightState.LANDING, FlightState.WAITING_FOR_LANDING)):
            self.state = FlightState.LANDING
        else:
            self.state = FlightState.ERROR

    def save_result(self):
        success = (self.state == FlightState.FINISHED and self.failure is None
                   and self.error_m is not None and self.error_m <= self.POSITION_TOLERANCE)
        p = self.odometry.pose.pose.position if self.odometry is not None else None
        result = {'variant': 15, 'height_m': self.TAKEOFF_ALTITUDE,
                  'start': self.start, 'target': self.target,
                  'before_landing': self.before_landing, 'error_m': self.error_m,
                  'final_position': [p.x, p.y, p.z] if p is not None else None,
                  'landing_completed': self.state == FlightState.FINISHED,
                  'flight_verified': success, 'failure': self.failure,
                  'elapsed_s': time.monotonic() - self._started}
        filename = str(self.get_parameter('result_file').value)
        if filename:
            path = Path(filename)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2), encoding='utf-8')
        return success
