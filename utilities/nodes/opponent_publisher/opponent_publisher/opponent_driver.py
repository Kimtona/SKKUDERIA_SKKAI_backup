"""Drive the f1tenth_gym second agent around the raceline as a moving obstacle.

Unlike obstacle_publisher, which injects an obstacle straight into
/perception/obstacles (virtual) or paints it into /map (lidar), this node steers
a real second gym agent. RaceCar.ray_cast_agents() casts against that agent's
body outline, so the ego's simulated LiDAR actually sees it and the whole
perception chain -- detect -> tracking -> spline_planner -> state_machine -- is
exercised for real.

Note that /map painting (obstacle_publisher's `lidar` mode) does nothing for the
ego's scan: gym_bridge fixes the scan simulator's map at gym.make() time and
never subscribes to /map, so a repainted map shows up in RViz but is invisible
to the LiDAR.

The parameter names mirror obstacle_publisher on purpose, so the same rqt
workflow applies; unlike obstacle_publisher, they are live-tunable here
(rqt_reconfigure, or `ros2 param set /opponent_driver speed_scaler 0.8`).

Publishing /opp_drive is also what unblocks the simulation: with num_agent > 1
the bridge only steps once both /drive and /opp_drive have been seen at least
once, so a silent opponent freezes the ego too.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from f110_msgs.msg import WpntArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# f1tenth_gym vehicle geometry, from stack_master/config/SIM/sim_params.yaml.
WHEELBASE = 0.15875 + 0.17145  # lf + lr [m]
MAX_STEER = 0.4189             # s_max [rad]

TRAJECTORY_TOPICS = {
    'min_curv': '/global_waypoints',
    'shortest_path': '/global_waypoints/shortest_path',
    'centerline': '/centerline_waypoints',
}


class OpponentDriver(Node):
    """Pure pursuit controller for the simulated opponent car."""

    def __init__(self):
        super().__init__('opponent_driver')

        self.declare_parameter('speed_scaler', 0.5, descriptor=ParameterDescriptor(
            description='Multiplier on the raceline speed profile, or the absolute '
                        'speed in m/s when constant_speed is true'))
        self.declare_parameter('constant_speed', False, descriptor=ParameterDescriptor(
            description='Ignore the raceline speed profile and hold speed_scaler m/s'))
        self.declare_parameter('trajectory', 'min_curv', descriptor=ParameterDescriptor(
            description='Which line to follow: min_curv / shortest_path / centerline'))
        self.declare_parameter('start_s', 22.0, descriptor=ParameterDescriptor(
            description='Frenet s [m] the opponent is placed at on startup'))
        self.declare_parameter('lookahead_dist', 0.8, descriptor=ParameterDescriptor(
            description='Pure pursuit lookahead distance [m]'))
        self.declare_parameter('rate', 40.0, descriptor=ParameterDescriptor(
            description='Control loop rate [Hz]'))

        self.speed_scaler = self.get_parameter('speed_scaler').value
        self.constant_speed = self.get_parameter('constant_speed').value
        self.trajectory = self.get_parameter('trajectory').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        rate = self.get_parameter('rate').value

        self.wpnts_xy = None      # (N, 2) map-frame positions
        self.wpnts_v = None       # (N,) raceline speed profile
        self.wpnts_s = None       # (N,) frenet s
        self.wpnt_spacing = None  # [m] between consecutive waypoints
        self.pose = None          # (x, y, yaw)
        self.placed = False

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)
        # gym_bridge resets the opponent from /goal_pose (PoseStamped); the ego's
        # equivalent is /initialpose.
        self.reset_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.wpnt_sub = None
        self._subscribe_trajectory(self.trajectory)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_cb, 10)

        self.add_on_set_parameters_callback(self.dyn_param_cb)
        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f'Opponent driver ready: following {self.trajectory}, '
            f'speed_scaler {self.speed_scaler}, start_s {self.get_parameter("start_s").value} m')

    def _subscribe_trajectory(self, trajectory):
        topic = TRAJECTORY_TOPICS.get(trajectory)
        if topic is None:
            self.get_logger().error(
                f'Unknown trajectory "{trajectory}", keeping the previous one. '
                f'Valid: {sorted(TRAJECTORY_TOPICS)}')
            return False
        if self.wpnt_sub is not None:
            self.destroy_subscription(self.wpnt_sub)
        self.wpnt_sub = self.create_subscription(WpntArray, topic, self.wpnts_cb, 10)
        return True

    def wpnts_cb(self, msg: WpntArray):
        if not msg.wpnts:
            return
        self.wpnts_xy = np.array([[w.x_m, w.y_m] for w in msg.wpnts])
        self.wpnts_v = np.array([w.vx_mps for w in msg.wpnts])
        self.wpnts_s = np.array([w.s_m for w in msg.wpnts])
        # Waypoints are evenly spaced along s, so one gap is enough to convert a
        # lookahead distance into an index offset.
        self.wpnt_spacing = float(np.median(np.diff(self.wpnts_s))) if len(msg.wpnts) > 1 else 0.1

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def dyn_param_cb(self, params):
        for p in params:
            if p.name == 'speed_scaler':
                self.speed_scaler = p.value
            elif p.name == 'constant_speed':
                self.constant_speed = p.value
            elif p.name == 'lookahead_dist':
                self.lookahead_dist = p.value
            elif p.name == 'trajectory':
                if not self._subscribe_trajectory(p.value):
                    return SetParametersResult(successful=False, reason='unknown trajectory')
                self.trajectory = p.value
                self.wpnts_xy = None  # wait for the new line before driving again
        return SetParametersResult(successful=True)

    def _place_at_start_s(self):
        """Put the opponent on the raceline at start_s, facing along the track."""
        start_s = self.get_parameter('start_s').value
        i = int(np.argmin(np.abs(self.wpnts_s - start_s)))
        nxt = (i + 1) % len(self.wpnts_xy)
        heading = np.arctan2(self.wpnts_xy[nxt][1] - self.wpnts_xy[i][1],
                             self.wpnts_xy[nxt][0] - self.wpnts_xy[i][0])
        q = quaternion_from_euler(0.0, 0.0, heading)

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.wpnts_xy[i][0])
        msg.pose.position.y = float(self.wpnts_xy[i][1])
        msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
        msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
        self.reset_pub.publish(msg)
        self.get_logger().info(
            f'Placed opponent at s={self.wpnts_s[i]:.2f} m '
            f'(x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f})')

    def _pure_pursuit(self):
        """Return (speed, steering_angle) from the f1tenth_gym pure pursuit geometry."""
        position = np.array(self.pose[:2])
        yaw = self.pose[2]

        i = int(np.argmin(np.linalg.norm(self.wpnts_xy - position, axis=1)))
        step = max(1, int(round(self.lookahead_dist / self.wpnt_spacing)))
        target = self.wpnts_xy[(i + step) % len(self.wpnts_xy)]

        speed = self.speed_scaler if self.constant_speed else self.wpnts_v[i] * self.speed_scaler

        # Lateral offset of the lookahead point in the car frame; the arc through it
        # has radius L^2 / 2y, and the bicycle model turns that into a steering angle.
        offset_y = float(np.dot(np.array([np.sin(-yaw), np.cos(-yaw)]), target - position))
        if abs(offset_y) < 1e-6:
            return speed, 0.0
        radius = 1.0 / (2.0 * offset_y / self.lookahead_dist ** 2)
        steering = float(np.arctan(WHEELBASE / radius))
        return speed, float(np.clip(steering, -MAX_STEER, MAX_STEER))

    def control_loop(self):
        if self.wpnts_xy is None or self.pose is None:
            return
        if not self.placed:
            self._place_at_start_s()
            self.placed = True
            return

        speed, steering = self._pure_pursuit()
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = steering
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
