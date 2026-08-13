#!/usr/bin/env python3
"""Bridge RViz's "2D Pose Estimate" button to Cartographer pure localization.

Cartographer has no /initialpose subscriber (that topic is an AMCL/nav2
convention), so clicking the button normally does nothing on the car. This
node listens on /initialpose and, per click:

  1. finds all ACTIVE trajectories via /get_trajectory_states
  2. finishes them via /finish_trajectory
  3. starts a fresh localization trajectory at the clicked pose via
     /start_trajectory (relative to trajectory 0, the frozen .pbstream map)

Result: localization restarts instantly at the clicked pose instead of
waiting for Cartographer's sampled global relocalization to converge.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.msg import StatusCode, TrajectoryStates
from cartographer_ros_msgs.srv import (
    FinishTrajectory,
    GetTrajectoryStates,
    StartTrajectory,
)

# The trajectory loaded from the .pbstream (-load_state_filename) is always
# id 0 and FROZEN; initial poses are expressed relative to it.
MAP_TRAJECTORY_ID = 0
SERVICE_TIMEOUT_SEC = 5.0


class InitialPoseToCartographer(Node):

    def __init__(self):
        super().__init__('initialpose_to_cartographer')

        self.declare_parameter('configuration_directory', '')
        self.declare_parameter('configuration_basename', 'f110_2d_loc.lua')
        self.config_dir = self.get_parameter('configuration_directory').value
        self.config_basename = self.get_parameter('configuration_basename').value
        if not self.config_dir:
            raise RuntimeError(
                'configuration_directory param is required (path to the dir '
                'containing the localization .lua)')

        # Reentrant group + MultiThreadedExecutor so the synchronous service
        # calls inside the subscription callback can be serviced.
        self.cb_group = ReentrantCallbackGroup()
        self.get_states_client = self.create_client(
            GetTrajectoryStates, 'get_trajectory_states',
            callback_group=self.cb_group)
        self.finish_client = self.create_client(
            FinishTrajectory, 'finish_trajectory',
            callback_group=self.cb_group)
        self.start_client = self.create_client(
            StartTrajectory, 'start_trajectory',
            callback_group=self.cb_group)

        self.busy = False
        # RViz publishes /initialpose BEST_EFFORT here (same gotcha as /scan):
        # a default RELIABLE subscription is QoS-incompatible and receives
        # nothing. A best-effort subscription accepts both publisher types.
        initialpose_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # self.create_subscription(
        #     PoseWithCovarianceStamped, '/initialpose',
        #     self.initialpose_cb, 1, callback_group=self.cb_group)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.initialpose_cb, initialpose_qos,
            callback_group=self.cb_group)

        self.get_logger().info(
            'Relaying /initialpose to Cartographer '
            f'({self.config_dir}/{self.config_basename})')

    def call(self, client, request):
        """Synchronous service call with timeout; returns None on failure."""
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            self.get_logger().error(
                f'Service {client.srv_name} not available')
            return None
        future = client.call_async(request)
        # Executor spins this future on another thread of the pool.
        deadline = time.monotonic() + SERVICE_TIMEOUT_SEC
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'Service {client.srv_name} timed out')
                future.cancel()
                return None
            time.sleep(0.05)
        return future.result()

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        if self.busy:
            self.get_logger().warn(
                'Ignoring /initialpose: previous relocalization still running')
            return
        self.busy = True
        try:
            self.relocalize(msg)
        finally:
            self.busy = False

    def relocalize(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.get_logger().info(
            f'Relocalizing at x={p.x:.2f} y={p.y:.2f} '
            f'(frame {msg.header.frame_id})')
        if msg.header.frame_id and msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'/initialpose frame is "{msg.header.frame_id}", expected '
                '"map" — check the RViz Fixed Frame; using the pose as-is')

        # 1. Find and finish every ACTIVE trajectory (the frozen map
        #    trajectory 0 is not ACTIVE, so it is never touched).
        states = self.call(self.get_states_client,
                           GetTrajectoryStates.Request())
        if states is None:
            return
        active_ids = [
            tid for tid, tstate in zip(
                states.trajectory_states.trajectory_id,
                states.trajectory_states.trajectory_state)
            if tstate == TrajectoryStates.ACTIVE
        ]
        for tid in active_ids:
            req = FinishTrajectory.Request(trajectory_id=tid)
            resp = self.call(self.finish_client, req)
            if resp is None or resp.status.code != StatusCode.OK:
                detail = resp.status.message if resp else 'no response'
                self.get_logger().error(
                    f'finish_trajectory({tid}) failed: {detail}')
                return
            self.get_logger().info(f'Finished trajectory {tid}')

        # 2. Start a new localization trajectory at the clicked pose.
        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_basename
        req.use_initial_pose = True
        req.initial_pose = msg.pose.pose
        req.relative_to_trajectory_id = MAP_TRAJECTORY_ID
        resp = self.call(self.start_client, req)
        if resp is None or resp.status.code != StatusCode.OK:
            detail = resp.status.message if resp else 'no response'
            self.get_logger().error(f'start_trajectory failed: {detail}')
            return
        self.get_logger().info(
            f'Started trajectory {resp.trajectory_id} at the clicked pose')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseToCartographer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
