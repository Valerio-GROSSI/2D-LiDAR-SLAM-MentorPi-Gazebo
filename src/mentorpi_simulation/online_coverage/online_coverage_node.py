import math
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]


class OnlineCoverageNode(Node):
    """Coverage manager for an occupancy grid that grows during online SLAM.

    The node samples sweep points in alternating rows, records the robot's
    actually travelled footprint, and uses frontier-adjacent goals when more
    of the unknown environment must be revealed.
    """

    def __init__(self) -> None:
        super().__init__('online_coverage_node')

        self.declare_parameter('enabled_on_startup', False)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('coverage_width', 0.30)
        self.declare_parameter('overlap_ratio', 0.15)
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('safety_margin', 0.05)
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('minimum_goal_distance', 0.20)
        self.declare_parameter('failed_goal_exclusion_radius', 0.35)
        self.declare_parameter('pose_record_distance', 0.025)
        self.declare_parameter('planning_period', 1.0)
        self.declare_parameter('transform_timeout', 0.20)
        self.declare_parameter('completion_stable_cycles', 8)
        self.declare_parameter('return_to_initial_pose', True)

        self.enabled = bool(self.get_parameter('enabled_on_startup').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.base_frame = str(self.get_parameter('robot_base_frame').value)
        self.coverage_width = float(self.get_parameter('coverage_width').value)
        self.overlap = float(self.get_parameter('overlap_ratio').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.min_goal_distance = float(self.get_parameter('minimum_goal_distance').value)
        self.failed_radius = float(self.get_parameter('failed_goal_exclusion_radius').value)
        self.pose_record_distance = float(self.get_parameter('pose_record_distance').value)
        self.tf_timeout = float(self.get_parameter('transform_timeout').value)
        self.completion_cycles = int(self.get_parameter('completion_stable_cycles').value)
        self.return_to_initial = bool(self.get_parameter('return_to_initial_pose').value)

        if not 0.0 <= self.overlap < 1.0:
            raise ValueError('overlap_ratio must be in [0, 1)')
        if self.coverage_width <= 0.0:
            raise ValueError('coverage_width must be positive')

        self.map_msg: Optional[OccupancyGrid] = None
        self.grid: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None
        self.traversable: Optional[np.ndarray] = None
        self.trajectory: List[WorldPoint] = []
        self.failed_goals: List[WorldPoint] = []
        self.initial_pose: Optional[WorldPoint] = None
        self.last_pose: Optional[WorldPoint] = None
        self.active_goal: Optional[WorldPoint] = None
        self.goal_handle = None
        self.goal_in_progress = False
        self.returning_home = False
        self.no_target_cycles = 0

        callback_group = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._map_callback,
            map_qos,
            callback_group=callback_group)

        self.coverage_pub = self.create_publisher(
            OccupancyGrid, '/coverage_map', map_qos)
        self.plan_pub = self.create_publisher(Path, '/coverage_plan', 10)
        self.start_service = self.create_service(
            SetBool, '/coverage/set_enabled', self._set_enabled,
            callback_group=callback_group)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('navigate_to_pose_action').value),
            callback_group=callback_group)

        self.create_timer(
            float(self.get_parameter('planning_period').value),
            self._tick,
            callback_group=callback_group)

        self.get_logger().info(
            'Online coverage ready. Enable with: ros2 service call '
            '/coverage/set_enabled std_srvs/srv/SetBool "{data: true}"')

    def _set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        self.enabled = request.data
        if not self.enabled:
            self._cancel_active_goal()
        else:
            self.returning_home = False
            self.no_target_cycles = 0
        response.success = True
        response.message = 'coverage enabled' if self.enabled else 'coverage stopped'
        return response

    def _map_callback(self, msg: OccupancyGrid) -> None:
        expected = msg.info.width * msg.info.height
        if expected == 0 or len(msg.data) != expected:
            return

        metadata_changed = (
            self.map_msg is None
            or msg.info.width != self.map_msg.info.width
            or msg.info.height != self.map_msg.info.height
            or msg.info.resolution != self.map_msg.info.resolution
            or msg.info.origin.position.x != self.map_msg.info.origin.position.x
            or msg.info.origin.position.y != self.map_msg.info.origin.position.y)

        self.map_msg = msg
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width))
        self.traversable = self._compute_traversable(self.grid)

        if metadata_changed or self.covered is None:
            self.covered = np.zeros_like(self.grid, dtype=bool)
            for point in self.trajectory:
                self._mark_covered(point)

        self._publish_coverage_map()

    def _compute_traversable(self, grid: np.ndarray) -> np.ndarray:
        free = (grid >= 0) & (grid <= self.free_threshold)
        occupied = grid >= self.occupied_threshold
        if self.map_msg is None:
            return free

        radius_m = self.robot_radius + self.safety_margin
        radius_cells = max(1, int(math.ceil(radius_m / self.map_msg.info.resolution)))
        inflated = occupied.copy()
        h, w = grid.shape

        # Binary disk dilation without an OpenCV/SciPy runtime dependency.
        for dy in range(-radius_cells, radius_cells + 1):
            max_dx = int(math.floor(math.sqrt(radius_cells ** 2 - dy ** 2)))
            src_y0 = max(0, -dy)
            src_y1 = min(h, h - dy)
            dst_y0 = max(0, dy)
            dst_y1 = min(h, h + dy)
            for dx in range(-max_dx, max_dx + 1):
                src_x0 = max(0, -dx)
                src_x1 = min(w, w - dx)
                dst_x0 = max(0, dx)
                dst_x1 = min(w, w + dx)
                inflated[dst_y0:dst_y1, dst_x0:dst_x1] |= occupied[
                    src_y0:src_y1, src_x0:src_x1]

        return free & ~inflated

    def _robot_pose(self) -> Optional[WorldPoint]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout))
            return (
                transform.transform.translation.x,
                transform.transform.translation.y)
        except TransformException as exc:
            self.get_logger().debug(f'TF unavailable: {exc}')
            return None

    def _tick(self) -> None:
        if self.map_msg is None or self.grid is None or self.covered is None:
            return

        pose = self._robot_pose()
        if pose is None:
            return

        if self.initial_pose is None:
            self.initial_pose = pose
        if self.last_pose is None or self._distance(pose, self.last_pose) >= self.pose_record_distance:
            self.trajectory.append(pose)
            self.last_pose = pose
            self._mark_covered(pose)
            self._publish_coverage_map()

        if not self.enabled or self.goal_in_progress:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.05):
            self.get_logger().warn('Waiting for Nav2 navigate_to_pose action...', throttle_duration_sec=5.0)
            return

        target = self._select_coverage_target(pose)
        target_kind = 'coverage'
        if target is None:
            target = self._select_frontier_target(pose)
            target_kind = 'frontier'

        if target is not None:
            self.no_target_cycles = 0
            self.returning_home = False
            self._send_goal(target, target_kind)
            return

        self.no_target_cycles += 1
        if self.no_target_cycles < self.completion_cycles:
            return

        if (self.return_to_initial and self.initial_pose is not None
                and self._distance(pose, self.initial_pose) > self.goal_tolerance
                and not self.returning_home):
            self.returning_home = True
            self._send_goal(self.initial_pose, 'return')
            return

        self.enabled = False
        self.get_logger().info('Coverage complete: no uncovered reachable sample or frontier remains.')

    def _mark_covered(self, point: WorldPoint) -> None:
        if self.covered is None or self.map_msg is None:
            return
        center = self._world_to_cell(point)
        if center is None:
            return
        cx, cy = center
        radius = max(1, int(math.ceil(0.5 * self.coverage_width / self.map_msg.info.resolution)))
        h, w = self.covered.shape
        for dy in range(-radius, radius + 1):
            yy = cy + dy
            if yy < 0 or yy >= h:
                continue
            max_dx = int(math.floor(math.sqrt(radius ** 2 - dy ** 2)))
            x0 = max(0, cx - max_dx)
            x1 = min(w, cx + max_dx + 1)
            self.covered[yy, x0:x1] = True

    def _select_coverage_target(self, pose: WorldPoint) -> Optional[WorldPoint]:
        assert self.map_msg is not None
        assert self.traversable is not None
        assert self.covered is not None

        resolution = self.map_msg.info.resolution
        spacing = max(1, int(round(self.coverage_width * (1.0 - self.overlap) / resolution)))
        candidates: List[Tuple[float, WorldPoint]] = []
        h, w = self.traversable.shape

        # Alternating rows produce a boustrophedon (lawnmower) ordering.
        for row_index, y in enumerate(range(spacing // 2, h, spacing)):
            xs = range(spacing // 2, w, spacing)
            if row_index % 2:
                xs = reversed(list(xs))
            for order, x in enumerate(xs):
                if not self.traversable[y, x] or self.covered[y, x]:
                    continue
                world = self._cell_to_world((x, y))
                if self._distance(world, pose) < self.min_goal_distance:
                    continue
                if self._is_failed(world):
                    continue
                distance = self._distance(world, pose)
                # Nearest useful stripe first; tiny order term keeps row direction.
                candidates.append((distance + 1.0e-5 * order, world))

        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        self._publish_plan([item[1] for item in candidates[:200]])
        return candidates[0][1]

    def _select_frontier_target(self, pose: WorldPoint) -> Optional[WorldPoint]:
        assert self.grid is not None
        assert self.traversable is not None
        unknown = self.grid < 0
        adjacent_unknown = np.zeros_like(unknown, dtype=bool)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        frontiers = self.traversable & adjacent_unknown

        ys, xs = np.nonzero(frontiers)
        candidates = []
        for x, y in zip(xs.tolist(), ys.tolist()):
            world = self._cell_to_world((x, y))
            if self._distance(world, pose) < self.min_goal_distance or self._is_failed(world):
                continue
            candidates.append((self._distance(world, pose), world))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None

    def _send_goal(self, target: WorldPoint, kind: str) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = target[0]
        goal.pose.pose.position.y = target[1]
        goal.pose.pose.orientation.w = 1.0

        self.goal_in_progress = True
        self.active_goal = target
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._goal_response(result, kind))
        self.get_logger().info(
            f'Sending {kind} goal: x={target[0]:.2f}, y={target[1]:.2f}')

    def _goal_response(self, future, kind: str) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:  # ROS action transport error
            self.get_logger().error(f'Goal request failed: {exc}')
            self._finish_goal(False, kind)
            return
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the coverage goal')
            self._finish_goal(False, kind)
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._goal_result(result, kind))

    def _goal_result(self, future, kind: str) -> None:
        try:
            wrapped = future.result()
            succeeded = wrapped.status == 4  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED
        except Exception as exc:
            self.get_logger().error(f'Goal result failed: {exc}')
            succeeded = False
        self._finish_goal(succeeded, kind)

    def _finish_goal(self, succeeded: bool, kind: str) -> None:
        if not succeeded and self.active_goal is not None and kind != 'return':
            self.failed_goals.append(self.active_goal)
            self.get_logger().warn('Goal failed; temporarily excluding this area')
        if kind == 'return' and succeeded:
            self.enabled = False
            self.get_logger().info('Coverage complete and initial pose reached.')
        self.goal_in_progress = False
        self.goal_handle = None
        self.active_goal = None

    def _cancel_active_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_in_progress = False
        self.active_goal = None

    def _is_failed(self, point: WorldPoint) -> bool:
        return any(self._distance(point, failed) <= self.failed_radius
                   for failed in self.failed_goals)

    def _world_to_cell(self, point: WorldPoint) -> Optional[GridCell]:
        assert self.map_msg is not None
        info = self.map_msg.info
        # OccupancyGrid origins are normally yaw=0 for SLAM Toolbox/RTAB-Map.
        x = int(math.floor((point[0] - info.origin.position.x) / info.resolution))
        y = int(math.floor((point[1] - info.origin.position.y) / info.resolution))
        if 0 <= x < info.width and 0 <= y < info.height:
            return x, y
        return None

    def _cell_to_world(self, cell: GridCell) -> WorldPoint:
        assert self.map_msg is not None
        info = self.map_msg.info
        return (
            info.origin.position.x + (cell[0] + 0.5) * info.resolution,
            info.origin.position.y + (cell[1] + 0.5) * info.resolution)

    @staticmethod
    def _distance(a: WorldPoint, b: WorldPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _publish_coverage_map(self) -> None:
        if self.map_msg is None or self.grid is None or self.covered is None:
            return
        output = OccupancyGrid()
        output.header = self.map_msg.header
        output.header.stamp = self.get_clock().now().to_msg()
        output.info = self.map_msg.info
        visual = np.full(self.grid.shape, -1, dtype=np.int8)
        visual[(self.grid >= 0) & (self.grid <= self.free_threshold)] = 0
        visual[self.grid >= self.occupied_threshold] = 100
        visual[self.covered & (visual == 0)] = 50
        output.data = visual.ravel().tolist()
        self.coverage_pub.publish(output)

    def _publish_plan(self, points: List[WorldPoint]) -> None:
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.plan_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OnlineCoverageNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()