# #!/usr/bin/env python3

# from collections import deque
# from dataclasses import dataclass
# import math
# from typing import List, Optional, Sequence, Tuple

# import numpy as np

# from action_msgs.msg import GoalStatus
# from geometry_msgs.msg import Point, PoseStamped
# from nav2_msgs.action import NavigateToPose
# from nav_msgs.msg import OccupancyGrid
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# from std_msgs.msg import ColorRGBA, String
# from tf2_ros import Buffer, TransformException, TransformListener
# from visualization_msgs.msg import Marker, MarkerArray


# GridCell = Tuple[int, int]
# WorldPoint = Tuple[float, float]


# @dataclass
# class FrontierCandidate:
#     utility: float
#     goal_cell: GridCell
#     goal_xy: WorldPoint
#     cluster: List[GridCell]
#     path_entropy: float
#     distance_m: float


# class ActiveSlamNode(Node):
#     def __init__(self):
#         super().__init__('active_slam_node')

#         self._declare_parameters()
#         self._read_parameters()

#         map_qos = QoSProfile(
#             depth=1,
#             reliability=ReliabilityPolicy.RELIABLE,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#         )
#         self.map_sub = self.create_subscription(
#             OccupancyGrid,
#             self.map_topic,
#             self._map_callback,
#             map_qos,
#         )

#         self.frontier_marker_pub = self.create_publisher(
#             MarkerArray,
#             self.marker_topic,
#             10,
#         )
#         self.selected_goal_pub = self.create_publisher(
#             PoseStamped,
#             self.selected_goal_topic,
#             10,
#         )
#         self.status_pub = self.create_publisher(String, self.status_topic, 10)

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

#         self.map_msg: Optional[OccupancyGrid] = None
#         self.goal_active = False
#         self.current_goal_xy: Optional[WorldPoint] = None
#         self.last_sent_goal_xy: Optional[WorldPoint] = None
#         self.last_goal_done_time = self.get_clock().now()
#         self.blacklisted_goals: List[Tuple[WorldPoint, float]] = []
#         self.last_selected_pose: Optional[PoseStamped] = None
#         self.last_candidate_rejections = {}
#         self._last_warn_time = {}

#         self.control_timer = self.create_timer(
#             self.control_period_sec,
#             self._control_loop,
#         )

#         self.get_logger().info(
#             'Active SLAM node started: map=%s, tf=%s->%s, nav2_action=%s'
#             % (
#                 self.map_topic,
#                 self.global_frame,
#                 self.robot_base_frame,
#                 self.nav2_action_name,
#             )
#         )

#     def _declare_parameters(self):
#         self.declare_parameter('map_topic', '/map')
#         self.declare_parameter('global_frame', 'map')
#         self.declare_parameter('robot_base_frame', 'base_footprint')
#         self.declare_parameter('nav2_action_name', '/navigate_to_pose')

#         self.declare_parameter('control_period_sec', 2.0)
#         self.declare_parameter('goal_cooldown_sec', 3.0)
#         self.declare_parameter('blacklist_duration_sec', 20.0)

#         self.declare_parameter('unknown_value', -1)
#         self.declare_parameter('free_max_value', 25)
#         self.declare_parameter('occupied_min_value', 65)

#         self.declare_parameter('min_frontier_cells', 8)
#         self.declare_parameter('min_goal_distance_m', 0.4)
#         self.declare_parameter('max_goal_distance_m', 8.0)
#         self.declare_parameter('goal_clearance_radius_m', 0.18)
#         self.declare_parameter('goal_backoff_m', 0.20)
#         self.declare_parameter('same_goal_tolerance_m', 0.30)

#         self.declare_parameter('lambda_distance', 0.25)
#         self.declare_parameter('entropy_weight', 1.0)
#         self.declare_parameter('frontier_size_weight', 0.2)
#         self.declare_parameter('frontier_size_norm_cells', 80.0)
#         self.declare_parameter('entropy_neighborhood_radius_cells', 0)

#         self.declare_parameter('publish_markers', True)
#         self.declare_parameter('marker_topic', '/active_slam/frontiers')
#         self.declare_parameter('selected_goal_topic', '/active_slam/selected_goal')
#         self.declare_parameter('status_topic', '/active_slam/status')

#     def _read_parameters(self):
#         self.map_topic = self.get_parameter('map_topic').value
#         self.global_frame = self.get_parameter('global_frame').value
#         self.robot_base_frame = self.get_parameter('robot_base_frame').value
#         self.nav2_action_name = self.get_parameter('nav2_action_name').value

#         self.control_period_sec = float(self.get_parameter('control_period_sec').value)
#         self.goal_cooldown_sec = float(self.get_parameter('goal_cooldown_sec').value)
#         self.blacklist_duration_sec = float(
#             self.get_parameter('blacklist_duration_sec').value
#         )

#         self.unknown_value = int(self.get_parameter('unknown_value').value)
#         self.free_max_value = int(self.get_parameter('free_max_value').value)
#         self.occupied_min_value = int(self.get_parameter('occupied_min_value').value)

#         self.min_frontier_cells = int(self.get_parameter('min_frontier_cells').value)
#         self.min_goal_distance_m = float(
#             self.get_parameter('min_goal_distance_m').value
#         )
#         self.max_goal_distance_m = float(
#             self.get_parameter('max_goal_distance_m').value
#         )
#         self.goal_clearance_radius_m = float(
#             self.get_parameter('goal_clearance_radius_m').value
#         )
#         self.goal_backoff_m = float(self.get_parameter('goal_backoff_m').value)
#         self.same_goal_tolerance_m = float(
#             self.get_parameter('same_goal_tolerance_m').value
#         )

#         self.lambda_distance = float(self.get_parameter('lambda_distance').value)
#         self.entropy_weight = float(self.get_parameter('entropy_weight').value)
#         self.frontier_size_weight = float(
#             self.get_parameter('frontier_size_weight').value
#         )
#         self.frontier_size_norm_cells = float(
#             self.get_parameter('frontier_size_norm_cells').value
#         )
#         self.entropy_neighborhood_radius_cells = int(
#             self.get_parameter('entropy_neighborhood_radius_cells').value
#         )

#         self.publish_markers = bool(self.get_parameter('publish_markers').value)
#         self.marker_topic = self.get_parameter('marker_topic').value
#         self.selected_goal_topic = self.get_parameter('selected_goal_topic').value
#         self.status_topic = self.get_parameter('status_topic').value

#     def _map_callback(self, msg: OccupancyGrid):
#         self.map_msg = msg

#     def _control_loop(self):
#         if self.map_msg is None:
#             self._publish_status('WAITING_FOR_MAP')
#             self._warn_throttled('map', 'Waiting for OccupancyGrid on %s' % self.map_topic)
#             return

#         robot_xy = self._lookup_robot_xy()
#         if robot_xy is None:
#             self._publish_status('WAITING_FOR_TF')
#             return

#         if self.goal_active:
#             self._publish_status('NAVIGATING')
#             return

#         if not self.nav_client.server_is_ready():
#             self._publish_status('WAITING_FOR_NAV2')
#             self._warn_throttled(
#                 'nav2',
#                 'Waiting for Nav2 action server %s' % self.nav2_action_name,
#             )
#             return

#         if not self._cooldown_elapsed():
#             self._publish_status('GOAL_COOLDOWN')
#             return

#         now_sec = self._now_seconds()
#         self.blacklisted_goals = [
#             (xy, expires_at)
#             for xy, expires_at in self.blacklisted_goals
#             if expires_at > now_sec
#         ]

#         grid = self._map_to_numpy(self.map_msg)
#         robot_cell = self._world_to_map(robot_xy[0], robot_xy[1], self.map_msg)
#         if robot_cell is None:
#             self._publish_status('ROBOT_OUT_OF_MAP')
#             return

#         frontier_mask = self._detect_frontier_mask(grid)
#         clusters = self._cluster_frontiers(frontier_mask)
#         candidates = self._build_candidates(grid, clusters, robot_xy, robot_cell)

#         if self.publish_markers:
#             self._publish_markers(clusters, candidates)

#         if not clusters:
#             self._publish_status('NO_FRONTIER_FOUND')
#             return

#         if not candidates:
#             self._publish_status(
#                 'NO_VALID_FRONTIER clusters=%d %s'
#                 % (len(clusters), self._format_rejections())
#             )
#             return

#         candidates.sort(key=lambda candidate: candidate.utility, reverse=True)
#         selected = self._select_candidate(candidates)
#         if selected is None:
#             self._publish_status('NO_NEW_FRONTIER_GOAL')
#             return

#         goal_pose = self._make_goal_pose(selected.goal_xy, robot_xy)
#         self.last_selected_pose = goal_pose
#         self.selected_goal_pub.publish(goal_pose)
#         self._send_nav_goal(goal_pose, selected)

#     def _lookup_robot_xy(self) -> Optional[WorldPoint]:
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 self.global_frame,
#                 self.robot_base_frame,
#                 rclpy.time.Time(),
#             )
#         except TransformException as exc:
#             self._warn_throttled(
#                 'tf',
#                 'Waiting for TF %s -> %s: %s'
#                 % (self.global_frame, self.robot_base_frame, exc),
#             )
#             return None

#         translation = transform.transform.translation
#         return translation.x, translation.y

#     def _map_to_numpy(self, msg: OccupancyGrid) -> np.ndarray:
#         return np.array(msg.data, dtype=np.int16).reshape(
#             (msg.info.height, msg.info.width)
#         )

#     def _detect_frontier_mask(self, grid: np.ndarray) -> np.ndarray:
#         free = (grid >= 0) & (grid <= self.free_max_value)
#         unknown = grid == self.unknown_value
#         padded_unknown = np.pad(unknown, 1, mode='constant', constant_values=False)

#         has_unknown_neighbor = np.zeros_like(unknown, dtype=bool)
#         for dy in (-1, 0, 1):
#             for dx in (-1, 0, 1):
#                 if dx == 0 and dy == 0:
#                     continue
#                 y0 = 1 + dy
#                 x0 = 1 + dx
#                 has_unknown_neighbor |= padded_unknown[
#                     y0:y0 + grid.shape[0],
#                     x0:x0 + grid.shape[1],
#                 ]

#         return free & has_unknown_neighbor

#     def _cluster_frontiers(self, frontier_mask: np.ndarray) -> List[List[GridCell]]:
#         height, width = frontier_mask.shape
#         visited = np.zeros_like(frontier_mask, dtype=bool)
#         clusters: List[List[GridCell]] = []

#         for y, x in np.argwhere(frontier_mask):
#             if visited[y, x]:
#                 continue

#             cluster: List[GridCell] = []
#             queue = deque([(int(x), int(y))])
#             visited[y, x] = True

#             while queue:
#                 cx, cy = queue.popleft()
#                 cluster.append((cx, cy))

#                 for ny in range(cy - 1, cy + 2):
#                     for nx in range(cx - 1, cx + 2):
#                         if nx == cx and ny == cy:
#                             continue
#                         if nx < 0 or ny < 0 or nx >= width or ny >= height:
#                             continue
#                         if visited[ny, nx] or not frontier_mask[ny, nx]:
#                             continue
#                         visited[ny, nx] = True
#                         queue.append((nx, ny))

#             if len(cluster) >= self.min_frontier_cells:
#                 clusters.append(cluster)

#         return clusters

#     def _build_candidates(
#         self,
#         grid: np.ndarray,
#         clusters: Sequence[List[GridCell]],
#         robot_xy: WorldPoint,
#         robot_cell: GridCell,
#     ) -> List[FrontierCandidate]:
#         candidates: List[FrontierCandidate] = []
#         rejections = {
#             'out_of_map': 0,
#             'no_free_snap': 0,
#             'unsafe': 0,
#             'blacklisted': 0,
#             'no_entropy': 0,
#             'duplicate': 0,
#         }
#         seen_goal_cells = set()

#         for cluster in clusters:
#             for representative_cell in self._cluster_seed_cells(cluster):
#                 representative_xy = self._map_to_world(
#                     representative_cell[0],
#                     representative_cell[1],
#                     self.map_msg,
#                 )
#                 backed_xy = self._backoff_goal(representative_xy, robot_xy)
#                 backed_cell = self._world_to_map(backed_xy[0], backed_xy[1], self.map_msg)
#                 if backed_cell is None:
#                     rejections['out_of_map'] += 1
#                     continue

#                 goal_cell = self._snap_to_free_cell(backed_cell, grid)
#                 if goal_cell is None:
#                     rejections['no_free_snap'] += 1
#                     continue
#                 if goal_cell in seen_goal_cells:
#                     rejections['duplicate'] += 1
#                     continue
#                 seen_goal_cells.add(goal_cell)

#                 goal_xy = self._map_to_world(goal_cell[0], goal_cell[1], self.map_msg)
#                 distance_m = math.hypot(goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1])
#                 if not self._is_goal_safe(goal_cell, grid, distance_m):
#                     rejections['unsafe'] += 1
#                     continue
#                 if self._is_blacklisted(goal_xy):
#                     rejections['blacklisted'] += 1
#                     continue

#                 path_cells = self._bresenham(
#                     robot_cell[0],
#                     robot_cell[1],
#                     goal_cell[0],
#                     goal_cell[1],
#                 )
#                 path_entropy = self._path_entropy(path_cells, grid)
#                 if path_entropy is None:
#                     rejections['no_entropy'] += 1
#                     continue

#                 distance_score = math.exp(-self.lambda_distance * distance_m)
#                 size_score = min(len(cluster) / self.frontier_size_norm_cells, 1.0)
#                 utility = (
#                     self.entropy_weight * path_entropy * distance_score
#                     + self.frontier_size_weight * size_score
#                 )
#                 candidates.append(
#                     FrontierCandidate(
#                         utility=utility,
#                         goal_cell=goal_cell,
#                         goal_xy=goal_xy,
#                         cluster=cluster,
#                         path_entropy=path_entropy,
#                         distance_m=distance_m,
#                     )
#                 )

#         self.last_candidate_rejections = rejections
#         return candidates

#     def _cluster_seed_cells(self, cluster: Sequence[GridCell]) -> List[GridCell]:
#         centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
#         centroid_y = sum(cell[1] for cell in cluster) / len(cluster)
#         sorted_cells = sorted(
#             cluster,
#             key=lambda cell: (cell[0] - centroid_x) ** 2 + (cell[1] - centroid_y) ** 2,
#         )
#         if len(sorted_cells) <= 24:
#             return list(sorted_cells)

#         step = max(1, len(sorted_cells) // 24)
#         return sorted_cells[:12] + sorted_cells[12::step][:12]

#     def _backoff_goal(self, frontier_xy: WorldPoint, robot_xy: WorldPoint) -> WorldPoint:
#         dx = frontier_xy[0] - robot_xy[0]
#         dy = frontier_xy[1] - robot_xy[1]
#         distance = math.hypot(dx, dy)
#         if distance < 1e-6 or self.goal_backoff_m <= 0.0:
#             return frontier_xy

#         scale = self.goal_backoff_m / distance
#         return frontier_xy[0] - dx * scale, frontier_xy[1] - dy * scale

#     def _snap_to_free_cell(
#         self,
#         start_cell: GridCell,
#         grid: np.ndarray,
#     ) -> Optional[GridCell]:
#         if self._in_bounds(start_cell, grid) and self._is_free_cell(start_cell, grid):
#             return start_cell

#         resolution = self.map_msg.info.resolution
#         max_radius = max(2, int(math.ceil(0.5 / resolution)))
#         sx, sy = start_cell

#         for radius in range(1, max_radius + 1):
#             cells = []
#             for y in range(sy - radius, sy + radius + 1):
#                 for x in range(sx - radius, sx + radius + 1):
#                     if max(abs(x - sx), abs(y - sy)) != radius:
#                         continue
#                     cells.append((x, y))
#             cells.sort(key=lambda cell: (cell[0] - sx) ** 2 + (cell[1] - sy) ** 2)

#             for cell in cells:
#                 if self._in_bounds(cell, grid) and self._is_free_cell(cell, grid):
#                     return cell

#         return None

#     def _is_goal_safe(
#         self,
#         goal_cell: GridCell,
#         grid: np.ndarray,
#         distance_m: float,
#     ) -> bool:
#         if distance_m < self.min_goal_distance_m:
#             return False
#         if distance_m > self.max_goal_distance_m:
#             return False
#         if not self._in_bounds(goal_cell, grid) or not self._is_free_cell(goal_cell, grid):
#             return False

#         resolution = self.map_msg.info.resolution
#         clearance_cells = int(math.ceil(self.goal_clearance_radius_m / resolution))
#         gx, gy = goal_cell
#         for y in range(gy - clearance_cells, gy + clearance_cells + 1):
#             for x in range(gx - clearance_cells, gx + clearance_cells + 1):
#                 if not self._in_bounds((x, y), grid):
#                     continue
#                 if math.hypot(x - gx, y - gy) * resolution > self.goal_clearance_radius_m:
#                     continue
#                 if grid[y, x] >= self.occupied_min_value:
#                     return False
#         return True

#     def _path_entropy(
#         self,
#         path_cells: Sequence[GridCell],
#         grid: np.ndarray,
#     ) -> Optional[float]:
#         entropy_values = []
#         radius = max(0, self.entropy_neighborhood_radius_cells)

#         for cell in path_cells:
#             if not self._in_bounds(cell, grid):
#                 return None
#             x, y = cell
#             value = int(grid[y, x])
#             if value != self.unknown_value and value >= self.occupied_min_value:
#                 continue

#             for ny in range(y - radius, y + radius + 1):
#                 for nx in range(x - radius, x + radius + 1):
#                     if not self._in_bounds((nx, ny), grid):
#                         continue
#                     neighbor_value = int(grid[ny, nx])
#                     if (
#                         neighbor_value != self.unknown_value
#                         and neighbor_value >= self.occupied_min_value
#                     ):
#                         continue
#                     entropy_values.append(self._cell_entropy(int(grid[ny, nx])))

#         if not entropy_values:
#             return None
#         return float(sum(entropy_values) / len(entropy_values))

#     def _format_rejections(self) -> str:
#         if not self.last_candidate_rejections:
#             return ''
#         parts = [
#             '%s=%d' % (key, value)
#             for key, value in self.last_candidate_rejections.items()
#             if value
#         ]
#         if not parts:
#             return 'rejects=none'
#         return 'rejects:' + ','.join(parts)

#     def _cell_entropy(self, value: int) -> float:
#         if value == self.unknown_value:
#             probability = 0.5
#         else:
#             probability = float(value) / 100.0

#         probability = min(max(probability, 1e-3), 1.0 - 1e-3)
#         entropy = (
#             -probability * math.log(probability)
#             - (1.0 - probability) * math.log(1.0 - probability)
#         )
#         return entropy / math.log(2.0)

#     def _select_candidate(
#         self,
#         candidates: Sequence[FrontierCandidate],
#     ) -> Optional[FrontierCandidate]:
#         for candidate in candidates:
#             if self.last_sent_goal_xy is None:
#                 return candidate
#             if self._distance(candidate.goal_xy, self.last_sent_goal_xy) >= (
#                 self.same_goal_tolerance_m
#             ):
#                 return candidate
#         return None

#     def _send_nav_goal(self, goal_pose: PoseStamped, candidate: FrontierCandidate):
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = goal_pose

#         self.goal_active = True
#         self.current_goal_xy = candidate.goal_xy
#         self.last_sent_goal_xy = candidate.goal_xy

#         self._publish_status(
#             'GOAL_SENT utility=%.3f entropy=%.3f distance=%.2f'
#             % (candidate.utility, candidate.path_entropy, candidate.distance_m)
#         )
#         self.get_logger().info(
#             'Sending Active SLAM goal x=%.2f y=%.2f utility=%.3f'
#             % (candidate.goal_xy[0], candidate.goal_xy[1], candidate.utility)
#         )

#         send_goal_future = self.nav_client.send_goal_async(goal_msg)
#         send_goal_future.add_done_callback(self._goal_response_callback)

#     def _goal_response_callback(self, future):
#         try:
#             goal_handle = future.result()
#         except Exception as exc:
#             self.get_logger().error('Failed to send Nav2 goal: %s' % exc)
#             self._blacklist_current_goal()
#             self.goal_active = False
#             self.current_goal_xy = None
#             self.last_goal_done_time = self.get_clock().now()
#             return

#         if not goal_handle.accepted:
#             self.get_logger().warn('Nav2 rejected Active SLAM goal')
#             self._blacklist_current_goal()
#             self.goal_active = False
#             self.current_goal_xy = None
#             self.last_goal_done_time = self.get_clock().now()
#             self._publish_status('GOAL_REJECTED')
#             return

#         self._publish_status('GOAL_ACCEPTED')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self._goal_result_callback)

#     def _goal_result_callback(self, future):
#         try:
#             result = future.result()
#         except Exception as exc:
#             self.get_logger().error('Failed while waiting for Nav2 result: %s' % exc)
#             self._blacklist_current_goal()
#             status = None
#         else:
#             status = result.status

#         if status == GoalStatus.STATUS_SUCCEEDED:
#             self.get_logger().info('Active SLAM goal succeeded')
#             self._publish_status('GOAL_SUCCEEDED')
#         else:
#             self.get_logger().warn('Active SLAM goal failed with status %s' % status)
#             self._blacklist_current_goal()
#             self._publish_status('GOAL_FAILED')

#         self.goal_active = False
#         self.current_goal_xy = None
#         self.last_goal_done_time = self.get_clock().now()

#     def _blacklist_current_goal(self):
#         if self.current_goal_xy is None:
#             return
#         expires_at = self._now_seconds() + self.blacklist_duration_sec
#         self.blacklisted_goals.append((self.current_goal_xy, expires_at))

#     def _is_blacklisted(self, goal_xy: WorldPoint) -> bool:
#         return any(
#             self._distance(goal_xy, blacklisted_xy) < self.same_goal_tolerance_m
#             for blacklisted_xy, _ in self.blacklisted_goals
#         )

#     def _cooldown_elapsed(self) -> bool:
#         elapsed = self.get_clock().now() - self.last_goal_done_time
#         return elapsed.nanoseconds / 1e9 >= self.goal_cooldown_sec

#     def _make_goal_pose(
#         self,
#         goal_xy: WorldPoint,
#         robot_xy: WorldPoint,
#     ) -> PoseStamped:
#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = self.global_frame
#         goal_pose.header.stamp = self.get_clock().now().to_msg()
#         goal_pose.pose.position.x = goal_xy[0]
#         goal_pose.pose.position.y = goal_xy[1]
#         goal_pose.pose.position.z = 0.0

#         yaw = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
#         goal_pose.pose.orientation.z = math.sin(yaw * 0.5)
#         goal_pose.pose.orientation.w = math.cos(yaw * 0.5)
#         return goal_pose

#     def _publish_markers(
#         self,
#         clusters: Sequence[List[GridCell]],
#         candidates: Sequence[FrontierCandidate],
#     ):
#         marker_array = MarkerArray()
#         delete_marker = Marker()
#         delete_marker.action = Marker.DELETEALL
#         marker_array.markers.append(delete_marker)

#         stamp = self.get_clock().now().to_msg()
#         resolution = self.map_msg.info.resolution

#         frontier_marker = Marker()
#         frontier_marker.header.frame_id = self.global_frame
#         frontier_marker.header.stamp = stamp
#         frontier_marker.ns = 'frontiers'
#         frontier_marker.id = 1
#         frontier_marker.type = Marker.CUBE_LIST
#         frontier_marker.action = Marker.ADD
#         frontier_marker.pose.orientation.w = 1.0
#         frontier_marker.scale.x = max(resolution, 0.04)
#         frontier_marker.scale.y = max(resolution, 0.04)
#         frontier_marker.scale.z = 0.04
#         frontier_marker.color = ColorRGBA(r=0.0, g=0.9, b=1.0, a=0.9)

#         for cluster in clusters:
#             for cell in cluster:
#                 x, y = self._map_to_world(cell[0], cell[1], self.map_msg)
#                 frontier_marker.points.append(Point(x=x, y=y, z=0.03))

#         marker_array.markers.append(frontier_marker)

#         candidate_marker = Marker()
#         candidate_marker.header.frame_id = self.global_frame
#         candidate_marker.header.stamp = stamp
#         candidate_marker.ns = 'candidate_goals'
#         candidate_marker.id = 2
#         candidate_marker.type = Marker.SPHERE_LIST
#         candidate_marker.action = Marker.ADD
#         candidate_marker.pose.orientation.w = 1.0
#         candidate_marker.scale.x = 0.12
#         candidate_marker.scale.y = 0.12
#         candidate_marker.scale.z = 0.12
#         candidate_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)

#         for candidate in candidates:
#             candidate_marker.points.append(
#                 Point(x=candidate.goal_xy[0], y=candidate.goal_xy[1], z=0.08)
#             )

#         marker_array.markers.append(candidate_marker)

#         if self.last_selected_pose is not None:
#             selected_marker = Marker()
#             selected_marker.header.frame_id = self.global_frame
#             selected_marker.header.stamp = stamp
#             selected_marker.ns = 'selected_goal'
#             selected_marker.id = 3
#             selected_marker.type = Marker.ARROW
#             selected_marker.action = Marker.ADD
#             selected_marker.pose = self.last_selected_pose.pose
#             selected_marker.scale.x = 0.35
#             selected_marker.scale.y = 0.08
#             selected_marker.scale.z = 0.08
#             selected_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
#             marker_array.markers.append(selected_marker)

#         self.frontier_marker_pub.publish(marker_array)

#     def _publish_status(self, status: str):
#         msg = String()
#         msg.data = status
#         self.status_pub.publish(msg)

#     def _warn_throttled(self, key: str, message: str, period_sec: float = 5.0):
#         now = self._now_seconds()
#         last = self._last_warn_time.get(key)
#         if last is None or now - last >= period_sec:
#             self.get_logger().warn(message)
#             self._last_warn_time[key] = now

#     def _world_to_map(
#         self,
#         x: float,
#         y: float,
#         msg: OccupancyGrid,
#     ) -> Optional[GridCell]:
#         origin = msg.info.origin.position
#         resolution = msg.info.resolution
#         mx = math.floor((x - origin.x) / resolution)
#         my = math.floor((y - origin.y) / resolution)
#         if mx < 0 or my < 0 or mx >= msg.info.width or my >= msg.info.height:
#             return None
#         return mx, my

#     def _map_to_world(
#         self,
#         mx: int,
#         my: int,
#         msg: OccupancyGrid,
#     ) -> WorldPoint:
#         origin = msg.info.origin.position
#         resolution = msg.info.resolution
#         x = origin.x + (mx + 0.5) * resolution
#         y = origin.y + (my + 0.5) * resolution
#         return x, y

#     def _in_bounds(self, cell: GridCell, grid: np.ndarray) -> bool:
#         x, y = cell
#         height, width = grid.shape
#         return 0 <= x < width and 0 <= y < height

#     def _is_free_cell(self, cell: GridCell, grid: np.ndarray) -> bool:
#         x, y = cell
#         value = grid[y, x]
#         return 0 <= value <= self.free_max_value

#     def _bresenham(
#         self,
#         x0: int,
#         y0: int,
#         x1: int,
#         y1: int,
#     ) -> List[GridCell]:
#         cells = []
#         dx = abs(x1 - x0)
#         dy = -abs(y1 - y0)
#         sx = 1 if x0 < x1 else -1
#         sy = 1 if y0 < y1 else -1
#         err = dx + dy
#         x, y = x0, y0

#         while True:
#             cells.append((x, y))
#             if x == x1 and y == y1:
#                 break
#             e2 = 2 * err
#             if e2 >= dy:
#                 err += dy
#                 x += sx
#             if e2 <= dx:
#                 err += dx
#                 y += sy

#         return cells

#     def _distance(self, a: WorldPoint, b: WorldPoint) -> float:
#         return math.hypot(a[0] - b[0], a[1] - b[1])

#     def _now_seconds(self) -> float:
#         return self.get_clock().now().nanoseconds / 1e9


# def main(args=None):
#     rclpy.init(args=args)
#     node = ActiveSlamNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

from collections import deque
from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass
class FrontierCandidate:
    utility: float
    goal_cell: GridCell
    goal_xy: WorldPoint
    cluster: List[GridCell]
    path_entropy: float
    distance_m: float


class ActiveSlamNode(Node):
    def __init__(self):
        super().__init__('active_slam_node')

        self._declare_parameters()
        self._read_parameters()

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )

        self.frontier_marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )
        self.selected_goal_pub = self.create_publisher(
            PoseStamped,
            self.selected_goal_topic,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav2_action_name,
        )

        self.map_msg: Optional[OccupancyGrid] = None

        self.goal_active = False
        self.goal_cancel_in_progress = False
        self.current_goal_xy: Optional[WorldPoint] = None
        self.current_goal_handle = None
        self.goal_start_time_sec: Optional[float] = None

        self.last_sent_goal_xy: Optional[WorldPoint] = None
        self.last_goal_done_time = self.get_clock().now()

        self.blacklisted_goals: List[Tuple[WorldPoint, float]] = []
        self.last_selected_pose: Optional[PoseStamped] = None
        self.last_candidate_rejections = {}
        self._last_warn_time = {}

        self.control_timer = self.create_timer(
            self.control_period_sec,
            self._control_loop,
        )

        self.get_logger().info(
            'Active SLAM node started: map=%s, tf=%s->%s, nav2_action=%s'
            % (
                self.map_topic,
                self.global_frame,
                self.robot_base_frame,
                self.nav2_action_name,
            )
        )

    def _declare_parameters(self):
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')

        self.declare_parameter('control_period_sec', 2.0)
        self.declare_parameter('goal_cooldown_sec', 2.0)
        self.declare_parameter('blacklist_duration_sec', 60.0)
        self.declare_parameter('blacklist_radius_m', 0.8)
        self.declare_parameter('nav_timeout_sec', 45.0)
        self.declare_parameter('min_free_cells_before_start', 200)

        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('free_max_value', 25)
        self.declare_parameter('occupied_min_value', 65)

        self.declare_parameter('min_frontier_cells', 15)
        self.declare_parameter('min_goal_distance_m', 0.8)
        self.declare_parameter('max_goal_distance_m', 8.0)
        self.declare_parameter('goal_clearance_radius_m', 0.22)
        self.declare_parameter('goal_backoff_m', 0.30)
        self.declare_parameter('same_goal_tolerance_m', 0.60)

        self.declare_parameter('lambda_distance', 0.10)
        self.declare_parameter('entropy_weight', 1.0)
        self.declare_parameter('frontier_size_weight', 0.05)
        self.declare_parameter('frontier_size_norm_cells', 100.0)
        self.declare_parameter('entropy_neighborhood_radius_cells', 2)

        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_topic', '/active_slam/frontiers')
        self.declare_parameter('selected_goal_topic', '/active_slam/selected_goal')
        self.declare_parameter('status_topic', '/active_slam/status')

    def _read_parameters(self):
        self.map_topic = self.get_parameter('map_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.nav2_action_name = self.get_parameter('nav2_action_name').value

        self.control_period_sec = float(
            self.get_parameter('control_period_sec').value
        )
        self.goal_cooldown_sec = float(
            self.get_parameter('goal_cooldown_sec').value
        )
        self.blacklist_duration_sec = float(
            self.get_parameter('blacklist_duration_sec').value
        )
        self.blacklist_radius_m = float(
            self.get_parameter('blacklist_radius_m').value
        )
        self.nav_timeout_sec = float(
            self.get_parameter('nav_timeout_sec').value
        )
        self.min_free_cells_before_start = int(
            self.get_parameter('min_free_cells_before_start').value
        )

        self.unknown_value = int(self.get_parameter('unknown_value').value)
        self.free_max_value = int(self.get_parameter('free_max_value').value)
        self.occupied_min_value = int(
            self.get_parameter('occupied_min_value').value
        )

        self.min_frontier_cells = int(
            self.get_parameter('min_frontier_cells').value
        )
        self.min_goal_distance_m = float(
            self.get_parameter('min_goal_distance_m').value
        )
        self.max_goal_distance_m = float(
            self.get_parameter('max_goal_distance_m').value
        )
        self.goal_clearance_radius_m = float(
            self.get_parameter('goal_clearance_radius_m').value
        )
        self.goal_backoff_m = float(
            self.get_parameter('goal_backoff_m').value
        )
        self.same_goal_tolerance_m = float(
            self.get_parameter('same_goal_tolerance_m').value
        )

        self.lambda_distance = float(
            self.get_parameter('lambda_distance').value
        )
        self.entropy_weight = float(
            self.get_parameter('entropy_weight').value
        )
        self.frontier_size_weight = float(
            self.get_parameter('frontier_size_weight').value
        )
        self.frontier_size_norm_cells = float(
            self.get_parameter('frontier_size_norm_cells').value
        )
        self.entropy_neighborhood_radius_cells = int(
            self.get_parameter('entropy_neighborhood_radius_cells').value
        )

        self.publish_markers = bool(
            self.get_parameter('publish_markers').value
        )
        self.marker_topic = self.get_parameter('marker_topic').value
        self.selected_goal_topic = self.get_parameter(
            'selected_goal_topic'
        ).value
        self.status_topic = self.get_parameter('status_topic').value

    def _map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _control_loop(self):
        if self.map_msg is None:
            self._publish_status('WAITING_FOR_MAP')
            self._warn_throttled(
                'map',
                'Waiting for OccupancyGrid on %s' % self.map_topic,
            )
            return

        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            self._publish_status('WAITING_FOR_TF')
            return

        if self.goal_active:
            if self._nav_timeout_elapsed():
                if not self.goal_cancel_in_progress:
                    self._publish_status('GOAL_TIMEOUT_CANCELING')
                    self.get_logger().warn(
                        'Nav2 goal timeout. Canceling and blacklisting current goal.'
                    )
                    self._cancel_current_goal()
                else:
                    self._publish_status('GOAL_CANCELING')
            else:
                self._publish_status('NAVIGATING')
            return

        if not self.nav_client.server_is_ready():
            self._publish_status('WAITING_FOR_NAV2')
            self._warn_throttled(
                'nav2',
                'Waiting for Nav2 action server %s' % self.nav2_action_name,
            )
            return

        if not self._cooldown_elapsed():
            self._publish_status('GOAL_COOLDOWN')
            return

        now_sec = self._now_seconds()
        self.blacklisted_goals = [
            (xy, expires_at)
            for xy, expires_at in self.blacklisted_goals
            if expires_at > now_sec
        ]

        grid = self._map_to_numpy(self.map_msg)

        free_cells = int(np.sum((grid >= 0) & (grid <= self.free_max_value)))
        if free_cells < self.min_free_cells_before_start:
            self._publish_status(
                'WAITING_FOR_LARGER_MAP free_cells=%d' % free_cells
            )
            return

        robot_cell = self._world_to_map(
            robot_xy[0],
            robot_xy[1],
            self.map_msg,
        )
        if robot_cell is None:
            self._publish_status('ROBOT_OUT_OF_MAP')
            return

        frontier_mask = self._detect_frontier_mask(grid)
        clusters = self._cluster_frontiers(frontier_mask)
        candidates = self._build_candidates(
            grid,
            clusters,
            robot_xy,
            robot_cell,
        )

        if self.publish_markers:
            self._publish_markers(clusters, candidates)

        if not clusters:
            self._publish_status('NO_FRONTIER_FOUND')
            return

        if not candidates:
            self._publish_status(
                'NO_VALID_FRONTIER clusters=%d %s'
                % (len(clusters), self._format_rejections())
            )
            return

        candidates.sort(key=lambda candidate: candidate.utility, reverse=True)
        self._log_top_candidates(candidates)

        selected = self._select_candidate(candidates)
        if selected is None:
            self._publish_status('NO_NEW_FRONTIER_GOAL')
            return

        goal_pose = self._make_goal_pose(selected.goal_xy, robot_xy)
        self.last_selected_pose = goal_pose
        self.selected_goal_pub.publish(goal_pose)

        self._send_nav_goal(goal_pose, selected)

    def _lookup_robot_xy(self) -> Optional[WorldPoint]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                'tf',
                'Waiting for TF %s -> %s: %s'
                % (self.global_frame, self.robot_base_frame, exc),
            )
            return None

        translation = transform.transform.translation
        return translation.x, translation.y

    def _map_to_numpy(self, msg: OccupancyGrid) -> np.ndarray:
        return np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

    def _detect_frontier_mask(self, grid: np.ndarray) -> np.ndarray:
        free = (grid >= 0) & (grid <= self.free_max_value)
        unknown = grid == self.unknown_value

        padded_unknown = np.pad(
            unknown,
            1,
            mode='constant',
            constant_values=False,
        )

        has_unknown_neighbor = np.zeros_like(unknown, dtype=bool)

        # Dùng 4-neighbor để giảm frontier nhiễu ở góc chéo.
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            y0 = 1 + dy
            x0 = 1 + dx
            has_unknown_neighbor |= padded_unknown[
                y0:y0 + grid.shape[0],
                x0:x0 + grid.shape[1],
            ]

        return free & has_unknown_neighbor

    def _cluster_frontiers(
        self,
        frontier_mask: np.ndarray,
    ) -> List[List[GridCell]]:
        height, width = frontier_mask.shape
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters: List[List[GridCell]] = []

        for y, x in np.argwhere(frontier_mask):
            if visited[y, x]:
                continue

            cluster: List[GridCell] = []
            queue = deque([(int(x), int(y))])
            visited[y, x] = True

            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))

                # Gom cụm vẫn dùng 8-connected để các frontier gần nhau không bị vỡ vụn.
                for ny in range(cy - 1, cy + 2):
                    for nx in range(cx - 1, cx + 2):
                        if nx == cx and ny == cy:
                            continue
                        if nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                        if visited[ny, nx] or not frontier_mask[ny, nx]:
                            continue

                        visited[ny, nx] = True
                        queue.append((nx, ny))

            if len(cluster) >= self.min_frontier_cells:
                clusters.append(cluster)

        return clusters

    def _build_candidates(
        self,
        grid: np.ndarray,
        clusters: Sequence[List[GridCell]],
        robot_xy: WorldPoint,
        robot_cell: GridCell,
    ) -> List[FrontierCandidate]:
        candidates: List[FrontierCandidate] = []
        rejections = {
            'out_of_map': 0,
            'no_free_snap': 0,
            'unsafe': 0,
            'too_close': 0,
            'too_far': 0,
            'unsafe_clearance': 0,
            'blacklisted': 0,
            'no_entropy': 0,
            'duplicate': 0,
        }
        seen_goal_cells = set()

        for cluster in clusters:
            for representative_cell in self._cluster_seed_cells(cluster):
                representative_xy = self._map_to_world(
                    representative_cell[0],
                    representative_cell[1],
                    self.map_msg,
                )

                backed_xy = self._backoff_goal(representative_xy, robot_xy)
                backed_cell = self._world_to_map(
                    backed_xy[0],
                    backed_xy[1],
                    self.map_msg,
                )
                if backed_cell is None:
                    rejections['out_of_map'] += 1
                    continue

                goal_cell = self._snap_to_free_cell(backed_cell, grid)
                if goal_cell is None:
                    rejections['no_free_snap'] += 1
                    continue

                if goal_cell in seen_goal_cells:
                    rejections['duplicate'] += 1
                    continue
                seen_goal_cells.add(goal_cell)

                goal_xy = self._map_to_world(
                    goal_cell[0],
                    goal_cell[1],
                    self.map_msg,
                )

                distance_m = math.hypot(
                    goal_xy[0] - robot_xy[0],
                    goal_xy[1] - robot_xy[1],
                )

                rejection_reason = self._goal_safety_rejection(
                    goal_cell,
                    grid,
                    distance_m,
                )
                if rejection_reason is not None:
                    rejections[rejection_reason] += 1
                    continue

                if self._is_blacklisted(goal_xy):
                    rejections['blacklisted'] += 1
                    continue

                path_cells = self._bresenham(
                    robot_cell[0],
                    robot_cell[1],
                    goal_cell[0],
                    goal_cell[1],
                )

                path_entropy = self._path_entropy(path_cells, grid)
                if path_entropy is None:
                    rejections['no_entropy'] += 1
                    continue

                distance_score = math.exp(-self.lambda_distance * distance_m)
                size_score = min(
                    len(cluster) / self.frontier_size_norm_cells,
                    1.0,
                )

                utility = (
                    self.entropy_weight * path_entropy * distance_score
                    + self.frontier_size_weight * size_score
                )

                candidates.append(
                    FrontierCandidate(
                        utility=utility,
                        goal_cell=goal_cell,
                        goal_xy=goal_xy,
                        cluster=cluster,
                        path_entropy=path_entropy,
                        distance_m=distance_m,
                    )
                )

        self.last_candidate_rejections = rejections
        return candidates

    def _cluster_seed_cells(self, cluster: Sequence[GridCell]) -> List[GridCell]:
        centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
        centroid_y = sum(cell[1] for cell in cluster) / len(cluster)

        sorted_cells = sorted(
            cluster,
            key=lambda cell: (
                (cell[0] - centroid_x) ** 2
                + (cell[1] - centroid_y) ** 2
            ),
        )

        if len(sorted_cells) <= 24:
            return list(sorted_cells)

        step = max(1, len(sorted_cells) // 24)
        return sorted_cells[:12] + sorted_cells[12::step][:12]

    def _backoff_goal(
        self,
        frontier_xy: WorldPoint,
        robot_xy: WorldPoint,
    ) -> WorldPoint:
        dx = frontier_xy[0] - robot_xy[0]
        dy = frontier_xy[1] - robot_xy[1]
        distance = math.hypot(dx, dy)

        if distance < 1e-6 or self.goal_backoff_m <= 0.0:
            return frontier_xy

        scale = self.goal_backoff_m / distance
        return frontier_xy[0] - dx * scale, frontier_xy[1] - dy * scale

    def _snap_to_free_cell(
        self,
        start_cell: GridCell,
        grid: np.ndarray,
    ) -> Optional[GridCell]:
        if (
            self._in_bounds(start_cell, grid)
            and self._is_free_cell(start_cell, grid)
            and self._has_occupied_clearance(
                start_cell,
                grid,
                self.goal_clearance_radius_m,
            )
        ):
            return start_cell

        resolution = self.map_msg.info.resolution
        max_radius = max(2, int(math.ceil(0.8 / resolution)))
        sx, sy = start_cell

        for radius in range(1, max_radius + 1):
            cells = []

            for y in range(sy - radius, sy + radius + 1):
                for x in range(sx - radius, sx + radius + 1):
                    if max(abs(x - sx), abs(y - sy)) != radius:
                        continue
                    cells.append((x, y))

            cells.sort(
                key=lambda cell: (
                    (cell[0] - sx) ** 2
                    + (cell[1] - sy) ** 2
                )
            )

            for cell in cells:
                if (
                    self._in_bounds(cell, grid)
                    and self._is_free_cell(cell, grid)
                    and self._has_occupied_clearance(
                        cell,
                        grid,
                        self.goal_clearance_radius_m,
                    )
                ):
                    return cell

        return None

    def _has_occupied_clearance(
        self,
        cell: GridCell,
        grid: np.ndarray,
        clearance_m: float,
    ) -> bool:
        resolution = self.map_msg.info.resolution
        clearance_cells = int(math.ceil(clearance_m / resolution))
        gx, gy = cell

        for y in range(gy - clearance_cells, gy + clearance_cells + 1):
            for x in range(gx - clearance_cells, gx + clearance_cells + 1):
                if not self._in_bounds((x, y), grid):
                    continue

                if math.hypot(x - gx, y - gy) * resolution > clearance_m:
                    continue

                if grid[y, x] >= self.occupied_min_value:
                    return False

        return True

    def _is_goal_safe(
        self,
        goal_cell: GridCell,
        grid: np.ndarray,
        distance_m: float,
    ) -> bool:
        return self._goal_safety_rejection(goal_cell, grid, distance_m) is None

    def _goal_safety_rejection(
        self,
        goal_cell: GridCell,
        grid: np.ndarray,
        distance_m: float,
    ) -> Optional[str]:
        if distance_m < self.min_goal_distance_m:
            return 'too_close'

        if distance_m > self.max_goal_distance_m:
            return 'too_far'

        if not self._in_bounds(goal_cell, grid):
            return 'unsafe'

        if not self._is_free_cell(goal_cell, grid):
            return 'unsafe'

        if not self._has_occupied_clearance(
            goal_cell,
            grid,
            self.goal_clearance_radius_m,
        ):
            return 'unsafe_clearance'

        return None

    def _path_entropy(
        self,
        path_cells: Sequence[GridCell],
        grid: np.ndarray,
    ) -> Optional[float]:
        entropy_values = []
        radius = max(0, self.entropy_neighborhood_radius_cells)

        for cell in path_cells:
            if not self._in_bounds(cell, grid):
                return None

            x, y = cell
            value = int(grid[y, x])

            # Nếu đường thẳng robot -> goal cắt qua obstacle, loại candidate.
            if value != self.unknown_value and value >= self.occupied_min_value:
                return None

            for ny in range(y - radius, y + radius + 1):
                for nx in range(x - radius, x + radius + 1):
                    if not self._in_bounds((nx, ny), grid):
                        continue

                    neighbor_value = int(grid[ny, nx])

                    # Không cộng entropy của obstacle quanh line.
                    if (
                        neighbor_value != self.unknown_value
                        and neighbor_value >= self.occupied_min_value
                    ):
                        continue

                    entropy_values.append(
                        self._cell_entropy(neighbor_value)
                    )

        if not entropy_values:
            return None

        return float(sum(entropy_values) / len(entropy_values))

    def _cell_entropy(self, value: int) -> float:
        if value == self.unknown_value:
            probability = 0.5
        else:
            probability = float(value) / 100.0

        probability = min(max(probability, 1e-3), 1.0 - 1e-3)

        entropy = (
            -probability * math.log(probability)
            - (1.0 - probability) * math.log(1.0 - probability)
        )

        # H(0.5) = log(2), nên giá trị trả về nằm xấp xỉ [0, 1].
        return entropy / math.log(2.0)

    def _select_candidate(
        self,
        candidates: Sequence[FrontierCandidate],
    ) -> Optional[FrontierCandidate]:
        for candidate in candidates:
            if self.last_sent_goal_xy is None:
                return candidate

            if (
                self._distance(candidate.goal_xy, self.last_sent_goal_xy)
                >= self.same_goal_tolerance_m
            ):
                return candidate

        return None

    def _send_nav_goal(
        self,
        goal_pose: PoseStamped,
        candidate: FrontierCandidate,
    ):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.goal_active = True
        self.goal_cancel_in_progress = False
        self.current_goal_xy = candidate.goal_xy
        self.current_goal_handle = None
        self.goal_start_time_sec = None
        self.last_sent_goal_xy = candidate.goal_xy

        self._publish_status(
            'GOAL_SENT utility=%.3f entropy=%.3f distance=%.2f'
            % (
                candidate.utility,
                candidate.path_entropy,
                candidate.distance_m,
            )
        )

        self.get_logger().info(
            'Sending Active SLAM goal x=%.2f y=%.2f utility=%.3f entropy=%.3f distance=%.2f'
            % (
                candidate.goal_xy[0],
                candidate.goal_xy[1],
                candidate.utility,
                candidate.path_entropy,
                candidate.distance_m,
            )
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error('Failed to send Nav2 goal: %s' % exc)
            self._blacklist_current_goal()
            self._reset_goal_state()
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected Active SLAM goal')
            self._blacklist_current_goal()
            self._reset_goal_state()
            self.last_goal_done_time = self.get_clock().now()
            self._publish_status('GOAL_REJECTED')
            return

        self.current_goal_handle = goal_handle
        self.goal_start_time_sec = self._now_seconds()

        self._publish_status('GOAL_ACCEPTED')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.get_logger().error(
                'Failed while waiting for Nav2 result: %s' % exc
            )
            status = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Active SLAM goal succeeded')
            self._publish_status('GOAL_SUCCEEDED')
        else:
            self.get_logger().warn(
                'Active SLAM goal failed with status %s' % status
            )
            self._blacklist_current_goal()
            self._publish_status('GOAL_FAILED_BLACKLISTED')

        self._reset_goal_state()
        self.last_goal_done_time = self.get_clock().now()

    def _nav_timeout_elapsed(self) -> bool:
        if not self.goal_active:
            return False

        if self.goal_start_time_sec is None:
            return False

        return (
            self._now_seconds() - self.goal_start_time_sec
            >= self.nav_timeout_sec
        )

    def _cancel_current_goal(self):
        if self.goal_cancel_in_progress:
            return

        self.goal_cancel_in_progress = True

        if self.current_goal_handle is None:
            self._blacklist_current_goal()
            self._publish_status('GOAL_TIMEOUT_BLACKLISTED')
            self._reset_goal_state()
            self.last_goal_done_time = self.get_clock().now()
            return

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_goal_done_callback)

    def _cancel_goal_done_callback(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warn('Failed to cancel Nav2 goal: %s' % exc)

        self._blacklist_current_goal()
        self._publish_status('GOAL_TIMEOUT_BLACKLISTED')

        self._reset_goal_state()
        self.last_goal_done_time = self.get_clock().now()

    def _blacklist_current_goal(self):
        if self.current_goal_xy is None:
            return

        expires_at = self._now_seconds() + self.blacklist_duration_sec
        self.blacklisted_goals.append((self.current_goal_xy, expires_at))

        self.get_logger().warn(
            'Blacklisted goal x=%.2f y=%.2f for %.1f sec'
            % (
                self.current_goal_xy[0],
                self.current_goal_xy[1],
                self.blacklist_duration_sec,
            )
        )

    def _is_blacklisted(self, goal_xy: WorldPoint) -> bool:
        return any(
            self._distance(goal_xy, blacklisted_xy) < self.blacklist_radius_m
            for blacklisted_xy, _ in self.blacklisted_goals
        )

    def _reset_goal_state(self):
        self.goal_active = False
        self.goal_cancel_in_progress = False
        self.current_goal_xy = None
        self.current_goal_handle = None
        self.goal_start_time_sec = None

    def _cooldown_elapsed(self) -> bool:
        elapsed = self.get_clock().now() - self.last_goal_done_time
        return elapsed.nanoseconds / 1e9 >= self.goal_cooldown_sec

    def _make_goal_pose(
        self,
        goal_xy: WorldPoint,
        robot_xy: WorldPoint,
    ) -> PoseStamped:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = goal_xy[0]
        goal_pose.pose.position.y = goal_xy[1]
        goal_pose.pose.position.z = 0.0

        yaw = math.atan2(
            goal_xy[1] - robot_xy[1],
            goal_xy[0] - robot_xy[0],
        )

        goal_pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal_pose.pose.orientation.w = math.cos(yaw * 0.5)

        return goal_pose

    def _publish_markers(
        self,
        clusters: Sequence[List[GridCell]],
        candidates: Sequence[FrontierCandidate],
    ):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        stamp = self.get_clock().now().to_msg()
        resolution = self.map_msg.info.resolution

        frontier_marker = Marker()
        frontier_marker.header.frame_id = self.global_frame
        frontier_marker.header.stamp = stamp
        frontier_marker.ns = 'frontiers'
        frontier_marker.id = 1
        frontier_marker.type = Marker.CUBE_LIST
        frontier_marker.action = Marker.ADD
        frontier_marker.pose.orientation.w = 1.0
        frontier_marker.scale.x = max(resolution, 0.04)
        frontier_marker.scale.y = max(resolution, 0.04)
        frontier_marker.scale.z = 0.04
        frontier_marker.color = ColorRGBA(r=0.0, g=0.9, b=1.0, a=0.9)

        for cluster in clusters:
            for cell in cluster:
                x, y = self._map_to_world(cell[0], cell[1], self.map_msg)
                frontier_marker.points.append(Point(x=x, y=y, z=0.03))

        marker_array.markers.append(frontier_marker)

        candidate_marker = Marker()
        candidate_marker.header.frame_id = self.global_frame
        candidate_marker.header.stamp = stamp
        candidate_marker.ns = 'candidate_goals'
        candidate_marker.id = 2
        candidate_marker.type = Marker.SPHERE_LIST
        candidate_marker.action = Marker.ADD
        candidate_marker.pose.orientation.w = 1.0
        candidate_marker.scale.x = 0.12
        candidate_marker.scale.y = 0.12
        candidate_marker.scale.z = 0.12
        candidate_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)

        for candidate in candidates:
            candidate_marker.points.append(
                Point(
                    x=candidate.goal_xy[0],
                    y=candidate.goal_xy[1],
                    z=0.08,
                )
            )

        marker_array.markers.append(candidate_marker)

        if self.last_selected_pose is not None:
            selected_marker = Marker()
            selected_marker.header.frame_id = self.global_frame
            selected_marker.header.stamp = stamp
            selected_marker.ns = 'selected_goal'
            selected_marker.id = 3
            selected_marker.type = Marker.ARROW
            selected_marker.action = Marker.ADD
            selected_marker.pose = self.last_selected_pose.pose
            selected_marker.scale.x = 0.35
            selected_marker.scale.y = 0.08
            selected_marker.scale.z = 0.08
            selected_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
            marker_array.markers.append(selected_marker)

        blacklisted_marker = Marker()
        blacklisted_marker.header.frame_id = self.global_frame
        blacklisted_marker.header.stamp = stamp
        blacklisted_marker.ns = 'blacklisted_goals'
        blacklisted_marker.id = 4
        blacklisted_marker.type = Marker.SPHERE_LIST
        blacklisted_marker.action = Marker.ADD
        blacklisted_marker.pose.orientation.w = 1.0
        blacklisted_marker.scale.x = 0.18
        blacklisted_marker.scale.y = 0.18
        blacklisted_marker.scale.z = 0.18
        blacklisted_marker.color = ColorRGBA(r=0.9, g=0.0, b=0.9, a=0.8)

        now_sec = self._now_seconds()
        for blacklisted_xy, expires_at in self.blacklisted_goals:
            if expires_at <= now_sec:
                continue
            blacklisted_marker.points.append(
                Point(
                    x=blacklisted_xy[0],
                    y=blacklisted_xy[1],
                    z=0.10,
                )
            )

        marker_array.markers.append(blacklisted_marker)

        self.frontier_marker_pub.publish(marker_array)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _warn_throttled(
        self,
        key: str,
        message: str,
        period_sec: float = 5.0,
    ):
        now = self._now_seconds()
        last = self._last_warn_time.get(key)

        if last is None or now - last >= period_sec:
            self.get_logger().warn(message)
            self._last_warn_time[key] = now

    def _log_top_candidates(
        self,
        candidates: Sequence[FrontierCandidate],
        limit: int = 5,
    ):
        if not candidates:
            return

        parts = []

        for i, candidate in enumerate(candidates[:limit]):
            parts.append(
                '#%d u=%.3f H=%.3f d=%.2f size=%d goal=(%.2f,%.2f)'
                % (
                    i + 1,
                    candidate.utility,
                    candidate.path_entropy,
                    candidate.distance_m,
                    len(candidate.cluster),
                    candidate.goal_xy[0],
                    candidate.goal_xy[1],
                )
            )

        self.get_logger().info(
            'Top frontier candidates: ' + ' | '.join(parts)
        )

    def _format_rejections(self) -> str:
        if not self.last_candidate_rejections:
            return ''

        parts = [
            '%s=%d' % (key, value)
            for key, value in self.last_candidate_rejections.items()
            if value
        ]

        if not parts:
            return 'rejects=none'

        return 'rejects:' + ','.join(parts)

    def _world_to_map(
        self,
        x: float,
        y: float,
        msg: OccupancyGrid,
    ) -> Optional[GridCell]:
        origin = msg.info.origin.position
        resolution = msg.info.resolution

        mx = math.floor((x - origin.x) / resolution)
        my = math.floor((y - origin.y) / resolution)

        if mx < 0 or my < 0:
            return None

        if mx >= msg.info.width or my >= msg.info.height:
            return None

        return mx, my

    def _map_to_world(
        self,
        mx: int,
        my: int,
        msg: OccupancyGrid,
    ) -> WorldPoint:
        origin = msg.info.origin.position
        resolution = msg.info.resolution

        x = origin.x + (mx + 0.5) * resolution
        y = origin.y + (my + 0.5) * resolution

        return x, y

    def _in_bounds(self, cell: GridCell, grid: np.ndarray) -> bool:
        x, y = cell
        height, width = grid.shape

        return 0 <= x < width and 0 <= y < height

    def _is_free_cell(self, cell: GridCell, grid: np.ndarray) -> bool:
        x, y = cell
        value = grid[y, x]

        return 0 <= value <= self.free_max_value

    def _bresenham(
        self,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ) -> List[GridCell]:
        cells = []

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        x, y = x0, y0

        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 >= dy:
                err += dy
                x += sx

            if e2 <= dx:
                err += dx
                y += sy

        return cells

    def _distance(self, a: WorldPoint, b: WorldPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)

    node = ActiveSlamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
