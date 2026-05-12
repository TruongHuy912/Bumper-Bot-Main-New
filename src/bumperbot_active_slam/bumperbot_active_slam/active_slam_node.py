#!/usr/bin/env python3

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt, uniform_filter
except ImportError:
    distance_transform_edt = None
    uniform_filter = None

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
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
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.tf_buffer = Buffer()
        try:
            self.tf_listener = TransformListener(
                self.tf_buffer,
                self,
                spin_thread=True,
            )
        except TypeError:
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().warn(
                'TransformListener spin_thread is not supported; using default listener.'
            )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav2_action_name,
        )
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_action_name,
        )
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.clear_global_costmap_service,
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            self.clear_local_costmap_service,
        )

        self.map_msg: Optional[OccupancyGrid] = None

        self.goal_active = False
        self.goal_cancel_in_progress = False
        self.current_goal_xy: Optional[WorldPoint] = None
        self.current_goal_handle = None
        self.goal_start_time_sec: Optional[float] = None
        self.path_check_active = False
        self.pending_goal_pose: Optional[PoseStamped] = None
        self.pending_candidate: Optional[FrontierCandidate] = None
        self.recovery_spin_until_sec = 0.0
        self.recovery_spin_active = False
        self.last_too_close_recovery_time_sec = 0.0
        self.consecutive_too_close_spins = 0
        self.path_failure_times: List[float] = []
        self.path_failure_pause_until_sec = 0.0
        self.pending_candidates_queue: List[FrontierCandidate] = []
        self.path_check_attempts_this_cycle = 0
        self.pending_candidate_index = 0
        self.visited_goals: List[Tuple[WorldPoint, float]] = []
        self.last_progress_xy: Optional[WorldPoint] = None
        self.last_progress_time_sec: Optional[float] = None
        self.run_recovery_after_cancel = False
        self.costmap_recovery_state = 'IDLE'
        self.costmap_recovery_until_sec = 0.0
        self.costmap_recovery_reason = ''
        self.last_robot_xy: Optional[WorldPoint] = None
        self.last_robot_pose_time_sec: Optional[float] = None
        self.map_helpers = {}
        self.last_timing_log_time_sec = 0.0

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
            'Active SLAM node started: map=%s, global_frame=%s, '
            'robot_base_frame=%s, nav2_action=%s'
            % (
                self.map_topic,
                self.global_frame,
                self.robot_base_frame,
                self.nav2_action_name,
            )
        )
        self.get_logger().info(
            'Active SLAM params: min_goal_distance_m=%.2f, '
            'max_goal_distance_m=%.2f, goal_clearance_radius_m=%.2f, '
            'unknown_clearance_radius_m=%.2f, goal_backoff_m=%.2f, '
            'blacklist_radius_m=%.2f, lambda_distance=%.3f, '
            'entropy_neighborhood_radius_cells=%d, compute_path_check=%s'
            % (
                self.min_goal_distance_m,
                self.max_goal_distance_m,
                self.goal_clearance_radius_m,
                self.unknown_clearance_radius_m,
                self.goal_backoff_m,
                self.blacklist_radius_m,
                self.lambda_distance,
                self.entropy_neighborhood_radius_cells,
                self.enable_compute_path_check,
            )
        )
        if self.nav_timeout_sec <= 0.0:
            self.get_logger().info(
                'Hard navigation timeout disabled; waiting for Nav2 result '
                'or stuck watchdog.'
            )

    def _declare_parameters(self):
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('nav2_action_name', '/navigate_to_pose')
        self.declare_parameter('compute_path_action_name', '/compute_path_to_pose')
        self.declare_parameter('enable_compute_path_check', True)
        self.declare_parameter('min_path_poses', 2)
        self.declare_parameter('tf_lookup_timeout_sec', 0.05)
        self.declare_parameter('use_last_pose_on_tf_failure', True)
        self.declare_parameter('max_cached_pose_age_sec', 2.0)
        self.declare_parameter('enable_recovery_spin', False)
        self.declare_parameter('enable_too_close_recovery_spin', False)
        self.declare_parameter('too_close_recovery_cooldown_sec', 15.0)
        self.declare_parameter('max_consecutive_too_close_spins', 1)
        self.declare_parameter('recovery_spin_duration_sec', 1.5)
        self.declare_parameter('recovery_spin_angular_vel', 0.15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('too_close_recovery_threshold', 999)
        self.declare_parameter('path_failure_threshold', 3)
        self.declare_parameter('path_failure_window_sec', 30.0)
        self.declare_parameter('path_failure_pause_sec', 0.0)
        self.declare_parameter('max_path_check_attempts_per_cycle', 3)
        self.declare_parameter('enable_timing_debug', True)
        self.declare_parameter('timing_log_period_sec', 5.0)
        self.declare_parameter('max_clusters_to_score', 8)
        self.declare_parameter('max_seed_cells_per_cluster', 6)
        self.declare_parameter('max_total_candidates', 40)
        self.declare_parameter('adaptive_goal_distance_enabled', True)
        self.declare_parameter('max_goal_distance_hard_m', 8.0)
        self.declare_parameter('goal_distance_expand_step_m', 1.0)
        self.declare_parameter('cluster_distance_weight', 0.7)
        self.declare_parameter('cluster_size_weight', 0.3)
        self.declare_parameter('cluster_distance_sample_count', 20)
        self.declare_parameter('visited_goal_radius_m', 0.80)
        self.declare_parameter('visited_goal_duration_sec', 180.0)
        self.declare_parameter('max_visited_goals', 50)
        self.declare_parameter('nav_xy_goal_tolerance_m', 0.20)
        self.declare_parameter('min_goal_distance_margin_m', 0.25)
        self.declare_parameter('enable_stuck_watchdog', True)
        self.declare_parameter('stuck_check_radius_m', 0.15)
        self.declare_parameter('stuck_timeout_sec', 60.0)
        self.declare_parameter('enable_costmap_recovery', True)
        self.declare_parameter(
            'clear_global_costmap_service',
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.declare_parameter(
            'clear_local_costmap_service',
            '/local_costmap/clear_entirely_local_costmap',
        )
        self.declare_parameter('recovery_backup_duration_sec', 0.0)
        self.declare_parameter('recovery_backup_linear_vel', -0.03)
        self.declare_parameter('recovery_wait_after_clear_sec', 1.0)

        self.declare_parameter('control_period_sec', 1.0)
        self.declare_parameter('goal_cooldown_sec', 1.0)
        self.declare_parameter('blacklist_duration_sec', 45.0)
        self.declare_parameter('blacklist_radius_m', 0.60)
        self.declare_parameter('nav_timeout_sec', 0.0)
        self.declare_parameter('min_free_cells_before_start', 200)

        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('free_max_value', 25)
        self.declare_parameter('occupied_min_value', 65)

        self.declare_parameter('min_frontier_cells', 20)
        self.declare_parameter('min_goal_distance_m', 0.60)
        self.declare_parameter('max_goal_distance_m', 7.0)
        self.declare_parameter('goal_clearance_radius_m', 0.35)
        self.declare_parameter('unknown_clearance_radius_m', 0.10)
        self.declare_parameter('path_clearance_radius_m', 0.30)
        self.declare_parameter('path_unknown_clearance_radius_m', 0.0)
        self.declare_parameter('path_check_stride', 3)
        self.declare_parameter('allow_unknown_path_cells', True)
        self.declare_parameter('allow_uncertain_path_cells', True)
        self.declare_parameter('goal_backoff_m', 0.20)
        self.declare_parameter('same_goal_tolerance_m', 0.80)

        self.declare_parameter('lambda_distance', 0.15)
        self.declare_parameter('entropy_weight', 1.0)
        self.declare_parameter('frontier_size_weight', 0.02)
        self.declare_parameter('frontier_size_norm_cells', 100.0)
        self.declare_parameter('entropy_neighborhood_radius_cells', 1)

        self.declare_parameter('publish_markers', True)
        self.declare_parameter('max_marker_frontier_points', 1500)
        self.declare_parameter('marker_topic', '/active_slam/frontiers')
        self.declare_parameter('selected_goal_topic', '/active_slam/selected_goal')
        self.declare_parameter('status_topic', '/active_slam/status')

    def _read_parameters(self):
        self.map_topic = self.get_parameter('map_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.nav2_action_name = self.get_parameter('nav2_action_name').value
        self.compute_path_action_name = self.get_parameter(
            'compute_path_action_name'
        ).value
        self.enable_compute_path_check = bool(
            self.get_parameter('enable_compute_path_check').value
        )
        self.min_path_poses = int(
            self.get_parameter('min_path_poses').value
        )
        self.tf_lookup_timeout_sec = float(
            self.get_parameter('tf_lookup_timeout_sec').value
        )
        self.use_last_pose_on_tf_failure = bool(
            self.get_parameter('use_last_pose_on_tf_failure').value
        )
        self.max_cached_pose_age_sec = float(
            self.get_parameter('max_cached_pose_age_sec').value
        )
        self.enable_recovery_spin = bool(
            self.get_parameter('enable_recovery_spin').value
        )
        self.enable_too_close_recovery_spin = bool(
            self.get_parameter('enable_too_close_recovery_spin').value
        )
        self.too_close_recovery_cooldown_sec = float(
            self.get_parameter('too_close_recovery_cooldown_sec').value
        )
        self.max_consecutive_too_close_spins = int(
            self.get_parameter('max_consecutive_too_close_spins').value
        )
        self.recovery_spin_duration_sec = float(
            self.get_parameter('recovery_spin_duration_sec').value
        )
        self.recovery_spin_angular_vel = float(
            self.get_parameter('recovery_spin_angular_vel').value
        )
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.too_close_recovery_threshold = int(
            self.get_parameter('too_close_recovery_threshold').value
        )
        self.path_failure_threshold = int(
            self.get_parameter('path_failure_threshold').value
        )
        self.path_failure_window_sec = float(
            self.get_parameter('path_failure_window_sec').value
        )
        self.path_failure_pause_sec = float(
            self.get_parameter('path_failure_pause_sec').value
        )
        self.max_path_check_attempts_per_cycle = int(
            self.get_parameter('max_path_check_attempts_per_cycle').value
        )
        self.enable_timing_debug = bool(
            self.get_parameter('enable_timing_debug').value
        )
        self.timing_log_period_sec = float(
            self.get_parameter('timing_log_period_sec').value
        )
        self.max_clusters_to_score = int(
            self.get_parameter('max_clusters_to_score').value
        )
        self.max_seed_cells_per_cluster = int(
            self.get_parameter('max_seed_cells_per_cluster').value
        )
        self.max_total_candidates = int(
            self.get_parameter('max_total_candidates').value
        )
        self.adaptive_goal_distance_enabled = bool(
            self.get_parameter('adaptive_goal_distance_enabled').value
        )
        self.max_goal_distance_hard_m = float(
            self.get_parameter('max_goal_distance_hard_m').value
        )
        self.goal_distance_expand_step_m = float(
            self.get_parameter('goal_distance_expand_step_m').value
        )
        self.cluster_distance_weight = float(
            self.get_parameter('cluster_distance_weight').value
        )
        self.cluster_size_weight = float(
            self.get_parameter('cluster_size_weight').value
        )
        self.cluster_distance_sample_count = int(
            self.get_parameter('cluster_distance_sample_count').value
        )
        self.visited_goal_radius_m = float(
            self.get_parameter('visited_goal_radius_m').value
        )
        self.visited_goal_duration_sec = float(
            self.get_parameter('visited_goal_duration_sec').value
        )
        self.max_visited_goals = int(
            self.get_parameter('max_visited_goals').value
        )
        self.nav_xy_goal_tolerance_m = float(
            self.get_parameter('nav_xy_goal_tolerance_m').value
        )
        self.min_goal_distance_margin_m = float(
            self.get_parameter('min_goal_distance_margin_m').value
        )
        self.enable_stuck_watchdog = bool(
            self.get_parameter('enable_stuck_watchdog').value
        )
        self.stuck_check_radius_m = float(
            self.get_parameter('stuck_check_radius_m').value
        )
        self.stuck_timeout_sec = float(
            self.get_parameter('stuck_timeout_sec').value
        )
        self.enable_costmap_recovery = bool(
            self.get_parameter('enable_costmap_recovery').value
        )
        self.clear_global_costmap_service = self.get_parameter(
            'clear_global_costmap_service'
        ).value
        self.clear_local_costmap_service = self.get_parameter(
            'clear_local_costmap_service'
        ).value
        self.recovery_backup_duration_sec = float(
            self.get_parameter('recovery_backup_duration_sec').value
        )
        self.recovery_backup_linear_vel = float(
            self.get_parameter('recovery_backup_linear_vel').value
        )
        self.recovery_wait_after_clear_sec = float(
            self.get_parameter('recovery_wait_after_clear_sec').value
        )

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
        self.unknown_clearance_radius_m = float(
            self.get_parameter('unknown_clearance_radius_m').value
        )
        self.path_clearance_radius_m = float(
            self.get_parameter('path_clearance_radius_m').value
        )
        self.path_unknown_clearance_radius_m = float(
            self.get_parameter('path_unknown_clearance_radius_m').value
        )
        self.path_check_stride = int(
            self.get_parameter('path_check_stride').value
        )
        self.allow_unknown_path_cells = bool(
            self.get_parameter('allow_unknown_path_cells').value
        )
        self.allow_uncertain_path_cells = bool(
            self.get_parameter('allow_uncertain_path_cells').value
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
        self.max_marker_frontier_points = int(
            self.get_parameter('max_marker_frontier_points').value
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
            self._publish_zero_cmd_vel()
            self._publish_status('WAITING_FOR_TF')
            return

        if self._process_costmap_recovery():
            return

        if (
            self.recovery_spin_active
            and not self.goal_active
            and not self.path_check_active
        ):
            if self._now_seconds() < self.recovery_spin_until_sec:
                self._publish_spin_cmd_vel()
                self._publish_status('RECOVERY_SPIN')
                return

            self._publish_zero_cmd_vel()
            self.recovery_spin_active = False

        if self.goal_active:
            if self._nav_timeout_elapsed():
                if not self.goal_cancel_in_progress:
                    self._publish_status('GOAL_TIMEOUT_CANCELING')
                    self._log_nav_timeout()
                    self._cancel_current_goal()
                else:
                    self._publish_status('GOAL_CANCELING')
            elif self._stuck_watchdog_elapsed(robot_xy):
                if not self.goal_cancel_in_progress:
                    self._publish_status('GOAL_STUCK_CANCELING')
                    self.run_recovery_after_cancel = True
                    self._log_stuck_watchdog(robot_xy)
                    self._cancel_current_goal()
                else:
                    self._publish_status('GOAL_CANCELING')
            else:
                self._publish_status('NAVIGATING')
            return

        if self.path_check_active:
            self._publish_status('CHECKING_PATH')
            return

        if self._path_failure_pause_active():
            self._publish_status('START_IN_LETHAL_OR_COSTMAP_BLOCKED')
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

        timings = {}
        total_start = time.perf_counter()

        step_start = time.perf_counter()
        grid = self._map_to_numpy(self.map_msg)
        timings['map'] = self._elapsed_ms(step_start)

        step_start = time.perf_counter()
        self.map_helpers = self._precompute_map_helpers(grid)
        timings['precompute'] = self._elapsed_ms(step_start)

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

        step_start = time.perf_counter()
        frontier_mask = self._detect_frontier_mask(grid)
        timings['frontier'] = self._elapsed_ms(step_start)

        step_start = time.perf_counter()
        clusters = self._cluster_frontiers(frontier_mask)
        ranked_clusters = self._rank_frontier_clusters(clusters, robot_xy)
        scored_clusters = ranked_clusters[:self.max_clusters_to_score]
        timings['cluster'] = self._elapsed_ms(step_start)

        step_start = time.perf_counter()
        candidates = self._build_candidates(
            grid,
            scored_clusters,
            robot_xy,
            robot_cell,
        )
        if not candidates and self._too_far_rejections_dominate():
            candidates = self._build_candidates_with_expanded_goal_distance(
                grid,
                scored_clusters,
                robot_xy,
                robot_cell,
            )
        timings['candidates'] = self._elapsed_ms(step_start)

        if self.publish_markers:
            step_start = time.perf_counter()
            self._publish_markers(clusters, candidates)
            timings['markers'] = self._elapsed_ms(step_start)
        else:
            timings['markers'] = 0.0

        if not clusters:
            timings['sort'] = 0.0
            timings['start_checks'] = 0.0
            timings['total'] = self._elapsed_ms(total_start)
            self._maybe_log_goal_timing(timings, len(clusters), len(candidates))
            self._publish_status('NO_FRONTIER_FOUND')
            return

        if not candidates:
            timings['sort'] = 0.0
            timings['start_checks'] = 0.0
            timings['total'] = self._elapsed_ms(total_start)
            self._maybe_log_goal_timing(timings, len(clusters), len(candidates))
            rejection_summary = self._format_rejections()
            if self._should_start_recovery_spin('too_close_frontier'):
                self._publish_status(
                    'NO_VALID_FRONTIER clusters=%d %s'
                    % (len(clusters), rejection_summary)
                )
                self._start_recovery_spin('too_close_frontier')
                return

            if self._too_far_rejections_dominate():
                self._warn_throttled(
                    'frontiers_too_far',
                    'All valid frontiers are farther than max_goal_distance_m; '
                    'consider increasing max_goal_distance_m.',
                    period_sec=10.0,
                )

            if self._too_close_rejections_dominate():
                self._warn_throttled(
                    'frontiers_too_close',
                    'No valid frontier candidates because frontiers are too close: %s'
                    % rejection_summary,
                    period_sec=5.0,
                )
                self._publish_status(
                    'NO_VALID_FRONTIER_TOO_CLOSE clusters=%d %s'
                    % (len(clusters), rejection_summary)
                )
                return

            self._publish_status(
                'NO_VALID_FRONTIER clusters=%d %s'
                % (len(clusters), rejection_summary)
            )
            return

        step_start = time.perf_counter()
        candidates.sort(key=lambda candidate: candidate.utility, reverse=True)
        if len(candidates) > self.max_total_candidates:
            candidates = candidates[:self.max_total_candidates]
        self.consecutive_too_close_spins = 0
        timings['sort'] = self._elapsed_ms(step_start)
        self._log_top_candidates(candidates)

        if self.enable_compute_path_check:
            if not self.compute_path_client.server_is_ready():
                self._publish_status('WAITING_FOR_COMPUTE_PATH')
                self._warn_throttled(
                    'compute_path',
                    'Waiting for Nav2 compute path action server %s'
                    % self.compute_path_action_name,
                )
                return
            step_start = time.perf_counter()
            self._start_candidate_path_checks(candidates, robot_xy)
            timings['start_checks'] = self._elapsed_ms(step_start)
            timings['total'] = self._elapsed_ms(total_start)
            self._maybe_log_goal_timing(timings, len(clusters), len(candidates))
            return

        selected = self._select_candidate(candidates)
        if selected is None:
            self._publish_status('NO_NEW_FRONTIER_GOAL')
            return

        goal_pose = self._make_goal_pose(selected.goal_xy, robot_xy)
        self.last_selected_pose = goal_pose
        self.selected_goal_pub.publish(goal_pose)

        self._send_nav_goal(goal_pose, selected)
        timings['start_checks'] = 0.0
        timings['total'] = self._elapsed_ms(total_start)
        self._maybe_log_goal_timing(timings, len(clusters), len(candidates))

    def _lookup_robot_xy(self) -> Optional[WorldPoint]:
        query_time = rclpy.time.Time()
        timeout = Duration(seconds=self.tf_lookup_timeout_sec)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                query_time,
                timeout=timeout,
            )
        except TransformException as exc:
            cached_xy = self._get_cached_robot_xy()
            if cached_xy is not None:
                self._warn_throttled(
                    'tf_cached_pose',
                    'Using cached robot pose because TF lookup failed: %s' % exc,
                    period_sec=2.0,
                )
                return cached_xy

            self._warn_throttled(
                'tf',
                'Waiting for TF %s -> %s failed: %s. '
                'TF tree exists, so this is likely a listener/timing miss.'
                % (self.global_frame, self.robot_base_frame, exc),
            )
            return None

        translation = transform.transform.translation
        robot_xy = (translation.x, translation.y)
        self.last_robot_xy = robot_xy
        self.last_robot_pose_time_sec = self._now_seconds()
        return robot_xy

    def _get_cached_robot_xy(self) -> Optional[WorldPoint]:
        if not self.use_last_pose_on_tf_failure:
            return None

        if self.last_robot_xy is None or self.last_robot_pose_time_sec is None:
            return None

        pose_age_sec = self._now_seconds() - self.last_robot_pose_time_sec
        if pose_age_sec > self.max_cached_pose_age_sec:
            return None

        return self.last_robot_xy

    def _map_to_numpy(self, msg: OccupancyGrid) -> np.ndarray:
        return np.array(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

    def _elapsed_ms(self, start_time: float) -> float:
        return (time.perf_counter() - start_time) * 1000.0

    def _maybe_log_goal_timing(
        self,
        timings: Dict[str, float],
        cluster_count: int,
        candidate_count: int,
    ):
        if not self.enable_timing_debug:
            return

        now_sec = self._now_seconds()
        if now_sec - self.last_timing_log_time_sec < self.timing_log_period_sec:
            return

        self.last_timing_log_time_sec = now_sec
        self.get_logger().info(
            'Goal computation timing: map=%.1fms precompute=%.1fms '
            'frontier=%.1fms cluster=%.1fms candidates=%.1fms sort=%.1fms '
            'markers=%.1fms start_checks=%.1fms total=%.1fms clusters=%d candidates=%d'
            % (
                timings.get('map', 0.0),
                timings.get('precompute', 0.0),
                timings.get('frontier', 0.0),
                timings.get('cluster', 0.0),
                timings.get('candidates', 0.0),
                timings.get('sort', 0.0),
                timings.get('markers', 0.0),
                timings.get('start_checks', 0.0),
                timings.get('total', 0.0),
                cluster_count,
                candidate_count,
            )
        )

    def _precompute_map_helpers(self, grid: np.ndarray) -> Dict[str, np.ndarray]:
        resolution = self.map_msg.info.resolution
        occupied_mask = grid >= self.occupied_min_value
        unknown_mask = grid == self.unknown_value

        helpers = {
            'occupied_mask': occupied_mask,
            'unknown_mask': unknown_mask,
            'entropy_grid': self._build_entropy_grid(grid),
        }

        if distance_transform_edt is not None:
            helpers['occupied_distance_m'] = (
                distance_transform_edt(~occupied_mask) * resolution
            )
            helpers['unknown_distance_m'] = (
                distance_transform_edt(~unknown_mask) * resolution
            )

        entropy_grid = helpers['entropy_grid']
        radius = max(0, self.entropy_neighborhood_radius_cells)
        if uniform_filter is not None and radius > 0:
            helpers['entropy_smooth_grid'] = uniform_filter(
                entropy_grid,
                size=radius * 2 + 1,
                mode='nearest',
            )
        else:
            helpers['entropy_smooth_grid'] = entropy_grid

        return helpers

    def _build_entropy_grid(self, grid: np.ndarray) -> np.ndarray:
        probabilities = np.where(
            grid == self.unknown_value,
            0.5,
            np.clip(grid.astype(np.float32) / 100.0, 1e-3, 1.0 - 1e-3),
        )
        entropy_grid = (
            -probabilities * np.log(probabilities)
            - (1.0 - probabilities) * np.log(1.0 - probabilities)
        ) / math.log(2.0)
        entropy_grid[grid >= self.occupied_min_value] = 0.0
        return entropy_grid.astype(np.float32)

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

    def _rank_frontier_clusters(
        self,
        clusters: Sequence[List[GridCell]],
        robot_xy: WorldPoint,
    ) -> List[List[GridCell]]:
        scored_clusters = []

        for cluster in clusters:
            min_distance_m = self._min_distance_to_cluster(cluster, robot_xy)
            distance_score = math.exp(-self.lambda_distance * min_distance_m)
            size_score = min(len(cluster) / self.frontier_size_norm_cells, 1.0)
            cluster_score = (
                self.cluster_distance_weight * distance_score
                + self.cluster_size_weight * size_score
            )
            scored_clusters.append((cluster_score, min_distance_m, len(cluster), cluster))

        scored_clusters.sort(key=lambda item: item[0], reverse=True)
        self._log_top_clusters(scored_clusters)
        return [cluster for _, _, _, cluster in scored_clusters]

    def _min_distance_to_cluster(
        self,
        cluster: Sequence[GridCell],
        robot_xy: WorldPoint,
    ) -> float:
        if not cluster:
            return float('inf')

        sample_count = max(1, self.cluster_distance_sample_count)
        if len(cluster) <= sample_count:
            sampled_cells = cluster
        else:
            step = max(1, len(cluster) // sample_count)
            sampled_cells = cluster[::step][:sample_count]

        min_distance_m = float('inf')
        for cell in sampled_cells:
            cell_xy = self._map_to_world(cell[0], cell[1], self.map_msg)
            min_distance_m = min(min_distance_m, self._distance(cell_xy, robot_xy))

        return min_distance_m

    def _log_top_clusters(self, scored_clusters, limit: int = 5):
        if not scored_clusters or not self.enable_timing_debug:
            return

        now_sec = self._now_seconds()
        last = self._last_warn_time.get('top_clusters')
        if last is not None and now_sec - last < self.timing_log_period_sec:
            return

        self._last_warn_time['top_clusters'] = now_sec
        parts = []
        for i, (score, distance_m, size, _) in enumerate(scored_clusters[:limit]):
            parts.append(
                '#%d score=%.3f d=%.2f size=%d'
                % (i + 1, score, distance_m, size)
            )

        self.get_logger().info('Top clusters: ' + ' | '.join(parts))

    def _build_candidates(
        self,
        grid: np.ndarray,
        clusters: Sequence[List[GridCell]],
        robot_xy: WorldPoint,
        robot_cell: GridCell,
        max_goal_distance_m: Optional[float] = None,
    ) -> List[FrontierCandidate]:
        search_max_goal_distance_m = (
            self.max_goal_distance_m
            if max_goal_distance_m is None
            else max_goal_distance_m
        )
        candidates: List[FrontierCandidate] = []
        rejections = {
            'out_of_map': 0,
            'no_free_snap': 0,
            'unsafe': 0,
            'too_close': 0,
            'too_far': 0,
            'unsafe_clearance': 0,
            'unsafe_unknown': 0,
            'blacklisted': 0,
            'visited': 0,
            'no_path': 0,
            'no_entropy': 0,
            'duplicate': 0,
        }
        seen_goal_cells = set()
        effective_min_distance = self._effective_min_goal_distance()

        for cluster in clusters:
            for representative_cell in self._cluster_seed_cells(cluster):
                representative_xy = self._map_to_world(
                    representative_cell[0],
                    representative_cell[1],
                    self.map_msg,
                )

                backed_xy = self._backoff_goal(representative_xy, robot_xy)

                preliminary_distance_m = math.hypot(
                    backed_xy[0] - robot_xy[0],
                    backed_xy[1] - robot_xy[1],
                )
                if preliminary_distance_m < effective_min_distance:
                    rejections['too_close'] += 1
                    continue
                if preliminary_distance_m > search_max_goal_distance_m + 0.5:
                    rejections['too_far'] += 1
                    continue
                if self._is_blacklisted(backed_xy):
                    rejections['blacklisted'] += 1
                    continue
                if self._is_visited_goal(backed_xy):
                    rejections['visited'] += 1
                    continue

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
                    search_max_goal_distance_m,
                )
                if rejection_reason is not None:
                    rejections[rejection_reason] += 1
                    continue

                if self._is_blacklisted(goal_xy):
                    rejections['blacklisted'] += 1
                    continue

                if self._is_visited_goal(goal_xy):
                    rejections['visited'] += 1
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

        if len(candidates) > self.max_total_candidates:
            candidates.sort(key=lambda candidate: candidate.utility, reverse=True)
            candidates = candidates[:self.max_total_candidates]

        self.last_candidate_rejections = rejections
        return candidates

    def _build_candidates_with_expanded_goal_distance(
        self,
        grid: np.ndarray,
        clusters: Sequence[List[GridCell]],
        robot_xy: WorldPoint,
        robot_cell: GridCell,
    ) -> List[FrontierCandidate]:
        if not self.adaptive_goal_distance_enabled:
            return []

        candidates: List[FrontierCandidate] = []
        current_max_distance_m = self.max_goal_distance_m
        hard_limit_m = max(self.max_goal_distance_m, self.max_goal_distance_hard_m)

        for _ in range(3):
            if current_max_distance_m >= hard_limit_m:
                break

            expanded_max_distance_m = min(
                current_max_distance_m + self.goal_distance_expand_step_m,
                hard_limit_m,
            )
            self.get_logger().warn(
                'Expanding goal search radius from %.1f to %.1f because all candidates are too far.'
                % (current_max_distance_m, expanded_max_distance_m)
            )

            candidates = self._build_candidates(
                grid,
                clusters,
                robot_xy,
                robot_cell,
                max_goal_distance_m=expanded_max_distance_m,
            )
            current_max_distance_m = expanded_max_distance_m

            if candidates or not self._too_far_rejections_dominate():
                break

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

        max_seeds = max(1, self.max_seed_cells_per_cluster)
        return list(sorted_cells[:max_seeds])

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

        backoff_m = min(
            self.goal_backoff_m,
            max(0.0, distance - self.min_goal_distance_m),
        )
        if backoff_m <= 0.0:
            return frontier_xy

        scale = backoff_m / distance
        return frontier_xy[0] - dx * scale, frontier_xy[1] - dy * scale

    def _snap_to_free_cell(
        self,
        start_cell: GridCell,
        grid: np.ndarray,
    ) -> Optional[GridCell]:
        resolution = self.map_msg.info.resolution
        max_radius = max(2, int(math.ceil(0.8 / resolution)))
        sx, sy = start_cell
        best_with_unknown_clearance = None
        best_with_unknown_score = None
        best_occupied_only = None
        best_occupied_score = None

        def consider(cell: GridCell):
            nonlocal best_with_unknown_clearance
            nonlocal best_with_unknown_score
            nonlocal best_occupied_only
            nonlocal best_occupied_score

            if not self._in_bounds(cell, grid) or not self._is_free_cell(cell, grid):
                return

            if not self._has_occupied_clearance_fast(
                cell,
                grid,
                self.goal_clearance_radius_m,
            ):
                return

            score = math.hypot(cell[0] - sx, cell[1] - sy)

            if best_occupied_score is None or score < best_occupied_score:
                best_occupied_only = cell
                best_occupied_score = score

            if not self._has_unknown_clearance_fast(
                cell,
                grid,
                self.unknown_clearance_radius_m,
            ):
                return

            if best_with_unknown_score is None or score < best_with_unknown_score:
                best_with_unknown_clearance = cell
                best_with_unknown_score = score

        consider(start_cell)

        for radius in range(1, max_radius + 1):
            for y in range(sy - radius, sy + radius + 1):
                for x in range(sx - radius, sx + radius + 1):
                    if max(abs(x - sx), abs(y - sy)) != radius:
                        continue
                    consider((x, y))

        if best_with_unknown_clearance is not None:
            return best_with_unknown_clearance

        return best_occupied_only

    def _has_occupied_clearance_fast(
        self,
        cell: GridCell,
        grid: np.ndarray,
        clearance_m: float,
    ) -> bool:
        distance_grid = self.map_helpers.get('occupied_distance_m')
        if distance_grid is None:
            return self._has_occupied_clearance(cell, grid, clearance_m)

        x, y = cell
        return float(distance_grid[y, x]) >= clearance_m

    def _has_unknown_clearance_fast(
        self,
        cell: GridCell,
        grid: np.ndarray,
        clearance_m: float,
    ) -> bool:
        if clearance_m <= 0.0:
            return True

        distance_grid = self.map_helpers.get('unknown_distance_m')
        if distance_grid is None:
            return self._has_unknown_clearance(cell, grid, clearance_m)

        x, y = cell
        return float(distance_grid[y, x]) >= clearance_m

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

    def _has_unknown_clearance(
        self,
        cell: GridCell,
        grid: np.ndarray,
        clearance_m: float,
    ) -> bool:
        if clearance_m <= 0.0:
            return True

        resolution = self.map_msg.info.resolution
        clearance_cells = int(math.ceil(clearance_m / resolution))
        gx, gy = cell

        for y in range(gy - clearance_cells, gy + clearance_cells + 1):
            for x in range(gx - clearance_cells, gx + clearance_cells + 1):
                if not self._in_bounds((x, y), grid):
                    continue

                if math.hypot(x - gx, y - gy) * resolution > clearance_m:
                    continue

                if grid[y, x] == self.unknown_value:
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
        max_goal_distance_m: Optional[float] = None,
    ) -> Optional[str]:
        effective_min_distance = self._effective_min_goal_distance()
        search_max_goal_distance_m = (
            self.max_goal_distance_m
            if max_goal_distance_m is None
            else max_goal_distance_m
        )

        if distance_m < effective_min_distance:
            return 'too_close'

        if distance_m > search_max_goal_distance_m:
            return 'too_far'

        if not self._in_bounds(goal_cell, grid):
            return 'unsafe'

        if not self._is_free_cell(goal_cell, grid):
            return 'unsafe'

        if not self._has_occupied_clearance_fast(
            goal_cell,
            grid,
            self.goal_clearance_radius_m,
        ):
            return 'unsafe_clearance'

        if not self._has_unknown_clearance_fast(
            goal_cell,
            grid,
            self.unknown_clearance_radius_m,
        ):
            return 'unsafe_unknown'

        return None

    def _effective_min_goal_distance(self) -> float:
        return max(
            self.min_goal_distance_m,
            self.nav_xy_goal_tolerance_m + self.min_goal_distance_margin_m,
        )

    def _path_entropy(
        self,
        path_cells: Sequence[GridCell],
        grid: np.ndarray,
    ) -> Optional[float]:
        entropy_values = []
        entropy_grid = self.map_helpers.get('entropy_smooth_grid')

        for cell in path_cells:
            if not self._in_bounds(cell, grid):
                return None

            x, y = cell
            value = int(grid[y, x])

            if value != self.unknown_value and value >= self.occupied_min_value:
                continue

            if entropy_grid is not None:
                entropy_values.append(float(entropy_grid[y, x]))
            else:
                entropy_values.append(self._cell_entropy(value))

        if not entropy_values:
            return 0.0

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
            if self._candidate_is_new_enough(candidate):
                return candidate

        return None

    def _candidate_is_new_enough(self, candidate: FrontierCandidate) -> bool:
        if self._is_visited_goal(candidate.goal_xy):
            return False

        if self.last_sent_goal_xy is None:
            return True

        return (
            self._distance(candidate.goal_xy, self.last_sent_goal_xy)
            >= self.same_goal_tolerance_m
        )

    def _mark_goal_visited(self, goal_xy: WorldPoint):
        expires_at = self._now_seconds() + self.visited_goal_duration_sec
        self.visited_goals.append((goal_xy, expires_at))

        if len(self.visited_goals) > self.max_visited_goals:
            self.visited_goals = self.visited_goals[-self.max_visited_goals:]

    def _is_visited_goal(self, goal_xy: WorldPoint) -> bool:
        now_sec = self._now_seconds()
        self.visited_goals = [
            (xy, expires_at)
            for xy, expires_at in self.visited_goals
            if expires_at > now_sec
        ]

        return any(
            self._distance(goal_xy, visited_xy) < self.visited_goal_radius_m
            for visited_xy, _ in self.visited_goals
        )

    def _start_candidate_path_checks(
        self,
        candidates: Sequence[FrontierCandidate],
        robot_xy: WorldPoint,
    ):
        self.pending_candidates_queue = [
            candidate
            for candidate in candidates
            if self._candidate_is_new_enough(candidate)
        ]
        self.path_check_attempts_this_cycle = 0
        self.pending_candidate_index = 0

        if not self.pending_candidates_queue:
            self._publish_status('NO_NEW_FRONTIER_GOAL')
            return

        self._try_next_path_candidate(robot_xy)

    def _try_next_path_candidate(self, robot_xy: Optional[WorldPoint] = None):
        if self.goal_active or self.path_check_active:
            return

        if (
            self.path_check_attempts_this_cycle
            >= self.max_path_check_attempts_per_cycle
        ):
            self._publish_status(
                'NO_PATH_CANDIDATES attempts=%d'
                % self.path_check_attempts_this_cycle
            )
            self.last_goal_done_time = self.get_clock().now()
            return

        while self.pending_candidates_queue:
            candidate = self.pending_candidates_queue.pop(0)
            candidate_index = self.pending_candidate_index + 1
            self.pending_candidate_index = candidate_index

            if self._is_blacklisted(candidate.goal_xy):
                self.last_candidate_rejections['blacklisted'] = (
                    self.last_candidate_rejections.get('blacklisted', 0) + 1
                )
                continue

            if robot_xy is None:
                robot_xy = self._lookup_robot_xy()
                if robot_xy is None:
                    self._publish_status('WAITING_FOR_TF')
                    return

            self.path_check_attempts_this_cycle += 1
            goal_pose = self._make_goal_pose(candidate.goal_xy, robot_xy)
            self.last_selected_pose = goal_pose
            self.selected_goal_pub.publish(goal_pose)
            self._start_compute_path_check(goal_pose, candidate, candidate_index)
            return

        self._publish_status('NO_PATH_CANDIDATES queue_empty')
        self.last_goal_done_time = self.get_clock().now()

    def _start_compute_path_check(
        self,
        goal_pose: PoseStamped,
        candidate: FrontierCandidate,
        candidate_index: int = 1,
    ):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose

        if hasattr(goal_msg, 'use_start'):
            goal_msg.use_start = False

        self.path_check_active = True
        self.pending_goal_pose = goal_pose
        self.pending_candidate = candidate
        self.pending_candidate_index = candidate_index

        self._publish_status(
            'CHECKING_PATH candidate=%d goal=(%.2f,%.2f) distance=%.2f'
            % (
                candidate_index,
                candidate.goal_xy[0],
                candidate.goal_xy[1],
                candidate.distance_m,
            )
        )

        self.get_logger().info(
            'Checking Nav2 path candidate=%d x=%.2f y=%.2f utility=%.3f '
            'entropy=%.3f distance=%.2f cluster_size=%d clearance=occupied/unknown pass'
            % (
                candidate_index,
                candidate.goal_xy[0],
                candidate.goal_xy[1],
                candidate.utility,
                candidate.path_entropy,
                candidate.distance_m,
                len(candidate.cluster),
            )
        )

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._compute_path_response_callback)

    def _compute_path_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn('Failed to send ComputePathToPose goal: %s' % exc)
            self._reject_pending_path_candidate('send_failed')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('ComputePathToPose rejected Active SLAM goal')
            self._reject_pending_path_candidate('rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._compute_path_result_callback)

    def _compute_path_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
            path = result.result.path
        except Exception as exc:
            self.get_logger().warn('Failed while waiting for ComputePathToPose: %s' % exc)
            self._reject_pending_path_candidate('result_failed')
            return

        pose_count = len(path.poses)
        if status != GoalStatus.STATUS_SUCCEEDED or pose_count < self.min_path_poses:
            self.get_logger().warn(
                'ComputePathToPose failed for candidate=%d: status=%s poses=%d min_poses=%d'
                % (
                    self.pending_candidate_index,
                    status,
                    pose_count,
                    self.min_path_poses,
                )
            )
            self._reject_pending_path_candidate('no_path')
            return

        path_safe, path_rejection_reason = self._is_path_safe(path)
        if not path_safe:
            self.get_logger().warn(
                'ComputePathToPose path rejected for candidate=%d: reason=%s poses=%d'
                % (
                    self.pending_candidate_index,
                    path_rejection_reason,
                    pose_count,
                )
            )
            self._reject_pending_path_candidate(
                path_rejection_reason,
                count_as_path_failure=False,
            )
            return

        candidate = self.pending_candidate
        goal_pose = self.pending_goal_pose
        self.path_check_active = False
        self.pending_candidate = None
        self.pending_goal_pose = None
        self.path_failure_times = []
        self.path_failure_pause_until_sec = 0.0

        if candidate is None or goal_pose is None:
            self.get_logger().warn('ComputePathToPose passed but pending goal state was empty')
            return

        self.get_logger().info(
            'ComputePathToPose passed for candidate=%d goal x=%.2f y=%.2f poses=%d'
            % (
                self.pending_candidate_index,
                candidate.goal_xy[0],
                candidate.goal_xy[1],
                pose_count,
            )
        )
        self._send_nav_goal(goal_pose, candidate)

    def _is_path_safe(self, path) -> Tuple[bool, str]:
        if self.map_msg is None:
            return False, 'no_map'

        grid = self._map_to_numpy(self.map_msg)
        self.map_helpers = self._precompute_map_helpers(grid)
        stride = max(1, self.path_check_stride)
        pose_count = len(path.poses)

        for i, pose_stamped in enumerate(path.poses):
            if i % stride != 0 and i != pose_count - 1:
                continue

            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            cell = self._world_to_map(x, y, self.map_msg)

            if cell is None:
                return False, 'path_out_of_map'

            traversable, reason = self._is_path_cell_traversable(cell, grid)
            if not traversable:
                return False, reason

            if not self._has_occupied_clearance_fast(
                cell,
                grid,
                self.path_clearance_radius_m,
            ):
                return False, 'path_occupied_clearance'

            if self.path_unknown_clearance_radius_m > 0.0:
                if not self._has_unknown_clearance_fast(
                    cell,
                    grid,
                    self.path_unknown_clearance_radius_m,
                ):
                    return False, 'path_unknown_clearance'

        return True, 'ok'

    def _is_path_cell_traversable(
        self,
        cell: GridCell,
        grid: np.ndarray,
    ) -> Tuple[bool, str]:
        x, y = cell
        value = int(grid[y, x])

        if value == self.unknown_value:
            if self.allow_unknown_path_cells:
                return True, 'unknown_allowed'
            return False, 'path_unknown_cell'

        if value >= self.occupied_min_value:
            return False, 'path_occupied_cell'

        if value > self.free_max_value and not self.allow_uncertain_path_cells:
            return False, 'path_uncertain_cell'

        return True, 'ok'

    def _reject_pending_path_candidate(
        self,
        reason: str,
        count_as_path_failure: bool = True,
    ):
        candidate = self.pending_candidate
        candidate_index = self.pending_candidate_index
        self.path_check_active = False
        self.pending_candidate = None
        self.pending_goal_pose = None
        start_or_costmap_blocked = False
        if count_as_path_failure:
            start_or_costmap_blocked = self._record_path_failure()

        if start_or_costmap_blocked:
            self.get_logger().warn(
                'Repeated path failures, assuming robot start or costmap is blocked. '
                'Check RViz local_costmap around robot, robot_radius/footprint, '
                'laser self-hit, and obstacle_layer footprint clearing.'
            )
            self._publish_status('START_IN_LETHAL_OR_COSTMAP_BLOCKED')
            self.pending_candidates_queue = []
            self._start_costmap_recovery('repeated_path_failures')
        elif candidate is not None:
            rejection_key = 'unsafe_path' if reason.startswith('path_') else 'no_path'
            self.last_candidate_rejections[rejection_key] = (
                self.last_candidate_rejections.get(rejection_key, 0) + 1
            )
            self._blacklist_goal_xy(candidate.goal_xy)
            self.get_logger().warn(
                'Blacklisted candidate=%d goal x=%.2f y=%.2f after path check failure: %s'
                % (
                    candidate_index,
                    candidate.goal_xy[0],
                    candidate.goal_xy[1],
                    reason,
                )
            )

        if not start_or_costmap_blocked:
            self._publish_status('NO_PATH_BLACKLISTED reason=%s' % reason)
            self._try_next_path_candidate()

        self.last_goal_done_time = self.get_clock().now()

    def _record_path_failure(self) -> bool:
        now_sec = self._now_seconds()
        window_start_sec = now_sec - self.path_failure_window_sec
        self.path_failure_times = [
            timestamp
            for timestamp in self.path_failure_times
            if timestamp >= window_start_sec
        ]
        self.path_failure_times.append(now_sec)

        if len(self.path_failure_times) < self.path_failure_threshold:
            return False

        self.path_failure_pause_until_sec = now_sec + self.path_failure_pause_sec
        return True

    def _path_failure_pause_active(self) -> bool:
        return self._now_seconds() < self.path_failure_pause_until_sec

    def _start_costmap_recovery(self, reason: str):
        if not self.enable_costmap_recovery:
            if self.path_failure_pause_sec > 0.0:
                self.path_failure_pause_until_sec = (
                    self._now_seconds() + self.path_failure_pause_sec
                )
            return

        if self.goal_active or self.path_check_active:
            return

        self.costmap_recovery_state = 'CLEAR_COSTMAP'
        self.costmap_recovery_reason = reason
        self.costmap_recovery_until_sec = (
            self._now_seconds() + self.recovery_wait_after_clear_sec
        )
        self._publish_zero_cmd_vel()
        self._clear_costmaps()
        self._publish_status('COSTMAP_RECOVERY_CLEARING')
        self.get_logger().warn(
            'Starting costmap recovery: clear local/global costmaps, light motion only. '
            'reason=%s. Check RViz local_costmap, robot_radius/footprint, laser self-hit, '
            'and obstacle_layer footprint clearing if this repeats.'
            % reason
        )

    def _process_costmap_recovery(self) -> bool:
        if self.costmap_recovery_state == 'IDLE':
            return False

        now_sec = self._now_seconds()

        if self.costmap_recovery_state == 'CLEAR_COSTMAP':
            self._publish_status('COSTMAP_RECOVERY_CLEARING')
            if now_sec < self.costmap_recovery_until_sec:
                return True

            if self.recovery_backup_duration_sec <= 0.0:
                self.costmap_recovery_state = 'SPIN'
                self.costmap_recovery_until_sec = (
                    now_sec + self.recovery_spin_duration_sec
                )
                self.get_logger().warn('Costmap recovery backup skipped')
                return True

            self.costmap_recovery_state = 'BACKUP'
            self.costmap_recovery_until_sec = (
                now_sec + self.recovery_backup_duration_sec
            )
            self.get_logger().warn('Costmap recovery backup started')
            return True

        if self.costmap_recovery_state == 'BACKUP':
            self._publish_backup_cmd_vel()
            self._publish_status('COSTMAP_RECOVERY_BACKUP')
            if now_sec < self.costmap_recovery_until_sec:
                return True

            self.costmap_recovery_state = 'SPIN'
            self.costmap_recovery_until_sec = now_sec + self.recovery_spin_duration_sec
            self.get_logger().warn('Costmap recovery spin started')
            return True

        if self.costmap_recovery_state == 'SPIN':
            self._publish_spin_cmd_vel()
            self._publish_status('COSTMAP_RECOVERY_SPIN')
            if now_sec < self.costmap_recovery_until_sec:
                return True

            self._publish_zero_cmd_vel()
            self.path_failure_times = []
            self.path_failure_pause_until_sec = 0.0
            self.pending_candidates_queue = []
            self.path_check_attempts_this_cycle = 0
            self.costmap_recovery_state = 'IDLE'
            self.costmap_recovery_until_sec = 0.0
            self._publish_status('COSTMAP_RECOVERY_DONE')
            self.get_logger().warn('Costmap recovery done')
            self.last_goal_done_time = self.get_clock().now()
            return True

        self.costmap_recovery_state = 'IDLE'
        self._publish_zero_cmd_vel()
        return False

    def _clear_costmaps(self):
        self._call_clear_costmap(
            self.clear_local_costmap_client,
            self.clear_local_costmap_service,
        )
        self._call_clear_costmap(
            self.clear_global_costmap_client,
            self.clear_global_costmap_service,
        )

    def _call_clear_costmap(self, client, service_name: str):
        if not client.service_is_ready():
            self._warn_throttled(
                service_name,
                'Costmap clear service not ready: %s' % service_name,
            )
            return

        self.get_logger().warn('Clearing costmap via %s' % service_name)
        client.call_async(ClearEntireCostmap.Request())

    def _stuck_watchdog_elapsed(self, robot_xy: WorldPoint) -> bool:
        if not self.enable_stuck_watchdog:
            return False

        if not self.goal_active:
            return False

        now_sec = self._now_seconds()

        if self.last_progress_xy is None:
            self.last_progress_xy = robot_xy
            self.last_progress_time_sec = now_sec
            return False

        moved = self._distance(robot_xy, self.last_progress_xy)

        if moved >= self.stuck_check_radius_m:
            self.last_progress_xy = robot_xy
            self.last_progress_time_sec = now_sec
            return False

        if self.last_progress_time_sec is None:
            self.last_progress_time_sec = now_sec
            return False

        return now_sec - self.last_progress_time_sec >= self.stuck_timeout_sec

    def _log_stuck_watchdog(self, robot_xy: WorldPoint):
        moved = 0.0
        if self.last_progress_xy is not None:
            moved = self._distance(robot_xy, self.last_progress_xy)

        self.get_logger().warn(
            'Stuck watchdog canceling goal: moved=%.3f m below %.3f m for %.1f sec'
            % (moved, self.stuck_check_radius_m, self.stuck_timeout_sec)
        )

    def _should_start_recovery_spin(self, reason: str) -> bool:
        if not self.enable_recovery_spin:
            return False

        if self.goal_active or self.path_check_active:
            return False

        if reason == 'too_close_frontier':
            if not self.enable_too_close_recovery_spin:
                return False

            now_sec = self._now_seconds()
            if (
                now_sec - self.last_too_close_recovery_time_sec
                < self.too_close_recovery_cooldown_sec
            ):
                return False

            if (
                self.consecutive_too_close_spins
                >= self.max_consecutive_too_close_spins
            ):
                return False

        too_close = self.last_candidate_rejections.get('too_close', 0)
        if too_close < self.too_close_recovery_threshold:
            return False

        blocking_rejections = {
            key: value
            for key, value in self.last_candidate_rejections.items()
            if key not in ('too_close', 'duplicate') and value
        }

        return not blocking_rejections

    def _start_recovery_spin(self, reason: str):
        self.recovery_spin_active = True
        self.recovery_spin_until_sec = (
            self._now_seconds() + self.recovery_spin_duration_sec
        )
        if reason == 'too_close_frontier':
            self.last_too_close_recovery_time_sec = self._now_seconds()
            self.consecutive_too_close_spins += 1
        self._publish_status('RECOVERY_SPIN reason=%s' % reason)
        self.get_logger().warn(
            'Starting recovery spin for %.1f sec at %.2f rad/s: %s'
            % (
                self.recovery_spin_duration_sec,
                self.recovery_spin_angular_vel,
                reason,
            )
        )
        self._publish_spin_cmd_vel()

    def _publish_spin_cmd_vel(self):
        twist = Twist()
        twist.angular.z = self.recovery_spin_angular_vel
        self.cmd_vel_pub.publish(twist)

    def _publish_backup_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self.recovery_backup_linear_vel
        self.cmd_vel_pub.publish(twist)

    def _publish_zero_cmd_vel(self):
        self.cmd_vel_pub.publish(Twist())

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
        self.consecutive_too_close_spins = 0
        self.last_progress_xy = None
        self.last_progress_time_sec = self._now_seconds()
        self.pending_candidates_queue = []
        self.path_check_attempts_this_cycle = 0

        self._publish_status(
            'GOAL_SENT utility=%.3f entropy=%.3f distance=%.2f'
            % (
                candidate.utility,
                candidate.path_entropy,
                candidate.distance_m,
            )
        )

        self.get_logger().info(
            'Sending Active SLAM goal x=%.2f y=%.2f utility=%.3f entropy=%.3f distance=%.2f cluster_size=%d'
            % (
                candidate.goal_xy[0],
                candidate.goal_xy[1],
                candidate.utility,
                candidate.path_entropy,
                candidate.distance_m,
                len(candidate.cluster),
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
            if self.current_goal_xy is not None:
                self._mark_goal_visited(self.current_goal_xy)
                self.get_logger().info(
                    'Marked goal as visited x=%.2f y=%.2f radius=%.2f visited_count=%d'
                    % (
                        self.current_goal_xy[0],
                        self.current_goal_xy[1],
                        self.visited_goal_radius_m,
                        len(self.visited_goals),
                    )
                )
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

        if self.nav_timeout_sec <= 0.0:
            return False

        if self.goal_start_time_sec is None:
            return False

        return (
            self._now_seconds() - self.goal_start_time_sec
            >= self.nav_timeout_sec
        )

    def _log_nav_timeout(self):
        elapsed_sec = 0.0
        if self.goal_start_time_sec is not None:
            elapsed_sec = self._now_seconds() - self.goal_start_time_sec

        if self.current_goal_xy is None:
            self.get_logger().warn(
                'Nav2 goal timeout after %.1f sec, timeout=%.1f sec, '
                'goal=(unknown). Canceling and blacklisting.'
                % (elapsed_sec, self.nav_timeout_sec)
            )
            return

        self.get_logger().warn(
            'Nav2 goal timeout after %.1f sec, timeout=%.1f sec, '
            'goal=(%.2f, %.2f). Canceling and blacklisting.'
            % (
                elapsed_sec,
                self.nav_timeout_sec,
                self.current_goal_xy[0],
                self.current_goal_xy[1],
            )
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
            if self.run_recovery_after_cancel:
                self.run_recovery_after_cancel = False
                self._start_costmap_recovery('stuck_watchdog')
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

        if self.run_recovery_after_cancel:
            self.run_recovery_after_cancel = False
            self._start_costmap_recovery('stuck_watchdog')

    def _blacklist_current_goal(self):
        if self.current_goal_xy is None:
            return

        self._blacklist_goal_xy(self.current_goal_xy)

        self.get_logger().warn(
            'Blacklisted goal x=%.2f y=%.2f for %.1f sec'
            % (
                self.current_goal_xy[0],
                self.current_goal_xy[1],
                self.blacklist_duration_sec,
            )
        )

    def _blacklist_goal_xy(self, goal_xy: WorldPoint):
        expires_at = self._now_seconds() + self.blacklist_duration_sec
        self.blacklisted_goals.append((goal_xy, expires_at))

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
        self.last_progress_xy = None
        self.last_progress_time_sec = None

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

        frontier_cells = [cell for cluster in clusters for cell in cluster]
        if (
            self.max_marker_frontier_points > 0
            and len(frontier_cells) > self.max_marker_frontier_points
        ):
            step = int(math.ceil(len(frontier_cells) / self.max_marker_frontier_points))
            frontier_cells = frontier_cells[::step]

        for cell in frontier_cells:
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

    def _too_far_rejections_dominate(self) -> bool:
        if not self.last_candidate_rejections:
            return False

        too_far = self.last_candidate_rejections.get('too_far', 0)
        if too_far <= 0:
            return False

        total_rejections = sum(self.last_candidate_rejections.values())
        other_rejections = total_rejections - too_far
        return too_far >= other_rejections

    def _too_close_rejections_dominate(self) -> bool:
        if not self.last_candidate_rejections:
            return False

        too_close = self.last_candidate_rejections.get('too_close', 0)
        if too_close <= 0:
            return False

        total_rejections = sum(self.last_candidate_rejections.values())
        other_rejections = total_rejections - too_close
        return too_close >= other_rejections

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
