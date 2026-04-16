#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
multi_waypoint_nav.py  —  EE3033 Wheeltec balancing car  (FSM v4)

Autonomous patrol in a KNOWN SLAM map using grid-sampled patrol points
+ move_base (which uses A*/Dijkstra internally for path planning).

Exploration strategy:
  1. Subscribe to /map (OccupancyGrid from map_server/AMCL)
  2. Sample patrol points on a grid across free space, with obstacle clearance
  3. Mark each point as visited when the robot arrives nearby
  4. Always pick the nearest unvisited point as the next goal
  5. When all points visited, reset and start a new patrol round
  6. move_base handles the actual path planning (A*/Dijkstra + DWA)

Target detection:
  - While patrolling, YOLO detections trigger candidate stop
  - Micro-scan confirmation -> pulse-turn alignment -> burst approach
  - Record target position, then go home

Python 2 / Melodic compatible.
"""
from __future__ import print_function

import copy
import math
import traceback
try:
    import actionlib
    import rospy
    from actionlib_msgs.msg import GoalStatus
    from geometry_msgs.msg import (Pose, PoseWithCovarianceStamped,
                                   Point, Quaternion, Twist)
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Empty
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
except Exception as e:
    print("[FATAL] ROS imports failed: %s" % e)
    raise

try:
    from darknet_ros_msgs.msg import BoundingBoxes
    _YOLO_OK = True
except Exception:
    BoundingBoxes = None
    _YOLO_OK = False

# ── goal status ────────────────────────────────────────────
_TERMINAL = frozenset([GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                       GoalStatus.REJECTED, GoalStatus.PREEMPTED,
                       GoalStatus.RECALLED])
_MB = {0:"PENDING",1:"ACTIVE",2:"PREEMPTED",3:"SUCCEEDED",
       4:"ABORTED",5:"REJECTED",6:"PREEMPTING",7:"RECALLING",
       8:"RECALLED",9:"LOST"}

# ── states ─────────────────────────────────────────────────
S_INIT     = "INIT"
S_WAIT     = "WAIT_LOC"
S_EXPLORE  = "EXPLORE"
S_CAND     = "CANDIDATE_STOP"
S_SCAN     = "MICRO_SCAN"
S_ALIGN    = "ALIGN"
S_APPROACH = "APPROACH"
S_RECORD   = "RECORD"
S_HOME     = "GO_HOME"
S_DONE     = "DONE"
S_RECOV    = "RECOVERY"


class MultiWaypointNav(object):

    def __init__(self):
        rospy.init_node("multi_waypoint_nav", anonymous=False)
        rospy.on_shutdown(self._shutdown)
        self._load_params()

        # ── FSM ──
        self.state = S_INIT
        self._state_t = rospy.Time.now()

        # ── pose / sensors ──
        self.pose = None
        self.yaw = 0.0
        self.cov = None
        self.front_range = None

        # ── map + patrol ──
        self._map_data = None
        self._map_info = None
        self._patrol_points = []      # list of (wx, wy) in map frame
        self._patrol_visited = set()  # indices of visited points
        self._patrol_generated = False
        self._patrol_round = 0
        self._patrol_fail_count = 0   # consecutive move_base failures for patrol goals

        # ── home ──
        self.home_pose = None
        self.home_locked = False
        self._loc_anchor = None
        self._loc_anchor_t = None
        self._loc_ready_t = None

        # ── YOLO cache ──
        self.det_time = None
        self.det_ex = 0.0
        self.det_area = 0.0
        self.det_prob = 0.0
        self.det_count = 0
        self.det_total = 0
        self._det_last_counted_t = None

        # ── navigation ──
        self.goal_active = False
        self.goal_sent_t = None
        self.goal_type = ""
        self.goal_pose = None
        self._stuck_pose = None
        self._stuck_t = 0.0
        self._stuck_warns = 0

        # ── micro-scan ──
        self._scan_dir = 1.0
        self._scan_steps = 0
        self._scan_phase = "TURN"
        self._scan_turn_target = 0.0
        self._scan_obs_t = None
        self._scan_obs_hits = 0
        self._scan_last_hit_t = None

        # ── approach ──
        self._app_phase = "FORWARD"
        self._app_phase_t = None
        self._app_total_t = None

        # ── result ──
        self.target_found = False
        self.target_robot_pose = None
        self.target_est_pose = None

        # ── misc ──
        self._start_t = rospy.Time.now()
        self._last_log_t = 0.0
        self._recovery_t = None
        self._cand_hold_hits = 0
        self._cand_last_hit_t = None

        # ── publishers / subscribers ──
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.amcl_topic, PoseWithCovarianceStamped,
                         self._pose_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(self.scan_topic, LaserScan,
                         self._scan_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("/map", OccupancyGrid,
                         self._map_cb, queue_size=1)
        if self.enable_yolo and _YOLO_OK:
            rospy.Subscriber(self.yolo_topic, BoundingBoxes,
                             self._yolo_cb, queue_size=1,
                             buff_size=2**22, tcp_nodelay=True)
        elif self.enable_yolo:
            rospy.logwarn("[INIT] darknet_ros_msgs unavailable — YOLO disabled")

        # ── move_base ──
        self.mb = actionlib.SimpleActionClient(self.mb_action, MoveBaseAction)
        rospy.loginfo("[INIT] Waiting for move_base …")
        if not self.mb.wait_for_server(rospy.Duration(self.mb_wait_timeout)):
            rospy.logfatal("[INIT] move_base not available")
            raise RuntimeError("move_base not available")
        rospy.loginfo("[INIT] move_base connected")

        self._print_config()
        self._set_state(S_WAIT, "startup complete")
        self._main_loop()

    # ================================================================
    #  PARAMETERS
    # ================================================================
    def _load_params(self):
        P = rospy.get_param

        self.target_class   = P("~target_class", "bottle")
        self.enable_yolo    = P("~enable_yolo", True)
        self.return_home    = P("~return_home", True)
        self.report_offset  = float(P("~report_forward_offset", 0.30))

        # topics
        self.cmd_vel_topic  = P("~cmd_vel_topic", "/cmd_vel")
        self.amcl_topic     = P("~amcl_topic", "/amcl_pose")
        self.scan_topic     = P("~scan_topic", "/scan")
        self.yolo_topic     = P("~yolo_boxes_topic", "/darknet_ros/bounding_boxes")
        self.mb_action      = P("~move_base_action", "move_base")
        self.clear_srv      = P("~clear_costmaps_service", "/move_base/clear_costmaps")
        self.mb_wait_timeout = float(P("~move_base_wait_timeout", 20.0))

        # localization
        self.loc_settle     = float(P("~localization_settle_time", 1.0))
        self.loc_cov_xx     = float(P("~home_cov_xx_max", 0.25))
        self.loc_cov_yy     = float(P("~home_cov_yy_max", 0.25))
        self.loc_cov_yaw    = float(P("~home_cov_yawyaw_max", 0.50))
        self.loc_stable_d   = float(P("~home_stable_dist", 0.15))
        self.loc_stable_yaw = float(P("~home_stable_yaw_deg", 20.0))
        self.loc_stable_t   = float(P("~home_stable_time", 1.5))
        self.loc_timeout    = float(P("~localization_wait_timeout", 30.0))

        # YOLO two-level thresholds
        self.cand_min_prob  = float(P("~cand_min_prob", 0.25))
        self.min_prob       = float(P("~min_prob", 0.35))

        # YOLO general
        self.img_w          = int(P("~image_width", 640))
        self.img_h          = int(P("~image_height", 480))
        self.min_det_area   = float(P("~min_detection_area", 0.004))
        self.det_memory     = float(P("~detection_memory", 1.2))
        self.area_thresh    = float(P("~area_threshold", 0.10))
        self.max_img_age    = float(P("~max_image_age", 2.5))
        self._use_age_filter = bool(P("~enable_image_age_filter", False))
        self._age_filter_warned = False

        # candidate stop
        self.cand_hold      = float(P("~candidate_hold_time", 2.5))
        self.cand_confirm   = int(P("~candidate_confirm_hits", 1))    # 1 hit enough — image is slow

        # micro-scan
        self.scan_step_deg  = float(P("~scan_step_deg", 5.0))
        self.scan_turn_spd  = float(P("~scan_turn_speed", 0.12))
        self.scan_obs_time  = float(P("~scan_observe_time", 2.5))    # longer window for slow image
        self.scan_hit_need  = int(P("~scan_hit_threshold", 1))       # 1 hit enough
        self.scan_max_steps = int(P("~scan_max_steps", 6))           # fewer steps, faster

        # align — pulse turn
        self.ang_sign       = float(P("~angular_sign", -1.0))
        self.align_step_deg = float(P("~align_step_deg", 5.0))
        self.align_turn_spd = float(P("~align_turn_speed", 0.25))
        self.dead_zone      = float(P("~dead_zone", 0.45))           # very wide — bbox within 25% of center = aligned
        self.align_confirm_count = int(P("~align_confirm_count", 3))  # must hit dead zone this many times
        self.center_zone    = float(P("~center_forward_zone", 0.20)) # wider — less re-align
        self.align_timeout  = float(P("~align_timeout", 25.0))       # more time
        self.align_lost_t   = float(P("~align_lost_timeout", 8.0))   # wait longer before giving up
        self.align_settle   = float(P("~align_settle_time", 2.5))    # longer wait for slow frame

        # approach
        self.approach_spd   = float(P("~approach_speed", 0.12))
        self.approach_burst = float(P("~approach_burst_time", 1.5))
        self.approach_pause = float(P("~approach_pause_time", 1.5))
        self.approach_timeout = float(P("~approach_timeout", 40.0))

        # laser
        self.front_stop     = float(P("~front_stop_dist", 0.28))
        self.front_slow     = float(P("~front_slow_dist", 0.45))
        self.front_angle    = float(P("~front_angle_deg", 15.0))
        self.front_n        = int(P("~front_sample_count", 5))

        # navigation
        self.nav_timeout    = float(P("~nav_goal_timeout", 60.0))
        self.goal_dist      = float(P("~goal_reached_dist", 0.30))
        self.rate_hz        = float(P("~control_rate_hz", 10.0))
        self.rest_time      = float(P("~rest_time", 0.4))
        self.mission_timeout = float(P("~mission_timeout", 900.0))

        # stuck
        self.stuck_period   = float(P("~stuck_check_period", 8.0))
        self.stuck_dist     = float(P("~stuck_progress_dist", 0.12))
        self.stuck_max_warns = int(P("~stuck_auto_skip_warnings", 2))

        # recovery
        self.recovery_dur   = float(P("~recovery_duration", 2.0))

        # ── PATROL GRID PARAMETERS ──
        # Spacing between patrol points in metres
        self.patrol_spacing    = float(P("~patrol_spacing", 0.6))
        # Minimum distance from obstacles (in grid cells)
        self.patrol_clearance  = int(P("~patrol_clearance_cells", 5))
        # Don't pick goals closer than this to robot
        self.patrol_min_dist   = float(P("~patrol_min_dist", 0.5))
        # Don't pick goals farther than this
        self.patrol_max_dist   = float(P("~patrol_max_dist", 8.0))
        # How close the robot must get to mark a patrol point as visited
        self.patrol_visit_dist = float(P("~patrol_visit_dist", 0.4))

        self._start_t = rospy.Time.now()

    def _print_config(self):
        rospy.loginfo("=" * 58)
        rospy.loginfo("EE3033 multi_waypoint_nav  FSM v4 — grid patrol")
        rospy.loginfo("  target_class     = %s", self.target_class)
        rospy.loginfo("  cand_min_prob    = %.2f  (low gate)", self.cand_min_prob)
        rospy.loginfo("  min_prob         = %.2f  (high gate)", self.min_prob)
        rospy.loginfo("  patrol_spacing   = %.2f m", self.patrol_spacing)
        rospy.loginfo("  patrol_clearance = %d cells", self.patrol_clearance)
        rospy.loginfo("  patrol_min_dist  = %.2f m", self.patrol_min_dist)
        rospy.loginfo("  patrol_max_dist  = %.2f m", self.patrol_max_dist)
        rospy.loginfo("  patrol_visit_dist= %.2f m", self.patrol_visit_dist)
        rospy.loginfo("  scan: step=%.0f° obs=%.1fs hits=%d",
                      self.scan_step_deg, self.scan_obs_time, self.scan_hit_need)
        rospy.loginfo("  approach: burst=%.1fs pause=%.1fs spd=%.2f",
                      self.approach_burst, self.approach_pause, self.approach_spd)
        rospy.loginfo("  area_threshold   = %.2f", self.area_thresh)
        rospy.loginfo("  loc_timeout      = %.0fs", self.loc_timeout)
        rospy.loginfo("  mission_timeout  = %.0fs", self.mission_timeout)
        rospy.loginfo("DO 2D Pose Estimate in RViz NOW")
        rospy.loginfo("=" * 58)

    # ================================================================
    #  CALLBACKS  — only update caches
    # ================================================================
    def _pose_cb(self, msg):
        self.pose = copy.deepcopy(msg.pose.pose)
        q = self.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.cov = list(msg.pose.covariance)

    def _scan_cb(self, msg):
        if not msg.ranges:
            self.front_range = None
            return
        half = math.radians(self.front_angle)
        vs = []
        a = msg.angle_min
        for r in msg.ranges:
            if abs(a) <= half and not (math.isinf(r) or math.isnan(r)) and r > 0.01:
                vs.append(r)
            a += msg.angle_increment
        if not vs:
            self.front_range = None
            return
        vs.sort()
        k = min(max(1, self.front_n), len(vs))
        self.front_range = sum(vs[:k]) / float(k)

    def _map_cb(self, msg):
        self._map_data = list(msg.data)
        self._map_info = msg.info
        # regenerate patrol points when map updates (but not every time)
        if not self._patrol_generated:
            self._generate_patrol_points()

    def _yolo_cb(self, msg):
        self.det_total += 1
        if self._use_age_filter:
            try:
                st = msg.image_header.stamp
                if st.to_sec() > 1.0:
                    age = (rospy.Time.now() - st).to_sec()
                    if age < -2.0:
                        if not self._age_filter_warned:
                            self._age_filter_warned = True
                            rospy.logwarn("[YOLO] Clock skew — disabling age filter")
                        self._use_age_filter = False
                    elif age > self.max_img_age:
                        return
            except Exception:
                pass

        best = None
        for b in msg.bounding_boxes:
            if b.Class == self.target_class and b.probability >= self.cand_min_prob:
                if best is None or b.probability > best.probability:
                    best = b
        if best is None:
            return

        bw = max(0, best.xmax - best.xmin)
        bh = max(0, best.ymax - best.ymin)
        area = (bw * bh) / float(max(1, self.img_w * self.img_h))
        if area < self.min_det_area:
            return

        cx = (best.xmin + best.xmax) / 2.0
        ex = (cx - self.img_w / 2.0) / float(max(1.0, self.img_w / 2.0))

        if self._since_det() > self.det_memory:
            self.det_count = 0

        self.det_time = rospy.Time.now()
        self.det_ex = ex
        self.det_area = area
        self.det_prob = best.probability
        self.det_count += 1

        rospy.loginfo_throttle(0.4,
            "[YOLO] %s p=%.2f ex=%+.3f area=%.4f cnt=%d",
            best.Class, best.probability, ex, area, self.det_count)

    # ================================================================
    #  PATROL POINT GENERATION  (grid sampling on free space)
    # ================================================================
    def _generate_patrol_points(self):
        """
        Sample patrol points on a regular grid across the free space in the map.
        Only cells that are free AND have sufficient clearance from obstacles are used.
        """
        if self._map_data is None or self._map_info is None:
            return

        info = self._map_info
        w = info.width
        h = info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        data = self._map_data

        rospy.loginfo("[PATROL] Generating patrol points from %dx%d map (res=%.3f)", w, h, res)

        # Step 1: build obstacle distance map (simple BFS from all occupied cells)
        # dist_map[i] = minimum distance in cells to nearest obstacle
        INF = w + h
        dist_map = [INF] * (w * h)
        queue = []

        for y in range(h):
            for x in range(w):
                idx = y * w + x
                val = data[idx]
                if val > 50 or val == -1:  # occupied or unknown = obstacle
                    dist_map[idx] = 0
                    queue.append((x, y))

        # BFS to compute distance from obstacles
        head = 0
        while head < len(queue):
            cx, cy = queue[head]
            head += 1
            cd = dist_map[cy * w + cx]
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    ni = ny * w + nx
                    if dist_map[ni] > cd + 1:
                        dist_map[ni] = cd + 1
                        queue.append((nx, ny))

        # Step 2: sample on a grid with patrol_spacing
        step_cells = max(1, int(self.patrol_spacing / res))
        clearance = self.patrol_clearance
        points = []

        for gy in range(clearance, h - clearance, step_cells):
            for gx in range(clearance, w - clearance, step_cells):
                idx = gy * w + gx
                # must be free
                if data[idx] != 0:
                    continue
                # must have sufficient clearance
                if dist_map[idx] < clearance:
                    continue
                # convert to world coordinates
                wx = ox + (gx + 0.5) * res
                wy = oy + (gy + 0.5) * res
                points.append((wx, wy))

        self._patrol_points = points
        self._patrol_visited = set()
        self._patrol_generated = True
        self._patrol_round = 1

        rospy.loginfo("[PATROL] Generated %d patrol points (spacing=%.2fm, clearance=%d cells)",
                      len(points), self.patrol_spacing, clearance)

    def _select_patrol_goal(self):
        """
        Pick the nearest unvisited patrol point to the robot.
        Returns a Pose or None.
        """
        if not self._patrol_points or self.pose is None:
            return None

        rx = self.pose.position.x
        ry = self.pose.position.y

        # first, mark any nearby points as visited
        for i, (px, py) in enumerate(self._patrol_points):
            if i not in self._patrol_visited:
                d = math.hypot(px - rx, py - ry)
                if d < self.patrol_visit_dist:
                    self._patrol_visited.add(i)

        # find nearest unvisited point within distance range
        best_i = -1
        best_d = 1e9
        for i, (px, py) in enumerate(self._patrol_points):
            if i in self._patrol_visited:
                continue
            d = math.hypot(px - rx, py - ry)
            if d < self.patrol_min_dist:
                # too close, skip but don't mark visited
                continue
            if d > self.patrol_max_dist:
                continue
            if d < best_d:
                best_d = d
                best_i = i

        if best_i < 0:
            # no point in range — check if all visited
            unvisited = len(self._patrol_points) - len(self._patrol_visited)
            if unvisited == 0:
                # all visited — start new round
                self._patrol_visited.clear()
                self._patrol_round += 1
                rospy.loginfo("[PATROL] All %d points visited — starting round %d",
                              len(self._patrol_points), self._patrol_round)
                return self._select_patrol_goal()  # recursive retry once
            else:
                # some unvisited but all out of range — pick closest unvisited regardless of max_dist
                for i, (px, py) in enumerate(self._patrol_points):
                    if i in self._patrol_visited:
                        continue
                    d = math.hypot(px - rx, py - ry)
                    if d < best_d:
                        best_d = d
                        best_i = i

        if best_i < 0:
            return None

        px, py = self._patrol_points[best_i]
        angle = math.atan2(py - ry, px - rx)
        q = quaternion_from_euler(0, 0, angle)
        p = Pose()
        p.position = Point(px, py, 0)
        p.orientation = Quaternion(*q)

        unvisited = len(self._patrol_points) - len(self._patrol_visited)
        rospy.loginfo("[PATROL] Goal #%d/%d  x=%.2f y=%.2f  dist=%.2f  unvisited=%d  round=%d",
                      best_i + 1, len(self._patrol_points),
                      px, py, best_d, unvisited, self._patrol_round)
        return p

    # ================================================================
    #  HELPERS
    # ================================================================
    @staticmethod
    def _norm(r):
        while r > math.pi:  r -= 2*math.pi
        while r < -math.pi: r += 2*math.pi
        return r

    @staticmethod
    def _dist(a, b):
        return math.hypot(a.position.x - b.position.x,
                          a.position.y - b.position.y)

    @staticmethod
    def _pose_yaw(p):
        q = p.orientation
        _, _, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return y

    def _since_det(self):
        if self.det_time is None: return 1e9
        return (rospy.Time.now() - self.det_time).to_sec()

    def _det_fresh(self):
        return self._since_det() <= self.det_memory

    def _det_fresh_strong(self):
        return self._det_fresh() and self.det_prob >= self.min_prob

    def _elapsed(self):
        return (rospy.Time.now() - self._start_t).to_sec()

    def _state_age(self):
        return (rospy.Time.now() - self._state_t).to_sec()

    def _pub(self, lx, az, tag=""):
        t = Twist(); t.linear.x = lx; t.angular.z = az
        self.cmd_pub.publish(t)
        now = rospy.get_time()
        if now - self._last_log_t > 0.8:
            self._last_log_t = now
            rospy.loginfo("[CTRL] %-10s vx=%+.3f wz=%+.3f %s", self.state, lx, az, tag)

    def _stop(self, tag=""):
        self._pub(0, 0, tag)

    def _set_state(self, new, reason):
        if new == self.state:
            return
        rospy.loginfo("=" * 48)
        rospy.loginfo("[FSM] %s -> %s  | %s", self.state, new, reason)
        rospy.loginfo("=" * 48)
        self.state = new
        self._state_t = rospy.Time.now()
        if new == S_ALIGN:
            self._align_phase = "DECIDE"
            self._align_settle_t = None
            self._align_centered_count = 0

    # ── move_base ──
    def _send_goal(self, target, gtype):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = "map"
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose = copy.deepcopy(target)
        self.mb.send_goal(g)
        self.goal_active = True
        self.goal_sent_t = rospy.Time.now()
        self.goal_type = gtype
        self.goal_pose = copy.deepcopy(target)
        self._stuck_warns = 0
        rospy.loginfo("[NAV] Goal (%s) x=%.2f y=%.2f", gtype,
                      target.position.x, target.position.y)

    def _cancel_goal(self):
        if self.goal_active:
            try:
                s = self.mb.get_state()
                if s not in _TERMINAL:
                    self.mb.cancel_goal()
                    rospy.loginfo("[NAV] Cancelled (was %s)", _MB.get(s, str(s)))
            except Exception:
                pass
        self.goal_active = False
        self.goal_sent_t = None
        self.goal_type = ""
        self.goal_pose = None

    def _clear_costmaps(self):
        try:
            rospy.wait_for_service(self.clear_srv, timeout=1.5)
            rospy.ServiceProxy(self.clear_srv, Empty)()
            rospy.loginfo("[NAV] Costmaps cleared")
        except Exception:
            pass

    # ================================================================
    #  MAIN LOOP
    # ================================================================
    def _main_loop(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                if self._elapsed() > self.mission_timeout:
                    rospy.logwarn("[FSM] MISSION TIMEOUT")
                    self._set_state(S_DONE, "mission timeout")

                s = self.state
                if   s == S_WAIT:     self._do_wait()
                elif s == S_EXPLORE:  self._do_explore()
                elif s == S_CAND:     self._do_candidate()
                elif s == S_SCAN:     self._do_scan()
                elif s == S_ALIGN:    self._do_align()
                elif s == S_APPROACH: self._do_approach()
                elif s == S_RECORD:   self._do_record()
                elif s == S_HOME:     self._do_home()
                elif s == S_RECOV:    self._do_recovery()
                elif s == S_DONE:     raise StopIteration
                else:                 self._stop("?")

                rate.sleep()
        except StopIteration:
            self._finish()
        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            rospy.logerr("[FATAL] %s\n%s", e, traceback.format_exc())
        finally:
            self._shutdown()

    # ================================================================
    #  WAIT_LOCALIZATION
    # ================================================================
    def _do_wait(self):
        self._stop("WAIT")
        if self.pose is None:
            rospy.loginfo_throttle(2, "[FSM] Waiting for /amcl_pose …")
            return
        if self.home_locked:
            if self._loc_ready_t and rospy.Time.now() < self._loc_ready_t:
                rospy.loginfo_throttle(1, "[FSM] Settling …")
                return
            self._set_state(S_EXPLORE, "localization ready")
            return
        if self.cov is None:
            rospy.loginfo_throttle(2, "[FSM] Waiting for covariance …")
            return

        cxx, cyy, cyaw = self.cov[0], self.cov[7], self.cov[35]
        wt = self._elapsed()
        if wt > self.loc_timeout:
            self._lock_home(self.pose, "TIMEOUT %.0fs" % wt)
            return
        cov_ok = cxx <= self.loc_cov_xx and cyy <= self.loc_cov_yy and cyaw <= self.loc_cov_yaw
        if not cov_ok:
            self._loc_anchor = None
            self._loc_anchor_t = None
            rospy.loginfo_throttle(2,
                "[FSM] WAIT  cov xx=%.3f yy=%.3f yaw=%.3f  timeout_in=%.0fs",
                cxx, cyy, cyaw, self.loc_timeout - wt)
            return
        if self._loc_anchor is None:
            self._loc_anchor = copy.deepcopy(self.pose)
            self._loc_anchor_t = rospy.Time.now()
            rospy.loginfo("[FSM] Localization candidate — hold still")
            return
        dd = self._dist(self.pose, self._loc_anchor)
        dy = abs(self._norm(self.yaw - self._pose_yaw(self._loc_anchor)))
        if dd > self.loc_stable_d or dy > math.radians(self.loc_stable_yaw):
            self._loc_anchor = copy.deepcopy(self.pose)
            self._loc_anchor_t = rospy.Time.now()
            return
        stable = (rospy.Time.now() - self._loc_anchor_t).to_sec()
        if stable >= self.loc_stable_t:
            self._lock_home(self.pose, "stable %.1fs" % stable)
        else:
            rospy.loginfo_throttle(1, "[FSM] WAIT  stable %.1f/%.1fs", stable, self.loc_stable_t)

    def _lock_home(self, p, reason):
        self.home_pose = copy.deepcopy(p)
        self.home_locked = True
        self._loc_ready_t = rospy.Time.now() + rospy.Duration(self.loc_settle)
        rospy.loginfo("=" * 48)
        rospy.loginfo("[NAV] HOME LOCKED x=%.3f y=%.3f yaw=%.1f° (%s)",
                      p.position.x, p.position.y,
                      math.degrees(self._pose_yaw(p)), reason)
        rospy.loginfo("=" * 48)

    # ================================================================
    #  EXPLORE  — grid patrol with move_base
    # ================================================================
    def _do_explore(self):
        # YOLO detection check
        if self._det_fresh() and self.det_count >= 1:
            rospy.loginfo("[EXPLORE] Candidate detection! prob=%.2f cnt=%d — STOPPING",
                          self.det_prob, self.det_count)
            self._cancel_goal()
            self._stop("CANDIDATE_TRIGGER")
            self._cand_hold_hits = 0
            self._cand_last_hit_t = None
            self._set_state(S_CAND, "detection p=%.2f" % self.det_prob)
            return

        # wait for map
        if not self._patrol_generated:
            rospy.loginfo_throttle(2, "[EXPLORE] Waiting for /map to generate patrol points …")
            return

        if not self.goal_active:
            self._send_patrol_goal()
            return

        self._check_nav_goal()
        self._check_stuck()

    def _send_patrol_goal(self):
        goal = self._select_patrol_goal()
        if goal is None:
            rospy.logwarn_throttle(5, "[PATROL] No valid patrol goal available")
            return
        self._patrol_fail_count = 0
        self._send_goal(goal, "PATROL")

    def _check_nav_goal(self):
        if not self.goal_active:
            return
        if self.pose and self.goal_pose:
            d = self._dist(self.pose, self.goal_pose)
            if d < self.goal_dist:
                gt = self.goal_type
                rospy.loginfo("[NAV] Reached (d=%.2f) %s", d, gt)
                self._cancel_goal()
                rospy.sleep(self.rest_time)
                if gt == "HOME":
                    self._set_state(S_DONE, "home reached")
                return

        sn = self.mb.get_state()
        if sn == GoalStatus.SUCCEEDED:
            gt = self.goal_type
            rospy.loginfo("[NAV] SUCCEEDED %s", gt)
            self._cancel_goal()
            rospy.sleep(self.rest_time)
            if gt == "HOME":
                self._set_state(S_DONE, "home reached")
            return
        if sn in (GoalStatus.ABORTED, GoalStatus.REJECTED):
            gt = self.goal_type
            rospy.logwarn("[NAV] %s %s", _MB.get(sn, "?"), gt)
            self._cancel_goal()
            if gt == "PATROL":
                self._patrol_fail_count += 1
                if self._patrol_fail_count >= 3:
                    self._set_state(S_RECOV, "3 patrol goals failed")
                    self._patrol_fail_count = 0
                else:
                    rospy.loginfo("[PATROL] Goal failed — trying another point")
                    # just let next loop iteration pick a new goal
            else:
                self._set_state(S_RECOV, "%s goal failed" % gt)
            return
        if sn == GoalStatus.PREEMPTED:
            self._cancel_goal()
            return
        if self.goal_sent_t:
            dt = (rospy.Time.now() - self.goal_sent_t).to_sec()
            if dt > self.nav_timeout:
                rospy.logwarn("[NAV] Timeout %.0fs %s", dt, self.goal_type)
                self._cancel_goal()

    def _check_stuck(self):
        if not self.pose or not self.goal_active:
            self._stuck_pose = copy.deepcopy(self.pose) if self.pose else None
            self._stuck_t = rospy.get_time()
            self._stuck_warns = 0
            return
        if self._stuck_pose is None:
            self._stuck_pose = copy.deepcopy(self.pose)
            self._stuck_t = rospy.get_time()
            return
        dt = rospy.get_time() - self._stuck_t
        if dt < self.stuck_period:
            return
        dd = self._dist(self.pose, self._stuck_pose)
        if dd < self.stuck_dist:
            self._stuck_warns += 1
            rospy.logwarn("[NAV] Stuck? %.2fm in %.0fs (%d/%d)",
                          dd, dt, self._stuck_warns, self.stuck_max_warns)
            if self._stuck_warns >= self.stuck_max_warns:
                rospy.logwarn("[NAV] Skipping stuck goal")
                self._cancel_goal()
                self._stuck_warns = 0
        else:
            self._stuck_warns = 0
        self._stuck_pose = copy.deepcopy(self.pose)
        self._stuck_t = rospy.get_time()

    # ================================================================
    #  CANDIDATE_STOP
    # ================================================================
    def _do_candidate(self):
        self._stop("CAND_HOLD")
        hold_dt = self._state_age()
        if self._det_fresh():
            if self._cand_last_hit_t is None or self.det_time != self._cand_last_hit_t:
                self._cand_last_hit_t = self.det_time
                self._cand_hold_hits += 1
                rospy.loginfo("[CAND] Hit %d/%d  p=%.2f",
                              self._cand_hold_hits, self.cand_confirm, self.det_prob)
        rospy.loginfo_throttle(0.5, "[CAND] Hold %.1f/%.1fs  hits=%d/%d",
                               hold_dt, self.cand_hold,
                               self._cand_hold_hits, self.cand_confirm)
        if hold_dt < self.cand_hold:
            return
        if self._cand_hold_hits >= self.cand_confirm:
            rospy.loginfo("[CAND] Confirmed — skipping micro-scan, straight to ALIGN")
            self._set_state(S_ALIGN, "candidate confirmed")
        else:
            rospy.loginfo("[CAND] False alarm — resume")
            self.det_count = 0
            self._set_state(S_EXPLORE, "candidate lost")

    # ================================================================
    #  MICRO_SCAN
    # ================================================================
    def _init_scan(self):
        self._scan_steps = 0
        self._scan_dir = -1.0 if self.det_ex > 0.0 else 1.0
        self._scan_phase = "TURN"
        self._scan_obs_t = None
        self._scan_obs_hits = 0
        self._scan_last_hit_t = None
        self._scan_turn_target = self._norm(
            self.yaw + self._scan_dir * math.radians(self.scan_step_deg))
        rospy.loginfo("[SCAN] Init dir=%+.0f step=%.0f°", self._scan_dir, self.scan_step_deg)

    def _do_scan(self):
        if self._scan_steps > self.scan_max_steps:
            rospy.logwarn("[SCAN] Exhausted — back to EXPLORE")
            self.det_count = 0
            self._set_state(S_EXPLORE, "scan exhausted")
            return
        if self._scan_phase == "TURN":
            err = self._norm(self._scan_turn_target - self.yaw)
            if abs(err) < math.radians(2.0):
                self._stop("TURN_DONE")
                self._scan_phase = "OBSERVE"
                self._scan_obs_t = rospy.Time.now()
                self._scan_obs_hits = 0
                self._scan_last_hit_t = None
                rospy.loginfo("[SCAN] Step %d — observe at %.0f°",
                              self._scan_steps, math.degrees(self.yaw))
                return
            spd = self.scan_turn_spd if err > 0 else -self.scan_turn_spd
            self._pub(0, spd, "SCAN_TURN e=%.0f°" % math.degrees(err))
            return
        if self._scan_phase == "OBSERVE":
            self._stop("SCAN_OBS")
            if self._det_fresh():
                if self._scan_last_hit_t is None or self.det_time != self._scan_last_hit_t:
                    self._scan_last_hit_t = self.det_time
                    self._scan_obs_hits += 1
            obs_dt = (rospy.Time.now() - self._scan_obs_t).to_sec()
            rospy.loginfo_throttle(0.5, "[SCAN] Obs %.1f/%.1fs hits=%d/%d",
                                   obs_dt, self.scan_obs_time,
                                   self._scan_obs_hits, self.scan_hit_need)
            if obs_dt < self.scan_obs_time:
                return
            if self._scan_obs_hits >= self.scan_hit_need:
                rospy.loginfo("[SCAN] CONFIRMED at %.0f°", math.degrees(self.yaw))
                self._set_state(S_ALIGN, "scan confirmed")
                return
            self._scan_steps += 1
            if self._scan_obs_hits > 0:
                self._scan_dir *= -1.0
            self._scan_turn_target = self._norm(
                self.yaw + self._scan_dir * math.radians(self.scan_step_deg))
            self._scan_phase = "TURN"
            rospy.loginfo("[SCAN] Next step %d", self._scan_steps)

    # ================================================================
    #  ALIGN  — pulse turn for balancing car
    # ================================================================
    def _do_align(self):
        if self._state_age() > self.align_timeout:
            self.det_count = 0
            self._set_state(S_EXPLORE, "align timeout")
            return
        if self._since_det() > self.align_lost_t:
            self.det_count = 0
            self._set_state(S_EXPLORE, "align lost")
            return

        if not hasattr(self, '_align_phase') or self._align_phase is None:
            self._align_phase = "DECIDE"
            self._align_turn_target = 0.0
            self._align_settle_t = None

        if self._align_phase == "DECIDE":
            if not self._det_fresh():
                self._stop("ALIGN_WAIT")
                return
            ex = self.det_ex
            rospy.loginfo("[ALIGN] DECIDE ex=%+.3f dz=%.2f centered=%d/%d",
                          ex, self.dead_zone, self._align_centered_count, self.align_confirm_count)
            if abs(ex) < self.dead_zone:
                self._align_centered_count += 1
                rospy.loginfo("[ALIGN] In dead zone (%d/%d)",
                              self._align_centered_count, self.align_confirm_count)
                if self._align_centered_count >= self.align_confirm_count:
                    self._stop("ALIGNED")
                    rospy.loginfo("[ALIGN] *** CONFIRMED CENTRED *** — entering APPROACH")
                    self._align_phase = None
                    self._app_phase = "FORWARD"
                    self._app_phase_t = rospy.Time.now()
                    if self._app_total_t is None:
                        self._app_total_t = rospy.Time.now()
                    self._set_state(S_APPROACH, "aligned")
                    return
                # in dead zone but not enough confirmations yet — settle and re-check
                self._align_phase = "SETTLE"
                self._align_settle_t = rospy.Time.now()
                return
            else:
                # outside dead zone — reset counter
                self._align_centered_count = 0
            turn_dir = self.ang_sign if ex > 0 else -self.ang_sign
            step_rad = math.radians(self.align_step_deg)
            self._align_turn_target = self._norm(self.yaw + turn_dir * step_rad)
            self._align_phase = "TURNING"
            rospy.loginfo("[ALIGN] Pulse %.0f°", math.degrees(turn_dir * step_rad))
            return

        if self._align_phase == "TURNING":
            err = self._norm(self._align_turn_target - self.yaw)
            if abs(err) < math.radians(1.5):
                self._stop("PULSE_DONE")
                self._align_phase = "SETTLE"
                self._align_settle_t = rospy.Time.now()
                return
            spd = self.align_turn_spd if err > 0 else -self.align_turn_spd
            self._pub(0, spd, "PULSE e=%.0f°" % math.degrees(err))
            return

        if self._align_phase == "SETTLE":
            self._stop("SETTLE")
            if (rospy.Time.now() - self._align_settle_t).to_sec() >= self.align_settle:
                self._align_phase = "DECIDE"
            return

    # ================================================================
    #  APPROACH  — burst forward + pause
    # ================================================================
    def _do_approach(self):
        if self._app_total_t is None:
            self._app_total_t = rospy.Time.now()
        if self._app_phase_t is None:
            self._app_phase_t = rospy.Time.now()

        total = (rospy.Time.now() - self._app_total_t).to_sec()
        if total > self.approach_timeout:
            self._set_state(S_RECORD, "approach timeout")
            return

        if self.front_range is not None and self.front_range <= self.front_stop:
            rospy.loginfo("[APPROACH] Front %.2fm — stop", self.front_range)
            self._stop("FRONT")
            self._set_state(S_RECORD, "front stop")
            return

        if self._det_fresh() and self.det_area >= self.area_thresh:
            rospy.loginfo("[APPROACH] Area %.3f >= %.3f — close enough",
                          self.det_area, self.area_thresh)
            self._stop("AREA")
            self._set_state(S_RECORD, "bbox area")
            return

        phase_dt = (rospy.Time.now() - self._app_phase_t).to_sec()

        if self._app_phase == "FORWARD":
            if phase_dt >= self.approach_burst:
                self._stop("BURST_END")
                self._app_phase = "PAUSE"
                self._app_phase_t = rospy.Time.now()
                rospy.loginfo("[APPROACH] Burst done — pause %.1fs", self.approach_pause)
                return
            spd = self.approach_spd
            if self.front_range is not None and self.front_range < self.front_slow:
                ratio = (self.front_range - self.front_stop) / max(0.01, self.front_slow - self.front_stop)
                spd *= max(0.0, min(1.0, ratio))
            wz = 0.0
            if self._det_fresh() and abs(self.det_ex) > self.dead_zone:
                wz = self.ang_sign * 0.3 * self.det_ex
                wz = max(-0.15, min(0.15, wz))
            self._pub(max(0.0, spd), wz, "FWD fr=%.2f area=%.3f" % (
                self.front_range if self.front_range else -1, self.det_area))

        elif self._app_phase == "PAUSE":
            self._stop("PAUSE")
            if phase_dt < self.approach_pause:
                return
            if self._det_fresh():
                if abs(self.det_ex) > self.center_zone:
                    self._set_state(S_ALIGN, "re-align")
                else:
                    self._app_phase = "FORWARD"
                    self._app_phase_t = rospy.Time.now()
                    rospy.loginfo("[APPROACH] Next burst")
            else:
                if self._since_det() > self.align_lost_t:
                    self.det_count = 0
                    self._app_total_t = None
                    self._set_state(S_EXPLORE, "lost in approach")
                else:
                    self._set_state(S_ALIGN, "re-acquire")

    # ================================================================
    #  RECORD
    # ================================================================
    def _do_record(self):
        self._stop("RECORD")
        if not self.pose:
            self._set_state(S_DONE, "no pose")
            return
        self.target_found = True
        self.target_robot_pose = copy.deepcopy(self.pose)
        yaw = self._pose_yaw(self.pose)
        est = copy.deepcopy(self.pose)
        # Use laser front_range if available (more accurate than fixed offset)
        offset = self.front_range if self.front_range is not None else self.report_offset
        est.position.x += offset * math.cos(yaw)
        est.position.y += offset * math.sin(yaw)
        self.target_est_pose = est
        rospy.loginfo("[TARGET] Using offset=%.3f m (laser=%s)",
                      offset, "yes" if self.front_range is not None else "no")

        rospy.loginfo("=" * 58)
        rospy.loginfo("[TARGET] ===== TARGET FOUND =====")
        rospy.loginfo("[TARGET] Robot  : x=%.3f y=%.3f yaw=%.1f°",
                      self.target_robot_pose.position.x,
                      self.target_robot_pose.position.y, math.degrees(yaw))
        rospy.loginfo("[TARGET] Object : x=%.3f y=%.3f", est.position.x, est.position.y)
        if self.front_range is not None:
            rospy.loginfo("[TARGET] Front  : %.3f m", self.front_range)
        rospy.loginfo("[TARGET] Det    : p=%.2f area=%.4f", self.det_prob, self.det_area)
        rospy.loginfo("=" * 58)

        self._app_total_t = None
        if self.return_home and self.home_locked:
            self._set_state(S_HOME, "target recorded")
        else:
            self._set_state(S_DONE, "target recorded")

    # ================================================================
    #  GO_HOME
    # ================================================================
    def _do_home(self):
        if not self.goal_active:
            rospy.loginfo("[NAV] HOME x=%.3f y=%.3f",
                          self.home_pose.position.x, self.home_pose.position.y)
            self._send_goal(self.home_pose, "HOME")
            return
        self._check_nav_goal()

    # ================================================================
    #  RECOVERY
    # ================================================================
    def _do_recovery(self):
        if self._recovery_t is None:
            self._recovery_t = rospy.Time.now()
            self._cancel_goal()
            self._clear_costmaps()
            rospy.logwarn("[RECOVERY] Waiting %.1fs", self.recovery_dur)
        self._stop("RECOVERY")
        if (rospy.Time.now() - self._recovery_t).to_sec() >= self.recovery_dur:
            self._recovery_t = None
            if self.target_found:
                self._set_state(S_HOME, "recovery -> home")
            else:
                self._set_state(S_EXPLORE, "recovery -> explore")

    # ================================================================
    #  DONE
    # ================================================================
    def _finish(self):
        self._stop("DONE")
        rospy.loginfo("=" * 58)
        rospy.loginfo("[FSM] ===== MISSION COMPLETE =====")
        if self.target_found:
            rospy.loginfo("[RESULT] Robot  : x=%.3f y=%.3f",
                          self.target_robot_pose.position.x,
                          self.target_robot_pose.position.y)
            rospy.loginfo("[RESULT] Object : x=%.3f y=%.3f",
                          self.target_est_pose.position.x,
                          self.target_est_pose.position.y)
        else:
            rospy.loginfo("[RESULT] Target NOT found")
        rospy.loginfo("=" * 58)

    def _shutdown(self):
        try: self._cancel_goal()
        except Exception: pass
        try: self.cmd_pub.publish(Twist())
        except Exception: pass


def main():
    try:
        MultiWaypointNav()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        try: rospy.logerr("[FATAL] %s", e)
        except Exception: print("[FATAL] %s" % e)

if __name__ == "__main__":
    main()
