"""Pure-Python alignment logic (no ROS imports) so it can be unit tested.

Idea
----
The two AprilTags give us a "sink face" frame F in the odom frame:
  * origin  = midpoint of the two tags (projected to the floor plane)
  * x axis  = horizontal normal of the cabinet face, pointing OUT toward the robot
  * y axis  = z cross x
Only tag POSITIONS are used (not tag orientations), so it does not matter
which way the tag frames' axes point and the yaw estimate comes from a 0.75 m
baseline instead of one small tag.

The face estimate lives in odom, so when the robot is close and the tags drop
out of the camera's view, it keeps steering on wheel/IMU odometry for the last
stretch.

Goal (in F): robot base_link at x = standoff, y = -lateral_offset, heading
facing the cabinet (pi in F).

Two drive modes:
  * holonomic=True  (Ridgeback, mecanum wheels): moves forward, sideways and
    turns at the same time, so lateral error is simply driven out.
  * holonomic=False (diff drive like a Husky): turn-then-drive with a pre-dock
    point and back-off retries, since it can't move sideways.
step() always returns (vx, vy, w); vy is 0 in diff mode.
"""

import math
from dataclasses import dataclass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class Face:
    x: float
    y: float
    yaw: float  # direction of outward normal in odom


@dataclass
class Params:
    holonomic: bool = True          # True for mecanum / omni bases (Ridgeback)
    standoff: float = 0.60          # base_link origin to cabinet face [m]
    lateral_offset: float = 0.0     # + shifts robot to ITS left when facing sink
    predock_distance: float = 0.50  # pre-dock point this far behind the goal [m]
    predock_tol: float = 0.008      # how close to the pre-dock point counts [m]
    tol_x: float = 0.004            # final along-normal tolerance [m]
    tol_y: float = 0.005            # final lateral tolerance [m]
    tol_yaw: float = 0.015          # final yaw tolerance [rad] (~0.9 deg)
    max_lin: float = 0.25
    max_ang: float = 0.6
    min_lin: float = 0.02           # creep speed so we don't stall on friction
    min_ang: float = 0.06
    k_lin: float = 0.8
    k_ang: float = 1.8
    max_heading_corr: float = 0.5   # cap on steering angle toward the line [rad]
    lookahead_min: float = 0.12
    lookahead_max: float = 0.40
    commit_distance: float = 0.12   # inside this, lateral error can't be fixed going forward
    commit_tol_y: float = 0.012     # lateral error allowed when we commit
    max_retries: int = 3
    expected_separation: float = 0.75
    separation_tol: float = 0.05
    smoothing: float = 0.3          # 0..1, weight of each new face measurement
    freeze_distance: float = 0.35   # stop updating the face estimate when this close
    min_measurements: int = 5       # face measurements before we start driving
    settle_time: float = 1.0        # wait at the pre-dock point to refresh the estimate [s]
    search_ang: float = 0.25
    search_timeout: float = 180.0


def face_from_tags(p_left, p_right, robot_xy):
    """Return a Face from two tag positions (x, y) in odom, or None."""
    dx = p_left[0] - p_right[0]
    dy = p_left[1] - p_right[1]
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return None
    mx = 0.5 * (p_left[0] + p_right[0])
    my = 0.5 * (p_left[1] + p_right[1])
    nx, ny = dy / n, -dx / n
    if nx * (robot_xy[0] - mx) + ny * (robot_xy[1] - my) < 0:
        nx, ny = -nx, -ny
    return Face(mx, my, math.atan2(ny, nx))


def robot_in_face(face, rx, ry, ryaw):
    c, s = math.cos(face.yaw), math.sin(face.yaw)
    dx, dy = rx - face.x, ry - face.y
    return (c * dx + s * dy, -s * dx + c * dy, wrap(ryaw - face.yaw))


def face_to_odom(face, fx, fy):
    c, s = math.cos(face.yaw), math.sin(face.yaw)
    return (face.x + c * fx - s * fy, face.y + s * fx + c * fy)


class Aligner:
    """State machine. Call add_tags() when both tags are seen, step() at a fixed rate.

    Holonomic mode:
    SEARCH   spin in place until both tags give a valid face estimate
    PREDOCK  move to a point predock_distance straight out from the goal while
             turning to face the cabinet (keeps the tags in view)
    SETTLE   stop there briefly so the face estimate refreshes from close range
    FINAL    move in to the goal, correcting along, sideways and yaw together

    Diff-drive mode:
    SEARCH   spin in place until both tags give a valid face estimate
    PREDOCK  drive (turn, go) to a point predock_distance straight out from the goal
    SETTLE   stop there briefly so the face estimate refreshes from close range
    FACE     turn in place to face the cabinet
    FINAL    drive straight in, following the normal line to the goal
    ROTATE   final turn in place to trim yaw
    BACKOFF  reverse out and retry from PREDOCK if lateral error is too big
    """

    SEARCH, PREDOCK, SETTLE, FACE, FINAL, ROTATE, BACKOFF, DONE, FAILED = (
        'SEARCH', 'PREDOCK', 'SETTLE', 'FACE', 'FINAL', 'ROTATE', 'BACKOFF', 'DONE', 'FAILED')

    def __init__(self, params=None):
        self.p = params or Params()
        self.reset()

    def reset(self):
        self.state = self.SEARCH
        self.face = None
        self.n_meas = 0
        self.retries = 0
        self.t_search = None
        self.t_settle = None
        self.last_error = None
        self.reason = ''

    # -- measurements --------------------------------------------------------
    def add_tags(self, p_left, p_right, robot_pose):
        """Feed both tag positions (x, y in odom). Returns True if used."""
        rx, ry, _ = robot_pose
        sep = math.hypot(p_left[0] - p_right[0], p_left[1] - p_right[1])
        if abs(sep - self.p.expected_separation) > self.p.separation_tol:
            return False
        if self.face is not None and self.n_meas >= self.p.min_measurements:
            fx, _, _ = robot_in_face(self.face, *robot_pose)
            if fx < self.p.standoff + self.p.freeze_distance:
                return False  # too close: keep the estimate we have
        m = face_from_tags(p_left, p_right, (rx, ry))
        if m is None:
            return False
        if self.face is None:
            self.face = m
        else:
            a = self.p.smoothing
            self.face = Face(
                (1 - a) * self.face.x + a * m.x,
                (1 - a) * self.face.y + a * m.y,
                wrap(self.face.yaw + a * wrap(m.yaw - self.face.yaw)))
        self.n_meas += 1
        return True

    # -- helpers ---------------------------------------------------------------
    def goal_in_face(self):
        return self.p.standoff, -self.p.lateral_offset

    def errors(self, robot_pose):
        """(ex, ey, eyaw): along-normal distance left, lateral, heading."""
        fx, fy, fth = robot_in_face(self.face, *robot_pose)
        gx, gy = self.goal_in_face()
        return fx - gx, fy - gy, wrap(fth - math.pi)

    def _turn(self, err):
        w = self.p.k_ang * err
        if abs(w) < self.p.min_ang:
            w = math.copysign(self.p.min_ang, w)
        return clip(w, -self.p.max_ang, self.p.max_ang)

    def _fail(self, why):
        self.state = self.FAILED
        self.reason = why
        return 0.0, 0.0, 0.0

    def _axis(self, err, tol):
        """P control on one axis with a creep speed, 0 inside the tolerance."""
        if abs(err) < tol:
            return 0.0
        return math.copysign(clip(self.p.k_lin * abs(err), self.p.min_lin, self.p.max_lin), err)

    def _goto_holonomic(self, robot_pose, fx_t, fy_t, tol_x, tol_y, tol_yaw):
        """Command toward (fx_t, fy_t) in F, facing the cabinet. Returns (vx, vy, w, arrived)."""
        p = self.p
        fx, fy, fth = robot_in_face(self.face, *robot_pose)
        dth = wrap(math.pi - fth)
        vfx = self._axis(fx_t - fx, tol_x)
        vfy = self._axis(fy_t - fy, tol_y)
        n = math.hypot(vfx, vfy)
        if n > p.max_lin:
            vfx, vfy = vfx * p.max_lin / n, vfy * p.max_lin / n
        # big heading error: turn first, so the camera keeps the tags in view
        scale = max(0.0, math.cos(dth)) ** 2
        vfx, vfy = vfx * scale, vfy * scale
        # face frame velocity -> robot frame
        c, s_ = math.cos(fth), math.sin(fth)
        vx = c * vfx + s_ * vfy
        vy = -s_ * vfx + c * vfy
        w = 0.0 if abs(dth) < tol_yaw else self._turn(dth)
        arrived = vfx == 0.0 and vfy == 0.0 and w == 0.0
        return vx, vy, w, arrived

    # -- control ---------------------------------------------------------------
    def step(self, robot_pose, t):
        """Return (vx, vy, w). robot_pose = (x, y, yaw) in odom, t in seconds."""
        p = self.p
        S = self
        if self.state in (S.DONE, S.FAILED):
            return 0.0, 0.0, 0.0

        if self.state == S.SEARCH:
            if self.face is not None and self.n_meas >= p.min_measurements:
                self.state = S.PREDOCK
            else:
                if self.t_search is None:
                    self.t_search = t
                if t - self.t_search > p.search_timeout:
                    return self._fail('tags not found')
                return 0.0, 0.0, (p.search_ang if self.face is None else 0.0)

        self.last_error = self.errors(robot_pose)
        if p.holonomic:
            return self._step_holonomic(robot_pose, t)
        v, w = self._step_diff(robot_pose, t)
        return v, 0.0, w

    def _step_holonomic(self, robot_pose, t):
        p = self.p
        S = self
        gx, gy = self.goal_in_face()

        if self.state == S.PREDOCK:
            vx, vy, w, arrived = self._goto_holonomic(
                robot_pose, gx + p.predock_distance, gy,
                p.predock_tol, p.predock_tol, 2 * p.tol_yaw)
            if arrived:
                self.state = S.SETTLE
                self.t_settle = t
                return 0.0, 0.0, 0.0
            return vx, vy, w

        if self.state == S.SETTLE:
            if t - self.t_settle >= p.settle_time:
                self.state = S.FINAL
            return 0.0, 0.0, 0.0

        if self.state == S.FINAL:
            vx, vy, w, arrived = self._goto_holonomic(
                robot_pose, gx, gy, p.tol_x, p.tol_y, p.tol_yaw)
            if arrived:
                self.state = S.DONE
                return 0.0, 0.0, 0.0
            return vx, vy, w

        return 0.0, 0.0, 0.0

    def _step_diff(self, robot_pose, t):
        """Turn-then-drive for bases that can't move sideways. Returns (v, w)."""
        p = self.p
        S = self
        ex, ey, eyaw = self.last_error
        rx, ry, ryaw = robot_pose

        if self.state == S.PREDOCK:
            gx, gy = self.goal_in_face()
            px, py = face_to_odom(self.face, gx + p.predock_distance, gy)
            dx, dy = px - rx, py - ry
            dist = math.hypot(dx, dy)
            herr = wrap(math.atan2(dy, dx) - ryaw)
            if dist < p.predock_tol or (dist < 3 * p.predock_tol and abs(herr) > math.pi / 2):
                self.state = S.SETTLE
                self.t_settle = t
                return 0.0, 0.0
            # far behind us? reverse rather than turning all the way round
            if abs(herr) > math.pi / 2 and dist < 0.4:
                herr = wrap(herr + math.pi)
                v = -clip(p.k_lin * dist, p.min_lin, p.max_lin)
            else:
                v = clip(p.k_lin * dist, p.min_lin, p.max_lin)
            v *= max(0.0, math.cos(herr)) ** 4
            w = clip(p.k_ang * herr, -p.max_ang, p.max_ang) if dist > 0.05 else 0.0
            return v, w

        if self.state == S.SETTLE:
            if t - self.t_settle >= p.settle_time:
                self.state = S.FACE
            return 0.0, 0.0

        if self.state == S.FACE:
            if abs(eyaw) < 2 * p.tol_yaw:
                self.state = S.FINAL
                return 0.0, 0.0
            return 0.0, self._turn(-eyaw)

        if self.state == S.FINAL:
            if abs(ex) < p.tol_x:
                self.state = S.ROTATE
                return 0.0, 0.0
            if 0 < ex < p.commit_distance and abs(ey) > p.commit_tol_y:
                return self._start_backoff()
            if ex > 0:
                look = clip(ex, p.lookahead_min, p.lookahead_max)
                h_des = clip(math.atan2(ey, look), -p.max_heading_corr, p.max_heading_corr)
            else:
                h_des = 0.0  # overshot: reverse straight
            herr = wrap(h_des - eyaw)
            w = clip(p.k_ang * herr, -p.max_ang, p.max_ang)
            v = clip(p.k_lin * ex, -p.max_lin, p.max_lin)
            if abs(v) < p.min_lin:
                v = math.copysign(p.min_lin, ex)
            v *= max(0.0, math.cos(herr)) ** 2
            return v, w

        if self.state == S.ROTATE:
            if abs(eyaw) < p.tol_yaw:
                if abs(ey) > p.tol_y:
                    return self._start_backoff()
                if abs(ex) > 2.0 * p.tol_x:
                    self.state = S.FINAL
                    return 0.0, 0.0
                self.state = S.DONE
                return 0.0, 0.0
            return 0.0, self._turn(-eyaw)

        if self.state == S.BACKOFF:
            if ex >= p.predock_distance - 0.05:
                self.state = S.PREDOCK
                return 0.0, 0.0
            return -0.15, clip(-p.k_ang * eyaw, -p.max_ang, p.max_ang)

        return 0.0, 0.0

    def _start_backoff(self):
        self.retries += 1
        if self.retries > self.p.max_retries:
            self._fail('lateral error after max retries')
            return 0.0, 0.0
        self.state = self.BACKOFF
        return 0.0, 0.0
