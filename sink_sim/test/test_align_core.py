"""Offline check of the alignment logic, no ROS or Gazebo needed.

Simulates a skid-steer base with actuation noise and odometry drift, a front
camera with a D435-like field of view, and noisy tag detections. Asserts the
TRUE final pose is inside the 2 cm / ~1 deg budget for many random starts.

    python3 -m pytest test/test_align_core.py -q
    python3 test/test_align_core.py          # prints a summary table
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sink_sim.align_core import Aligner, Params, wrap  # noqa: E402

N = int(os.environ.get("N", 60))
FACE_X = 3.45
TAG_Y = 0.375
TAG_LEFT = (FACE_X, TAG_Y)
TAG_RIGHT = (FACE_X, -TAG_Y)
CAM_X = 0.445                 # camera offset forward of base_link
HFOV = math.radians(69.0)
MAX_RANGE = 3.0
MIN_RANGE = 0.12


def visible(pose, tag):
    x, y, th = pose
    cx, cy = x + CAM_X * math.cos(th), y + CAM_X * math.sin(th)
    dx, dy = tag[0] - cx, tag[1] - cy
    d = math.hypot(dx, dy)
    if not (MIN_RANGE < d < MAX_RANGE):
        return None
    if abs(wrap(math.atan2(dy, dx) - th)) > HFOV / 2 * 0.95:
        return None
    # tag must face the camera (we're on the -x side of the face)
    if cx >= FACE_X:
        return None
    return d


def run(start, seed, p=None, verbose=False):
    rng = random.Random(seed)
    p = p or Params()
    al = Aligner(p)
    truth = list(start)
    odom = list(start)
    bias_v = 1 + rng.gauss(0, 0.01)
    bias_w = 1 + rng.gauss(0, 0.02)
    dt = 0.05
    t = 0.0
    while t < 120 and al.state not in (al.DONE, al.FAILED):
        # detections at 10 Hz
        if int(round(t / dt)) % 2 == 0:
            dl, dr = visible(truth, TAG_LEFT), visible(truth, TAG_RIGHT)
            if dl and dr:
                meas = []
                for tag, d in ((TAG_LEFT, dl), (TAG_RIGHT, dr)):
                    s = 0.002 + 0.004 * d
                    # tag position in robot frame from truth, then into odom
                    tx, ty, tth = truth
                    rx = math.cos(-tth) * (tag[0] - tx) - math.sin(-tth) * (tag[1] - ty)
                    ry = math.sin(-tth) * (tag[0] - tx) + math.cos(-tth) * (tag[1] - ty)
                    rx += rng.gauss(0, s)
                    ry += rng.gauss(0, s)
                    ox, oy, oth = odom
                    meas.append((ox + math.cos(oth) * rx - math.sin(oth) * ry,
                                 oy + math.sin(oth) * rx + math.cos(oth) * ry))
                al.add_tags(meas[0], meas[1], tuple(odom))
        v, w = al.step(tuple(odom), t)
        va = v * (1 + rng.gauss(0, 0.03))
        wa = w * (1 + rng.gauss(0, 0.03))
        for st, vv, ww in ((truth, va, wa), (odom, va * bias_v, wa * bias_w)):
            st[0] += vv * math.cos(st[2]) * dt
            st[1] += vv * math.sin(st[2]) * dt
            st[2] = wrap(st[2] + ww * dt)
        t += dt
    # true error vs the true goal
    goal_x = FACE_X - p.standoff
    goal_y = p.lateral_offset  # robot faces +x here, so its left is +y
    ex = truth[0] - goal_x
    ey = truth[1] - goal_y
    eyaw = wrap(truth[2] - 0.0)
    if verbose:
        print(f'start=({start[0]:.2f},{start[1]:+.2f},{math.degrees(start[2]):+5.1f}deg) '
              f'state={al.state:7s} t={t:5.1f}s retries={al.retries} '
              f'ex={ex*100:+5.2f}cm ey={ey*100:+5.2f}cm yaw={math.degrees(eyaw):+5.2f}deg')
    return al, ex, ey, eyaw, t


def random_starts(n, seed=0):
    rng = random.Random(seed)
    for i in range(n):
        yield (rng.uniform(1.4, 2.3), rng.uniform(-0.5, 0.5), rng.uniform(-0.45, 0.45)), i


def test_reaches_two_cm_budget():
    fails = []
    for start, i in random_starts(N):
        al, ex, ey, eyaw, _ = run(start, seed=i)
        ok = al.state == al.DONE and math.hypot(ex, ey) < 0.02 and abs(eyaw) < math.radians(1.5)
        if not ok:
            fails.append((start, al.state, ex, ey, math.degrees(eyaw)))
    assert not fails, fails


def test_lateral_offset_respected():
    p = Params(lateral_offset=0.10)
    al, ex, ey, eyaw, _ = run((1.8, -0.2, 0.2), seed=3, p=p)
    assert al.state == al.DONE
    # run() measures ey against y = +0.10 (robot's left when facing +x is +y)
    assert abs(ey) < 0.02, ey


def test_fails_cleanly_without_tags():
    al, *_ = run((1.8, 0.0, math.pi), seed=1, p=Params(search_timeout=3.0))
    assert al.state in (al.FAILED, al.DONE)


if __name__ == '__main__':
    errs = []
    for start, i in random_starts(N):
        al, ex, ey, eyaw, t = run(start, seed=i, verbose=True)
        errs.append((al.state, math.hypot(ex, ey), abs(math.degrees(eyaw)), t))
    done = [e for e in errs if e[0] == 'DONE']
    print(f'\n{len(done)}/{len(errs)} reached DONE')
    if done:
        print(f'worst position error {max(e[1] for e in done)*100:.2f} cm, '
              f'worst yaw {max(e[2] for e in done):.2f} deg, '
              f'mean time {sum(e[3] for e in done)/len(done):.1f} s')
