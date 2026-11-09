# sink_sim: Ridgeback + KUKA iiwa 7 sink alignment sim

A ROS 2 Jazzy + Gazebo Harmonic package that spawns a Clearpath Ridgeback with a KUKA LBR
iiwa 7 R800, a front RealSense (plus an optional rear one) and the Ridgeback's front lidar in
a generic hospital room, then drives it up to a sink cabinet and lines it up using two
AprilTags on the cabinet face.

```
sink_sim/
  clearpath/robot.yaml                    robot config (Ridgeback, 2 cameras, lidar, iiwa)
  clearpath/robot_front_camera_only.yaml  same thing, front camera only (lighter)
  urdf/iiwa_arm.urdf.xacro                puts the iiwa on the deck (mount position is here)
  config/arm_controllers.yaml             trajectory controller for the iiwa in sim
  worlds/hospital_sink.sdf                room, sink cabinet, P-trap, two tags
  tools/make_world.py                     regenerates the world from dimensions
  config/apriltag.yaml                    tag detector settings
  config/aligner.yaml                     alignment settings (standoff, tolerances...)
  launch/sim.launch.py                    Gazebo + robot in the sink world + arm controller
  launch/align.launch.py                  tag detection + aligner
  sink_sim/align_core.py                  the alignment logic (no ROS, testable)
  sink_sim/sink_aligner.py                ROS 2 node around it
  sink_sim/arm_pose.py                    moves the iiwa to a joint pose
  test/test_align_core.py                 offline test of the alignment logic
```

## How the robot is put together

* **Ridgeback** comes straight from Clearpath (`serial_number: r100-0000`, namespace
  `r100_0000`). Its mecanum wheels let it drive forward, sideways and turn at the same time.
* **KUKA iiwa 7 R800** isn't in Clearpath's supported arm list, so it's added through
  `platform.extras` in `robot.yaml`. That points at `urdf/iiwa_arm.urdf.xacro`, which uses
  the model from [lbr-stack](https://github.com/lbr-stack/lbr_iiwa7_r800_description)
  (cloned into the workspace by `setup.sh`, it isn't on apt) and bolts it to the deck.
  Joints are `arm_0_A1` to `arm_0_A7`. All zeros is the straight-up "candle" pose.
* **The arm mount** defaults to 0.20 m forward of the deck centre. Change `arm_mount_xyz`
  at the top of `urdf/iiwa_arm.urdf.xacro` to match where it sits on your real robot.
* **Arm control in sim:** `sim.launch.py` starts `arm_controller` (a joint trajectory
  controller) in Clearpath's controller manager. Until it starts, the arm can sag under
  gravity. Send it to a pose with `arm_pose` once it's up.

## How the alignment works

1. **Two tags, one frame.** Tag 0 is on the left stile of the cabinet, tag 1 on the right,
   0.75 m apart. The node only uses the tag *positions*. The midpoint is the sink centre and
   the line between them gives the cabinet's yaw.
2. **Remember it in odom.** The sink pose is stored in the `odom` frame and smoothed. When the
   robot gets close, the tags fall out of the camera's view, so the last stretch runs on
   odometry. The estimate is refreshed at the pre-dock point (0.5 m out) so that stretch is short.
3. **Pre-dock, then in.** The Ridgeback moves to a point 0.5 m straight out from the goal while
   turning to face the cabinet (so the tags stay in view), pauses to re-measure, then moves in,
   fixing forward, sideways and yaw error together. No backing out and retrying like the Husky
   needed. (Set `holonomic: false` in `config/aligner.yaml` to get the old diff-drive behaviour.)
4. **Frames for the arm.** It broadcasts `sink_face` (x out of the cabinet toward the robot,
   z up, origin at tag height) and `p_trap` (a fixed offset inside the cabinet).

Offline test (300 random starts, actuation noise, mecanum-style odometry drift with extra
sideways slip, noisy detections, the sim camera's field of view): every run finished, median
final error 0.64 cm, worst 2.05 cm, worst yaw 1.2 deg. Nearly all of the error is in the
forward direction and comes from wheel odometry over the last 0.5 m, where the camera can't
see the tags. That's the logic only, Gazebo will be a bit different.

**Arm reach:** at the default standoff (0.60 m) the arm base ends up about 0.40 m from the
cabinet face and roughly 0.7 m from the P-trap bend, well inside the iiwa 7's 0.8 m reach.

## 1. Laptop setup (one time)

You need **Ubuntu 24.04** for Jazzy. A dedicated GPU helps a lot with the cameras. From the
repo root (the folder with `.devcontainer`), after installing ROS 2 Jazzy desktop:

```bash
bash .devcontainer/setup.sh
source ~/.bashrc
```

That installs the Clearpath, AprilTag and controller packages, clones the iiwa model, builds
the workspace and copies the front-camera robot config to `~/clearpath/robot.yaml`. In a
GitHub Codespace this all runs by itself when the Codespace is created.

To use both cameras instead (needs a decent GPU):

```bash
cp ~/sink_ws/src/sink_sim/clearpath/robot.yaml ~/clearpath/robot.yaml
```

## 2. Run the alignment demo

Terminal 1, the sim (robot spawns about 1.6 m from the cabinet, a bit off to the side):

```bash
ros2 launch sink_sim sim.launch.py
```

Wait until the robot shows up and the log says `Configured and activated arm_controller`.
If Clearpath's own controllers time out on a slow machine, start them by hand:

```bash
ros2 run controller_manager spawner joint_state_broadcaster platform_velocity_controller --controller-manager-timeout 300 --ros-args -r __ns:=/r100_0000
```

Check all three are **active**:

```bash
ros2 control list_controllers --ros-args -r __ns:=/r100_0000
```

Terminal 2, put the arm straight up so it clears the counter:

```bash
ros2 run sink_sim arm_pose --ros-args -r __ns:=/r100_0000 -p use_sim_time:=true -p pose:=up
```

(`-p pose:=transport` folds it back over the deck instead, handy for driving around.)

Terminal 3, detection + alignment:

```bash
ros2 launch sink_sim align.launch.py
```

Watch it in the log (`SEARCH`, `PREDOCK`, `SETTLE`, `FINAL`, `DONE`), or:

```bash
ros2 topic echo /r100_0000/sink_aligner/state
ros2 topic echo /r100_0000/sink_aligner/errors     # x = along, y = lateral (m), z = yaw (rad)
```

Run it again without restarting:

```bash
ros2 service call /r100_0000/sink_aligner/start std_srvs/srv/Trigger
ros2 service call /r100_0000/sink_aligner/cancel std_srvs/srv/Trigger
```

Move the robot by hand (the Ridgeback can go sideways with `linear.y`):

```bash
ros2 topic pub -r 10 -t 30 /r100_0000/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: -0.3}}}"
ros2 topic pub -r 10 -t 20 /r100_0000/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {y: 0.2}}}"
```

To try a different starting spot: `ros2 launch sink_sim sim.launch.py x:=1.0 y:=0.4 yaw:=-0.3`

## 3. Full flow: navigate across the room, then align

```bash
# T1
ros2 launch sink_sim sim.launch.py x:=-2.5 y:=-1.5 yaw:=0.0
# T2
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=$HOME/clearpath/
# T3
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=$HOME/clearpath/
# T4
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/r100_0000 use_sim_time:=true
# T5 (don't drive yet)
ros2 launch sink_sim align.launch.py autostart:=false
```

In RViz, use the Nav2 Goal tool to send the robot to roughly 1 to 1.5 m in front of the
sink, facing it. When it arrives:

```bash
ros2 service call /r100_0000/sink_aligner/start std_srvs/srv/Trigger
```

## 4. The arm

Clearpath only builds MoveIt configs for the arms it supports, so MoveIt isn't set up for the
iiwa yet. For now, move it with joint poses through `arm_pose` (see the top of
`sink_sim/arm_pose.py`). lbr-stack has an `iiwa7_moveit_config` that can be adapted for the
arm on the Ridgeback as a next step.

After alignment, add the TF display in RViz and you'll see `sink_face` and `p_trap`.
`p_trap_offset` in `config/aligner.yaml` sets where `p_trap` sits relative to `sink_face`.

## 5. Changing the geometry

All room, cabinet, tag and P-trap dimensions are at the top of `tools/make_world.py`.

```bash
cd ~/sink_ws/src/sink_sim
python3 tools/make_world.py
cd ~/sink_ws && colcon build --symlink-install
```

If you change the tag spacing or size, also update `expected_separation` in
`config/aligner.yaml` and `size` / `sizes` in `config/apriltag.yaml`. Tag `size` is the black
square only (the texture has a white border around it).

The stop distance is `standoff` (base_link to the cabinet face). At 0.60 m the front of the
Ridgeback ends up about 12 cm from the cabinet.

## 6. Offline test (no ROS needed)

```bash
cd ~/sink_ws/src/sink_sim
python3 -m pytest test/test_align_core.py -q
N=300 python3 test/test_align_core.py          # prints every run + a summary
DIFF=1 N=300 python3 test/test_align_core.py   # same, diff-drive mode
```

## Troubleshooting

* **Topic names.** These assume namespace `r100_0000` and the front camera as `camera_0`.
  Check with `ros2 topic list | grep -E "camera_0|cmd_vel|tf"`.
* **Robot never appears / "Waiting messages on topic [robot_description]".** The robot model
  failed to build. Look for the first error with
  `grep -n -i -E "error|died" ~/.ros/log/latest/launch.log | head`. A message about
  `lbr_iiwa7_r800_description` means the iiwa model wasn't cloned or built; rerun
  `bash .devcontainer/setup.sh`.
* **Arm is floppy or `arm_pose` can't find the arm.** `arm_controller` isn't active yet. Check
  `ros2 control list_controllers --ros-args -r __ns:=/r100_0000`. To start it by hand:
  `ros2 run controller_manager spawner arm_controller --param-file ~/sink_ws/install/sink_sim/share/sink_sim/config/arm_controllers.yaml --controller-manager-timeout 300 --ros-args -r __ns:=/r100_0000`
* **Aligner stuck at SEARCH.** Check both tags are detected:
  `ros2 topic echo /r100_0000/detections --once | grep "id:"`, and that TF links them to the
  robot: `ros2 run tf2_ros tf2_echo odom sink_tag_left --ros-args -r /tf:=/r100_0000/tf -r /tf_static:=/r100_0000/tf_static -p use_sim_time:=true`
* **Camera frames.** Clearpath 2.9.19 and newer include the RealSense optical frames in sim.
  On older versions `align.launch.py` adds the missing link itself. It picks automatically;
  force it with `sim_optical_frame:=true` or `false` if TF complains.
* **Tags visible in Gazebo but never detected.** Set `USE_MIRRORED_TEXTURE = True` in
  `tools/make_world.py`, regenerate and rebuild.
* **Robot won't move.** The node checks whether `cmd_vel` wants `Twist` or `TwistStamped` and
  logs what it picked. If that's wrong, set `cmd_vel_stamped: true` or `false` in
  `config/aligner.yaml`.
* **Sim is slow.** Use `robot_front_camera_only.yaml` and keep other apps closed.

## Moving to the real robot later

The same nodes run on the robot with `use_sim_time:=false` and `sim_optical_frame:=false`.
Things that will change: real camera calibration, printed tags (measure the black square and
their spacing exactly), mecanum odometry on hospital floors (it slips more than wheels on a
diff drive, so measure the final error with a tape a few dozen times), and the arm, which on
the real robot is driven through KUKA's FRI by the lbr-stack driver rather than gz_ros2_control.
