# sink_sim: Husky A200 + UR5 sink alignment sim

A ROS 2 Jazzy + Gazebo Harmonic package that spawns a Clearpath Husky A200 with a UR5,
a front RealSense (plus an optional rear one) and a 2D lidar in a generic hospital room,
then drives it up to a sink cabinet and lines it up using two AprilTags on the cabinet face.

```
sink_sim/
  clearpath/robot.yaml                    robot config (2 cameras + lidar + UR5)
  clearpath/robot_front_camera_only.yaml  same thing, front camera only (lighter)
  worlds/hospital_sink.sdf                room, sink cabinet, P-trap, two tags
  tools/make_world.py                     regenerates the world from dimensions
  config/apriltag.yaml                    tag detector settings
  config/aligner.yaml                     alignment settings (standoff, tolerances...)
  launch/sim.launch.py                    Gazebo + robot in the sink world
  launch/align.launch.py                  tag detection + aligner
  sink_sim/align_core.py                  the alignment logic (no ROS, testable)
  sink_sim/sink_aligner.py                ROS 2 node around it
  sink_sim/arm_pose.py                    moves the UR5 to a joint pose
  test/test_align_core.py                 offline test of the alignment logic
```

## How the alignment works

1. **Two tags, one frame.** Tag 0 is on the left stile of the cabinet, tag 1 on the right,
   0.75 m apart. The node only uses the tag *positions*. The midpoint is the sink centre and
   the line between them gives the cabinet's yaw. A 0.75 m baseline gives a much better yaw
   estimate than one small tag's orientation, and it doesn't care how the tag axes are defined.
2. **Remember it in odom.** The sink pose is stored in the `odom` frame and smoothed. When the
   robot gets close, the tags fall out of the camera's view, so the last stretch runs on
   odometry. The estimate is refreshed at the pre-dock point (0.5 m out) so that last stretch
   is short.
3. **Pre-dock, then straight in.** A diff-drive base can't slide sideways, so it first drives to
   a point 0.5 m straight out from the goal, stops to refresh the estimate, turns to face the
   cabinet, then drives straight in along the cabinet's normal. If the lateral error is still
   too big near the end, it backs out and retries (up to 3 times).
4. **Frames for the arm.** It broadcasts `sink_face` (x out of the cabinet toward the robot,
   z up, origin at tag height) and `p_trap` (a fixed offset inside the cabinet). Plan the UR5
   relative to those in MoveIt.

Offline test (300 random starts, actuation noise, 1% odom scale drift, noisy detections, D435
field of view): every run finished, median final error 0.55 cm, 95th percentile 1.3 cm,
worst 1.7 cm, worst yaw 1.1 deg. That's the logic only, Gazebo will be a bit different.

## 1. Laptop setup (one time)

You need **Ubuntu 24.04** for Jazzy. A dedicated GPU helps a lot with the cameras.

```bash
# ROS 2 Jazzy desktop: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update
sudo apt install ros-jazzy-desktop \
  ros-jazzy-clearpath-simulator ros-jazzy-clearpath-nav2-demos \
  ros-jazzy-clearpath-desktop ros-jazzy-clearpath-config-live \
  ros-jazzy-apriltag-ros ros-jazzy-ur-description \
  python3-colcon-common-extensions
```

If apt can't find the `clearpath` packages, add Clearpath's package server first by following
their "Offboard Computer" setup guide, then rerun the install.

## 2. Build

```bash
mkdir -p ~/sink_ws/src
cp -r sink_sim ~/sink_ws/src/
cd ~/sink_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash          # do this in every new terminal
```

## 3. Put the robot config where Clearpath looks for it

The file has to be called `robot.yaml`.

```bash
mkdir -p ~/clearpath
cp ~/sink_ws/src/sink_sim/clearpath/robot.yaml ~/clearpath/robot.yaml
# laptop struggling? use the one-camera version instead:
# cp ~/sink_ws/src/sink_sim/clearpath/robot_front_camera_only.yaml ~/clearpath/robot.yaml
```

Want to see the robot before simulating? This opens RViz and live-updates as you edit the yaml:

```bash
ros2 launch clearpath_config_live view_model.launch.py setup_path:=$HOME/clearpath
```

## 4. Run the alignment demo

Terminal 1, the sim (robot spawns about 1.6 m from the cabinet, a bit off to the side):

```bash
ros2 launch sink_sim sim.launch.py
```

Wait until Gazebo is up and the robot has settled. If it's paused, press play (bottom left).

Terminal 2 (optional), put the arm straight up so it clears the counter:

```bash
ros2 run sink_sim arm_pose --ros-args -r __ns:=/a200_0000 -p use_sim_time:=true -p pose:=up
```

Terminal 3, detection + alignment:

```bash
ros2 launch sink_sim align.launch.py
```

Watch it in the log, or:

```bash
ros2 topic echo /a200_0000/sink_aligner/state
ros2 topic echo /a200_0000/sink_aligner/errors     # x = along, y = lateral (m), z = yaw (rad)
```

Run it again without restarting:

```bash
ros2 service call /a200_0000/sink_aligner/start std_srvs/srv/Trigger
ros2 service call /a200_0000/sink_aligner/cancel std_srvs/srv/Trigger
```

To try a different starting spot: `ros2 launch sink_sim sim.launch.py x:=1.0 y:=0.4 yaw:=-0.3`

## 5. Full flow: navigate across the room, then align

Use Nav2 for the coarse part and the aligner for the last metre.

```bash
# T1
ros2 launch sink_sim sim.launch.py x:=-2.5 y:=-1.5 yaw:=0.0
# T2
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=$HOME/clearpath/
# T3
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=$HOME/clearpath/
# T4
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a200_0000 use_sim_time:=true
# T5 (don't drive yet)
ros2 launch sink_sim align.launch.py autostart:=false
```

In RViz, use the Nav2 Goal tool to send the robot to roughly 1 to 1.5 m in front of the
sink, facing it. When it arrives:

```bash
ros2 service call /a200_0000/sink_aligner/start std_srvs/srv/Trigger
```

Save the map once it looks good so you can use localization next time:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/hospital_map --ros-args -p map_subscribe_transient_local:=true -r __ns:=/a200_0000
```

Next step from here is a small script that sends the Nav2 goal and then calls `start`
automatically.

## 6. The arm (MoveIt)

MoveIt is turned on in `robot.yaml`.

```bash
ros2 launch clearpath_manipulators moveit.launch.py setup_path:=$HOME/clearpath use_sim_time:=true
ros2 launch clearpath_viz view_moveit.launch.py namespace:=a200_0000 use_sim_time:=True
```

After alignment, add the TF display in RViz and you'll see `sink_face` and `p_trap`.
Drag the interactive marker toward `p_trap` and plan/execute to check the UR5 can reach.
`p_trap_offset` in `config/aligner.yaml` sets where `p_trap` sits relative to `sink_face`.

## 7. Changing the geometry

All room, cabinet, tag and P-trap dimensions are at the top of `tools/make_world.py`.

```bash
cd ~/sink_ws/src/sink_sim
python3 tools/make_world.py
cd ~/sink_ws && colcon build --symlink-install
```

If you change the tag spacing or size, also update `expected_separation` in
`config/aligner.yaml` and `size` / `sizes` in `config/apriltag.yaml`. Tag `size` is the black
square only (the texture has a white border around it).

The stop distance is `standoff` (base_link to the cabinet face). At 0.62 m the front bumper
ends up about 12 cm from the cabinet. `lateral_offset` shifts the final spot sideways if the
P-trap isn't centred between the tags.

## 8. Offline test (no ROS needed)

```bash
cd ~/sink_ws/src/sink_sim
python3 -m pytest test/test_align_core.py -q
N=300 python3 test/test_align_core.py        # prints every run + a summary
```

Handy when you tune `config/aligner.yaml` defaults in `align_core.Params`.

## Troubleshooting

* **Topic names.** These assume namespace `a200_0000` and the front camera as `camera_0`.
  Check with `ros2 topic list | grep -E "camera_0|cmd_vel|tf"`. The image should be
  `/a200_0000/sensors/camera_0/color/image`.
* **Nothing in TF / aligner stuck at SEARCH.** Clearpath publishes TF on `/a200_0000/tf`,
  not `/tf`. Both nodes are remapped in `align.launch.py`; anything else you write needs the
  same remap. Check detections with `ros2 topic echo /a200_0000/detections`.
* **Tags visible in Gazebo but never detected.** Probably the texture renders mirrored. Set
  `USE_MIRRORED_TEXTURE = True` in `tools/make_world.py`, regenerate and rebuild.
* **Tag poses look rotated or far off.** apriltag_ros expects the image `frame_id` to be the
  camera's *optical* frame. Check with
  `ros2 topic echo /a200_0000/sensors/camera_0/color/camera_info --once | grep frame_id`.
* **Robot won't move.** The node checks whether `cmd_vel` wants `Twist` or `TwistStamped` and
  logs what it picked. If that's wrong, set `cmd_vel_stamped: true` or `false` in
  `config/aligner.yaml`.
* **Sim is slow.** Use `robot_front_camera_only.yaml`, close the Gazebo GUI camera views,
  and keep other apps closed. Real time factor (bottom right in Gazebo) below ~0.5 makes
  control sluggish but it should still converge.
* **Arm pose script can't find the arm.** Make sure the sim is fully up, then
  `ros2 action list | grep follow_joint_trajectory`.

## Moving to the real robot later

The same nodes run on the robot. Things that will change: real camera intrinsics and
calibration, printed tags (measure the black square and their spacing exactly), and odometry
quality on hospital floors. Before trusting the 2 cm budget, measure the real final error
with a tape a few dozen times.

Heads up on the Jetson: the original Jetson Nano tops out at JetPack 4.6 (Ubuntu 18.04), which
doesn't run ROS 2 Jazzy natively, so you'd need Docker there. A Jetson Orin Nano on
JetPack 7.2 runs Ubuntu 24.04, where Jazzy is NVIDIA's recommended ROS version.
