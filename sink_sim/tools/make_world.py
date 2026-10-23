#!/usr/bin/env python3
"""Generate worlds/hospital_sink.sdf.

Everything about the room, the sink cabinet and the two AprilTags is set in
the constants below, so when you get real measurements you change them here
and rerun:

    python3 tools/make_world.py

World layout (metres, Gazebo world frame, z up):
  * Room is ROOM_X by ROOM_Y, centred on the origin.
  * Sink cabinet sits against the +x wall. Its open front face is at x = FACE_X
    and faces -x (toward where the robot spawns).
  * Two tags are on the cabinet face frame (the stiles either side of the
    opening). Tag 0 is on the robot's LEFT when it faces the sink (+y side),
    tag 1 is on the RIGHT (-y side).
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'worlds', 'hospital_sink.sdf')

# ---- room -----------------------------------------------------------------
ROOM_X = 8.0
ROOM_Y = 6.0
WALL_T = 0.10
WALL_H = 2.4

# ---- sink cabinet -----------------------------------------------------------
CAB_W = 0.90        # width along y
CAB_D = 0.55        # depth along x
CAB_H = 0.85        # height of cabinet box (counter sits on top)
PANEL_T = 0.02      # side/back/bottom panel thickness
STILE_W = 0.15      # face frame stile width (tags go here)
TOP_RAIL_H = 0.10
BOTTOM_RAIL_H = 0.12
COUNTER_T = 0.04
COUNTER_OVERHANG = 0.04
WALL_INNER_X = ROOM_X / 2.0           # inner surface of the +x wall
CAB_BACK_X = WALL_INNER_X
FACE_X = CAB_BACK_X - CAB_D           # cabinet front face plane

# ---- AprilTags (tag36h11) ---------------------------------------------------
TAG_SIZE = 0.10                       # black square edge = apriltag 'size'
TAG_PLANE = TAG_SIZE * 10.0 / 8.0     # texture includes the 1-cell white ring
TAG_Z = 0.45                          # tag centre height above the floor
TAG_Y = CAB_W / 2.0 - STILE_W / 2.0   # centred on each stile
TAG_GAP = 0.001                       # sits just proud of the face
USE_MIRRORED_TEXTURE = False          # flip if tags render but never detect

# ---- P-trap (inside the cabinet) -------------------------------------------
PIPE_R = 0.02
DRAIN_X = CAB_BACK_X - 0.28           # tailpiece x
TRAP_BOTTOM_Z = 0.42                  # bottom of the U bend
TRAP_ARM_Z = 0.55                     # trap arm going into the wall


def box(name, pose, size, color, collide=True):
    x, y, z = pose
    sx, sy, sz = size
    r, g, b = color
    col = f"""
        <collision name="{name}_c"><pose>{x} {y} {z} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>""" if collide else ''
    return f"""
        <visual name="{name}_v"><pose>{x} {y} {z} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>{col}"""


def cyl(name, pose, rpy, radius, length, color):
    x, y, z = pose
    R, P, Y = rpy
    r, g, b = color
    return f"""
        <visual name="{name}_v"><pose>{x} {y} {z} {R} {P} {Y}</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
        <collision name="{name}_c"><pose>{x} {y} {z} {R} {P} {Y}</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>"""


def static_model(name, body):
    return f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">{body}
      </link>
    </model>"""


def tag_model(name, tag_id, y):
    suffix = '_mirrored' if USE_MIRRORED_TEXTURE else ''
    tex = f'textures/tag36_11_{tag_id:05d}{suffix}.png'
    # A plane with normal +z, rotated so the normal points -x (out of the
    # cabinet toward the robot).
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{FACE_X - TAG_GAP} {y} {TAG_Z} 0 -1.5708 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>{TAG_PLANE} {TAG_PLANE}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
            <pbr><metal><albedo_map>{tex}</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def build():
    wood = (0.82, 0.78, 0.70)
    white = (0.95, 0.95, 0.95)
    steel = (0.6, 0.62, 0.65)
    wall_c = (0.88, 0.90, 0.92)
    pipe_c = (0.85, 0.85, 0.80)

    hx, hy = ROOM_X / 2.0, ROOM_Y / 2.0
    walls = ''.join([
        box('wall_px', (hx + WALL_T / 2, 0, WALL_H / 2), (WALL_T, ROOM_Y + 2 * WALL_T, WALL_H), wall_c),
        box('wall_nx', (-hx - WALL_T / 2, 0, WALL_H / 2), (WALL_T, ROOM_Y + 2 * WALL_T, WALL_H), wall_c),
        box('wall_py', (0, hy + WALL_T / 2, WALL_H / 2), (ROOM_X, WALL_T, WALL_H), wall_c),
        box('wall_ny', (0, -hy - WALL_T / 2, WALL_H / 2), (ROOM_X, WALL_T, WALL_H), wall_c),
    ])

    cx = CAB_BACK_X - CAB_D / 2.0           # cabinet centre x
    hw = CAB_W / 2.0
    cab = ''.join([
        # side panels
        box('side_l', (cx, hw - PANEL_T / 2, CAB_H / 2), (CAB_D, PANEL_T, CAB_H), wood),
        box('side_r', (cx, -hw + PANEL_T / 2, CAB_H / 2), (CAB_D, PANEL_T, CAB_H), wood),
        # back and bottom
        box('back', (CAB_BACK_X - PANEL_T / 2, 0, CAB_H / 2), (PANEL_T, CAB_W, CAB_H), wood),
        box('bottom', (cx, 0, BOTTOM_RAIL_H - PANEL_T / 2), (CAB_D, CAB_W, PANEL_T), wood),
        # face frame: two stiles, top rail, bottom rail
        box('stile_l', (FACE_X + PANEL_T / 2, hw - STILE_W / 2, CAB_H / 2), (PANEL_T, STILE_W, CAB_H), wood),
        box('stile_r', (FACE_X + PANEL_T / 2, -hw + STILE_W / 2, CAB_H / 2), (PANEL_T, STILE_W, CAB_H), wood),
        box('top_rail', (FACE_X + PANEL_T / 2, 0, CAB_H - TOP_RAIL_H / 2), (PANEL_T, CAB_W, TOP_RAIL_H), wood),
        box('bottom_rail', (FACE_X + PANEL_T / 2, 0, BOTTOM_RAIL_H / 2), (PANEL_T, CAB_W, BOTTOM_RAIL_H), wood),
        # counter and basin (basin is visual only)
        box('counter', (cx - COUNTER_OVERHANG / 2, 0, CAB_H + COUNTER_T / 2),
            (CAB_D + COUNTER_OVERHANG, CAB_W + 0.06, COUNTER_T), white),
        box('basin', (DRAIN_X, 0, CAB_H + COUNTER_T + 0.001), (0.34, 0.44, 0.002), steel, collide=False),
        cyl('faucet', (CAB_BACK_X - 0.06, 0, CAB_H + COUNTER_T + 0.12), (0, 0, 0), 0.015, 0.24, steel),
    ])

    tail_len = (CAB_H - PANEL_T) - TRAP_BOTTOM_Z
    riser_x = DRAIN_X + 0.10
    riser_len = TRAP_ARM_Z - TRAP_BOTTOM_Z
    arm_len = CAB_BACK_X - riser_x
    trap = ''.join([
        cyl('tailpiece', (DRAIN_X, 0, TRAP_BOTTOM_Z + tail_len / 2), (0, 0, 0), PIPE_R, tail_len, pipe_c),
        cyl('u_bottom', (DRAIN_X + 0.05, 0, TRAP_BOTTOM_Z), (0, 1.5708, 0), PIPE_R, 0.10 + 2 * PIPE_R, pipe_c),
        cyl('u_riser', (riser_x, 0, TRAP_BOTTOM_Z + riser_len / 2), (0, 0, 0), PIPE_R, riser_len, pipe_c),
        cyl('trap_arm', (riser_x + arm_len / 2, 0, TRAP_ARM_Z), (0, 1.5708, 0), PIPE_R, arm_len, pipe_c),
    ])

    # a little furniture so the lidar has something to map
    bed = box('bed', (-1.5, 1.6, 0.30), (2.0, 1.0, 0.60), (0.75, 0.82, 0.90))
    cart = box('cart', (1.2, -2.3, 0.45), (0.6, 0.45, 0.90), (0.7, 0.7, 0.72))

    sdf = f"""<?xml version="1.0"?>
<!-- Generated by tools/make_world.py. Edit that file, not this one. -->
<sdf version="1.9">
  <world name="hospital_sink">
    <physics type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>333</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="libgz-sim-physics-system.so"/>
    <plugin name="gz::sim::systems::UserCommands" filename="libgz-sim-user-commands-system.so"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="libgz-sim-scene-broadcaster-system.so"/>
    <plugin name="gz::sim::systems::Sensors" filename="libgz-sim-sensors-system.so">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::Imu" filename="libgz-sim-imu-system.so"/>

    <scene>
      <ambient>0.9 0.9 0.9 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="ceiling">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 2.3 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{ROOM_X + 1} {ROOM_Y + 1}</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>{ROOM_X + 1} {ROOM_Y + 1}</size></plane></geometry>
          <material><ambient>0.7 0.72 0.7 1</ambient><diffuse>0.7 0.72 0.7 1</diffuse></material></visual>
      </link>
    </model>
{static_model('walls', walls)}
{static_model('sink_cabinet', cab)}
{static_model('p_trap', trap)}
{static_model('bed', bed)}
{static_model('cart', cart)}
{tag_model('sink_tag_left', 0, TAG_Y)}
{tag_model('sink_tag_right', 1, -TAG_Y)}
  </world>
</sdf>
"""
    with open(OUT, 'w') as f:
        f.write(sdf)
    print(f'wrote {os.path.normpath(OUT)}')
    print(f'cabinet face x = {FACE_X:.3f}, tag separation = {2 * TAG_Y:.3f} m, tag size = {TAG_SIZE} m')
    print(f'P-trap tailpiece is {FACE_X - DRAIN_X:.3f} m behind the face (negative = inside)')


if __name__ == '__main__':
    build()
