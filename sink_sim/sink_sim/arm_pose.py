#!/usr/bin/env python3
"""Send the UR5 to a joint pose through its FollowJointTrajectory controller.

Finds the arm's trajectory action and joint names on its own, so you don't
need to know Clearpath's exact controller names.

    ros2 run sink_sim arm_pose --ros-args -r __ns:=/a200_0000 -p use_sim_time:=true
    ros2 run sink_sim arm_pose --ros-args -r __ns:=/a200_0000 -p use_sim_time:=true -p pose:=up
    ros2 run sink_sim arm_pose --ros-args -r __ns:=/a200_0000 -p use_sim_time:=true \
        -p joints:="[0.0, -1.2, 1.0, -1.4, -1.57, 0.0]"

Joint order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 (radians).
"""

import time

import rclpy
from rclpy.action import ActionClient, get_action_names_and_types
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

SUFFIXES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

POSES = {
    # arm straight up: small footprint, clears the counter overhang while docking
    'up': [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
}


class ArmPose(Node):

    def __init__(self):
        super().__init__('arm_pose')
        self.pose_name = self.declare_parameter('pose', 'up').value
        self.joints = list(self.declare_parameter('joints', [0.0]).value)
        self.duration = float(self.declare_parameter('duration', 4.0).value)
        self.arm = self.declare_parameter('arm', 'arm_0').value
        self.joint_names = None
        self.create_subscription(JointState, 'platform/joint_states', self.on_js, 10)
        self.create_subscription(JointState, 'manipulators/joint_states', self.on_js, 10)
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)

    def on_js(self, msg):
        if self.joint_names:
            return
        names = []
        for s in SUFFIXES:
            match = [n for n in msg.name if n.endswith(s) and self.arm in n]
            if match:
                names.append(match[0])
        if len(names) == 6:
            self.joint_names = names

    def find_action(self):
        for name, types in get_action_names_and_types(self):
            if ('control_msgs/action/FollowJointTrajectory' in types
                    and self.arm in name and 'gripper' not in name):
                return name
        return None

    def run(self):
        target = self.joints if len(self.joints) == 6 else POSES.get(self.pose_name)
        if target is None:
            self.get_logger().error(f'unknown pose {self.pose_name}; known: {list(POSES)}')
            return False
        deadline = time.time() + 15.0
        action = None
        while time.time() < deadline and (action is None or self.joint_names is None):
            rclpy.spin_once(self, timeout_sec=0.2)
            action = action or self.find_action()
        if action is None or self.joint_names is None:
            self.get_logger().error(
                f'could not find the arm (action={action}, joints={self.joint_names}). '
                'Is the sim running and the namespace set with -r __ns:=/a200_0000 ?')
            return False
        self.get_logger().info(f'using {action}')
        client = ActionClient(self, FollowJointTrajectory, action)
        client.wait_for_server(timeout_sec=10.0)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in target]
        pt.time_from_start = Duration(seconds=self.duration).to_msg()
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        self.get_logger().info('done' if code == 0 else f'finished with error code {code}')
        return code == 0


def main():
    rclpy.init()
    node = ArmPose()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
