#!/usr/bin/env python3
"""ROS 2 wrapper around align_core.Aligner.

Subscribes (via TF):  odom -> base_link, camera -> sink_tag_left / sink_tag_right
Publishes:            cmd_vel (Twist or TwistStamped), ~/state, ~/errors
Broadcasts TF:        odom -> sink_face, sink_face -> p_trap
Services:             ~/start, ~/cancel  (std_srvs/Trigger)
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, Vector3
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from sink_sim.align_core import Aligner, Params


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SinkAligner(Node):

    def __init__(self):
        super().__init__('sink_aligner')
        d = self.declare_parameter
        self.autostart = d('autostart', True).value
        rate = d('rate', 20.0).value
        self.odom_frame = d('odom_frame', 'odom').value
        self.base_frame = d('base_frame', 'base_link').value
        self.tag_l = d('tag_left_frame', 'sink_tag_left').value
        self.tag_r = d('tag_right_frame', 'sink_tag_right').value
        self.max_tag_age = d('max_tag_age', 0.5).value
        self.p_trap_offset = list(d('p_trap_offset', [-0.27, 0.0, 0.0]).value)
        self.stamped_mode = str(d('cmd_vel_stamped', 'auto').value).lower()

        p = Params()
        p.holonomic = bool(d('holonomic', p.holonomic).value)
        for name in ('standoff', 'lateral_offset', 'predock_distance', 'predock_tol',
                     'expected_separation', 'separation_tol', 'tol_x', 'tol_y', 'tol_yaw',
                     'max_lin', 'max_ang', 'search_ang', 'search_timeout'):
            setattr(p, name, float(d(name, getattr(p, name)).value))
        self.params = p
        self.aligner = Aligner(p)
        self.active = bool(self.autostart)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_pub = None
        self.cmd_stamped = None
        self.state_pub = self.create_publisher(String, '~/state', 10)
        self.err_pub = self.create_publisher(Vector3, '~/errors', 10)
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/cancel', self.on_cancel)

        self.last_tag_stamp = None
        self.tag_z = 0.0
        self.last_logged_state = None
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'sink_aligner ready ({"holonomic" if p.holonomic else "diff drive"}, '
            f'standoff {p.standoff:.2f} m, autostart {self.active})')

    # -- services ----------------------------------------------------------------
    def on_start(self, _req, res):
        self.aligner.reset()
        self.active = True
        res.success, res.message = True, 'alignment started'
        return res

    def on_cancel(self, _req, res):
        self.active = False
        self.send(0.0, 0.0, 0.0)
        res.success, res.message = True, 'alignment cancelled'
        return res

    # -- cmd_vel -----------------------------------------------------------------
    def ensure_cmd_pub(self):
        if self.cmd_pub is not None:
            return
        stamped = True
        if self.stamped_mode in ('true', 'false'):
            stamped = self.stamped_mode == 'true'
        else:
            try:
                topic = self.resolve_topic_name('cmd_vel')
            except AttributeError:
                ns = self.get_namespace().rstrip('/')
                topic = f'{ns}/cmd_vel'
            types = dict(self.get_topic_names_and_types()).get(topic, [])
            if 'geometry_msgs/msg/Twist' in types and 'geometry_msgs/msg/TwistStamped' not in types:
                stamped = False
            self.get_logger().info(f'{topic} types seen: {types or "none yet"}; '
                                   f'publishing {"TwistStamped" if stamped else "Twist"}')
        self.cmd_stamped = stamped
        self.cmd_pub = self.create_publisher(TwistStamped if stamped else Twist, 'cmd_vel', 10)

    def send(self, vx, vy, w):
        self.ensure_cmd_pub()
        if self.cmd_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            tw = msg.twist
        else:
            msg = Twist()
            tw = msg
        tw.linear.x, tw.linear.y, tw.angular.z = float(vx), float(vy), float(w)
        self.cmd_pub.publish(msg)

    # -- TF helpers ----------------------------------------------------------------
    def lookup(self, child):
        try:
            return self.tf_buffer.lookup_transform(self.odom_frame, child, Time())
        except Exception:  # noqa: BLE001 - tf2 raises several types
            return None

    def robot_pose(self):
        tf = self.lookup(self.base_frame)
        if tf is None:
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_from_quat(t.rotation))

    def feed_tags(self, pose):
        tl, tr = self.lookup(self.tag_l), self.lookup(self.tag_r)
        if tl is None or tr is None:
            return
        now = self.get_clock().now()
        st_l, st_r = Time.from_msg(tl.header.stamp), Time.from_msg(tr.header.stamp)
        max_age = Duration(seconds=self.max_tag_age)
        if now - st_l > max_age or now - st_r > max_age:
            return
        if self.last_tag_stamp is not None and st_l == self.last_tag_stamp:
            return  # same detection as last tick
        self.last_tag_stamp = st_l
        a, b = tl.transform.translation, tr.transform.translation
        if self.aligner.add_tags((a.x, a.y), (b.x, b.y), pose):
            self.tag_z = 0.5 * (a.z + b.z)

    def broadcast_face(self):
        f = self.aligner.face
        if f is None:
            return
        now = self.get_clock().now().to_msg()
        face = TransformStamped()
        face.header.stamp = now
        face.header.frame_id = self.odom_frame
        face.child_frame_id = 'sink_face'
        face.transform.translation.x = f.x
        face.transform.translation.y = f.y
        face.transform.translation.z = self.tag_z
        face.transform.rotation.z = math.sin(f.yaw / 2.0)
        face.transform.rotation.w = math.cos(f.yaw / 2.0)
        trap = TransformStamped()
        trap.header.stamp = now
        trap.header.frame_id = 'sink_face'
        trap.child_frame_id = 'p_trap'
        trap.transform.translation.x = float(self.p_trap_offset[0])
        trap.transform.translation.y = float(self.p_trap_offset[1])
        trap.transform.translation.z = float(self.p_trap_offset[2])
        trap.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([face, trap])

    # -- main loop -------------------------------------------------------------------
    def tick(self):
        pose = self.robot_pose()
        if pose is None:
            return
        self.feed_tags(pose)
        self.broadcast_face()
        a = self.aligner
        if self.active:
            t = self.get_clock().now().nanoseconds * 1e-9
            vx, vy, w = a.step(pose, t)
            self.send(vx, vy, w)
            if a.state in (a.DONE, a.FAILED):
                self.send(0.0, 0.0, 0.0)
                self.active = False
        self.state_pub.publish(String(data=a.state))
        if a.last_error is not None:
            ex, ey, eyaw = a.last_error
            self.err_pub.publish(Vector3(x=ex, y=ey, z=eyaw))
        if a.state != self.last_logged_state:
            extra = ''
            if a.last_error is not None:
                ex, ey, eyaw = a.last_error
                extra = (f' (along {ex * 100:+.1f} cm, lateral {ey * 100:+.1f} cm, '
                         f'yaw {math.degrees(eyaw):+.2f} deg)')
            if a.state == a.FAILED:
                extra += f' reason: {a.reason}'
            self.get_logger().info(f'state -> {a.state}{extra}')
            self.last_logged_state = a.state


def main():
    rclpy.init()
    node = SinkAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
