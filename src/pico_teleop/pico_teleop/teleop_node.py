#!/usr/bin/env python3
import math, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from xr_msgs.msg import Controller
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger

class PicoTeleopNode(Node):
    def __init__(self):
        super().__init__("pico_teleop")
        # Params
        self.declare_parameter("trigger_threshold", 0.6)
        self.declare_parameter("linear_scale", 0.5)
        self.declare_parameter("angular_scale", 1.0)
        self.declare_parameter("max_linear_speed", 0.1)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("deadzone", 0.003)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("timeout_sec", 0.3)
        self.declare_parameter("frame_id", "eeflink")
        self.declare_parameter("home_action", "/robot_arm_group_controller/follow_joint_trajectory")
        self.declare_parameter("home_joints", ["Joint_1","Joint_2","Joint_3","Joint_4","Joint_5","Joint_6","Joint_7"])
        self.declare_parameter("home_positions", [0.0, 0.0, 0.0, 1.5, 0.0, 0.1, 0.0])
        self.declare_parameter("home_duration", 3.0)
        self.trig_thr = float(self.get_parameter("trigger_threshold").value)
        self.lin_scale = float(self.get_parameter("linear_scale").value)
        self.ang_scale = float(self.get_parameter("angular_scale").value)
        self.max_lin = float(self.get_parameter("max_linear_speed").value)
        self.max_ang = float(self.get_parameter("max_angular_speed").value)
        self.dead = float(self.get_parameter("deadzone").value)
        self.freq = float(self.get_parameter("publish_rate").value)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        self.frame = self.get_parameter("frame_id").value
        home_action = self.get_parameter("home_action").value
        self.home_joints = list(self.get_parameter("home_joints").value)
        self.home_pos = list(self.get_parameter("home_positions").value)
        self.home_dur = float(self.get_parameter("home_duration").value)
        # State
        self.lock = threading.Lock()
        self.controller = Controller()
        self.last_time = 0.0
        self.prev_pose = None
        self.returning_home = False
        self.prev_a = False
        self.home_timer = None
        # Pub/Sub
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.sub = self.create_subscription(Controller, "/picoxr/right_controller", self.cb, 10)
        # Clients
        self.home_client = ActionClient(self, FollowJointTrajectory, home_action)
        self.pause_cli = self.create_client(Trigger, "/servo_node/pause_servo")
        self.unpause_cli = self.create_client(Trigger, "/servo_node/unpause_servo")
        # Timer
        self.create_timer(1.0/self.freq, self.timer_cb)
        self.get_logger().info("pico_teleop node started")

    def cb(self, msg):
        with self.lock:
            self.controller = msg
            self.last_time = time.time()

    def timer_cb(self):
        now = time.time()
        with self.lock:
            ctrl = self.controller
            last = self.last_time
        if now - last > self.timeout:
            self.pub_zero()
            return
        if ctrl.secondary_button:
            self.pub_zero()
            self.get_logger().warn("EM STOP (B)")
            return
        if ctrl.primary_button and not self.prev_a:
            self.prev_a = True
            self.go_home()
            return
        self.prev_a = ctrl.primary_button
        if self.returning_home:
            self.pub_zero()
            return
        if ctrl.trigger < self.trig_thr:
            self.pub_zero()
            return
        p = ctrl.pose
        if self.prev_pose is None:
            self.prev_pose = list(p)
            self.pub_zero()
            return
        dx = p[0] - self.prev_pose[0]
        dy = p[1] - self.prev_pose[1]
        dz = p[2] - self.prev_pose[2]
        rx, ry, rz = self.quat_delta(self.prev_pose[3:7], p[3:7])
        self.prev_pose = list(p)
        vx = self.clamp(self.deadzone(dx * self.lin_scale * self.freq), self.max_lin)
        vy = self.clamp(self.deadzone(dy * self.lin_scale * self.freq), self.max_lin)
        vz = self.clamp(self.deadzone(dz * self.lin_scale * self.freq), self.max_lin)
        wx = self.clamp(self.deadzone(rx * self.ang_scale * self.freq), self.max_ang)
        wy = self.clamp(self.deadzone(ry * self.ang_scale * self.freq), self.max_ang)
        wz = self.clamp(self.deadzone(rz * self.ang_scale * self.freq), self.max_ang)
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.twist.linear.x = vx
        t.twist.linear.y = vy
        t.twist.linear.z = vz
        t.twist.angular.x = wx
        t.twist.angular.y = wy
        t.twist.angular.z = wz
        self.twist_pub.publish(t)

    def quat_delta(self, a, b):
        def qinv(q): return (-q[0], -q[1], -q[2], q[3])
        def qmul(p, q):
            return (p[3]*q[0]+p[0]*q[3]+p[1]*q[2]-p[2]*q[1],
                    p[3]*q[1]-p[0]*q[2]+p[1]*q[3]+p[2]*q[0],
                    p[3]*q[2]+p[0]*q[1]-p[1]*q[0]+p[2]*q[3],
                    p[3]*q[3]-p[0]*q[0]-p[1]*q[1]-p[2]*q[2])
        d = qmul(b, qinv(a))
        if d[3] < 0: d = tuple(-x for x in d)
        s = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        if s < 1e-9: return (0.0, 0.0, 0.0)
        ang = 2 * math.atan2(s, d[3])
        return (d[0]/s*ang, d[1]/s*ang, d[2]/s*ang)

    def deadzone(self, v): return 0.0 if abs(v) < self.dead else float(v)
    def clamp(self, v, m): return float(max(-m, min(m, v)))
    def pub_zero(self):
        self.twist_pub.publish(TwistStamped())

    def go_home(self):
        self.get_logger().warn("HOME: pause servo, send goal...")
        self.returning_home = True
        self.pub_zero()
        req = Trigger.Request()
        if self.pause_cli.wait_for_service(0.5):
            self.pause_cli.call_async(req)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.home_joints
        p = JointTrajectoryPoint()
        p.positions = self.home_pos
        p.time_from_start.sec = int(self.home_dur)
        goal.trajectory.points.append(p)
        if self.home_client.wait_for_server(1.0):
            self.home_client.send_goal_async(goal)
        self.home_timer = self.create_timer(self.home_dur + 1.0, self.home_cb)

    def home_cb(self):
        if self.home_timer: self.home_timer.cancel()
        self.returning_home = False
        req = Trigger.Request()
        if self.unpause_cli.wait_for_service(0.5):
            self.unpause_cli.call_async(req)
        self.get_logger().warn("HOME done")

def main():
    rclpy.init()
    node = PicoTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
