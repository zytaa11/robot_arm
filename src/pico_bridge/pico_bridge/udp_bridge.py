#!/usr/bin/env python3
import json, socket, threading, time
import rclpy
from rclpy.node import Node
from xr_msgs.msg import Controller

class UDPBridge(Node):
    def __init__(self):
        super().__init__("udp_bridge")
        self.declare_parameter("udp_ip", "0.0.0.0")
        self.declare_parameter("udp_port", 5005)
        self.declare_parameter("publish_rate", 50.0)
        ip = self.get_parameter("udp_ip").value
        port = int(self.get_parameter("udp_port").value)
        rate = float(self.get_parameter("publish_rate").value)
        self.pub_left = self.create_publisher(Controller, "/picoxr/left_controller", 10)
        self.pub_right = self.create_publisher(Controller, "/picoxr/right_controller", 10)
        self.lock = threading.Lock()
        self.left = Controller()
        self.right = Controller()
        self.last_udp = 0.0
        threading.Thread(target=self._udp_loop, args=(ip, port), daemon=True).start()
        self.create_timer(1.0/rate, self._timer_cb)
        self.get_logger().info(f"UDP bridge on {ip}:{port}")

    def _udp_loop(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))
        sock.settimeout(0.1)
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except:
                continue
            try:
                j = json.loads(data.decode())
            except:
                continue
            ctrl = Controller()
            ctrl.pose = [float(j.get(k, 0)) for k in ["px","py","pz","qx","qy","qz","qw"]]
            ctrl.trigger = float(j.get("trigger", 0))
            ctrl.gripper = float(j.get("grip", 0))
            ctrl.primary_button = bool(j.get("buttonA", False))
            ctrl.secondary_button = bool(j.get("buttonB", False))
            hand = j.get("hand", "right")
            with self.lock:
                if hand == "left":
                    self.left = ctrl
                else:
                    self.right = ctrl
                self.last_udp = time.time()

    def _timer_cb(self):
        now = time.time()
        with self.lock:
            if now - self.last_udp > 0.5:
                self.pub_left.publish(Controller())
                self.pub_right.publish(Controller())
                return
            self.pub_left.publish(self.left)
            self.pub_right.publish(self.right)

def main():
    rclpy.init()
    node = UDPBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
