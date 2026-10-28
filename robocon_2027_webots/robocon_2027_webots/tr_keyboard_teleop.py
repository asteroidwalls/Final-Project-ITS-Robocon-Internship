import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


HELP_TEXT = """
==================== TR KEYBOARD TELEOP ====================
 Gerak (latched, tiap sumbu independen -> bisa diagonal):
     w : maju (+X)            q : putar kiri  (CCW, +wz)
     s : mundur (-X)          e : putar kanan (CW,  -wz)
     a : geser kiri  (+Y)
     d : geser kanan (-Y)

 SPASI : stop semua             c : stop rotasi saja

 m : ganti mode LOCAL <-> GLOBAL (robot berhenti saat ganti mode)
 r / f : kecepatan linear  naik / turun
 t / g : kecepatan angular naik / turun

 Ctrl-C : keluar
 Mode GLOBAL: w/s/a/d searah sumbu +X/-X/+Y/-Y frame 'odom'.
============================================================
"""


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def global_to_body(vx_g, vy_g, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vx_b = cos_yaw * vx_g + sin_yaw * vy_g
    vy_b = -sin_yaw * vx_g + cos_yaw * vy_g
    return vx_b, vy_b


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class TRKeyboardTeleop(Node):

    LOCAL = 'local'
    GLOBAL = 'global'

    def __init__(self):
        super().__init__('tr_keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_angular_speed', 3.0)
        self.declare_parameter('linear_accel', 2.0)
        self.declare_parameter('angular_accel', 6.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('start_mode', self.LOCAL)

        self._linear_speed = float(self.get_parameter('linear_speed').value)
        self._angular_speed = float(self.get_parameter('angular_speed').value)
        self._max_linear = float(self.get_parameter('max_linear_speed').value)
        self._max_angular = float(self.get_parameter('max_angular_speed').value)
        self._linear_accel = float(self.get_parameter('linear_accel').value)
        self._angular_accel = float(self.get_parameter('angular_accel').value)
        self._rate = float(self.get_parameter('publish_rate').value)

        start_mode = str(self.get_parameter('start_mode').value).lower()
        self._mode = self.GLOBAL if start_mode == self.GLOBAL else self.LOCAL

        self._direction = [0.0, 0.0, 0.0]
        self._smoothed = [0.0, 0.0, 0.0]

        self._yaw = 0.0
        self._odom_received = False
        self._last_status_time = 0.0

        self._publisher = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            10,
        )
        self.create_timer(1.0 / self._rate, self._publish_command)


    def _odom_callback(self, message):
        q = message.pose.pose.orientation
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._odom_received = True

    @staticmethod
    def _slew(current, target, max_step):
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    def _publish_command(self):
        dt = 1.0 / self._rate
        targets = (
            self._direction[0] * self._linear_speed,
            self._direction[1] * self._linear_speed,
            self._direction[2] * self._angular_speed,
        )
        steps = (
            self._linear_accel * dt,
            self._linear_accel * dt,
            self._angular_accel * dt,
        )
        for i in range(3):
            self._smoothed[i] = self._slew(
                self._smoothed[i], targets[i], steps[i]
            )

        vx, vy, wz = self._smoothed

        if self._mode == self.GLOBAL:
            if self._odom_received:
                vx, vy = global_to_body(vx, vy, self._yaw)
            else:

                vx, vy, wz = 0.0, 0.0, 0.0

        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self._publisher.publish(message)

    def stop_robot(self):
        self._direction = [0.0, 0.0, 0.0]
        self._smoothed = [0.0, 0.0, 0.0]
        try:
            for _ in range(3):
                self._publisher.publish(Twist())
        except Exception:
            pass


    def handle_key(self, key):
        if key == 'w':
            self._direction[0] = 1.0
        elif key == 's':
            self._direction[0] = -1.0
        elif key == 'a':
            self._direction[1] = 1.0
        elif key == 'd':
            self._direction[1] = -1.0
        elif key == 'q':
            self._direction[2] = 1.0
        elif key == 'e':
            self._direction[2] = -1.0
        elif key == ' ':
            self._direction = [0.0, 0.0, 0.0]
            self._smoothed = [0.0, 0.0, 0.0]
        elif key == 'c':
            self._direction[2] = 0.0
            self._smoothed[2] = 0.0
        elif key == 'm':
            self._mode = (
                self.GLOBAL if self._mode == self.LOCAL else self.LOCAL
            )
            self._direction = [0.0, 0.0, 0.0]
            self._smoothed = [0.0, 0.0, 0.0]
        elif key == 'r':
            self._linear_speed = clamp(
                self._linear_speed + 0.1, 0.1, self._max_linear
            )
        elif key == 'f':
            self._linear_speed = clamp(
                self._linear_speed - 0.1, 0.1, self._max_linear
            )
        elif key == 't':
            self._angular_speed = clamp(
                self._angular_speed + 0.25, 0.25, self._max_angular
            )
        elif key == 'g':
            self._angular_speed = clamp(
                self._angular_speed - 0.25, 0.25, self._max_angular
            )

    def _read_keys(self, timeout):
        fd = sys.stdin.fileno()
        ready, _, _ = select.select([fd], [], [], timeout)
        if not ready:
            return ''
        data = os.read(fd, 32).decode(errors='ignore')
        if data.startswith('\x1b'):
            return ''
        return data.lower()

    def _print_status(self):
        now = time.monotonic()
        if now - self._last_status_time < 0.2:
            return
        self._last_status_time = now

        if self._mode == self.GLOBAL and not self._odom_received:
            note = ' | MENUNGGU /odom...'
        else:
            note = ''
        sys.stdout.write(
            '\r[{mode:6s}] lin={lin:.1f} m/s ang={ang:.2f} rad/s | '
            'cmd=({vx:+.2f},{vy:+.2f},{wz:+.2f}) | yaw={yaw:+6.1f} deg{note}'
            '\x1b[K'.format(
                mode=self._mode.upper(),
                lin=self._linear_speed,
                ang=self._angular_speed,
                vx=self._smoothed[0],
                vy=self._smoothed[1],
                wz=self._smoothed[2],
                yaw=math.degrees(self._yaw),
                note=note,
            )
        )
        sys.stdout.flush()

    def run(self):
        sys.stdout.write(HELP_TEXT.replace('\n', '\r\n'))
        sys.stdout.flush()
        while rclpy.ok():
            for key in self._read_keys(0.02):
                self.handle_key(key)
            rclpy.spin_once(self, timeout_sec=0)
            self._print_status()


def main(args=None):
    if not sys.stdin.isatty():
        print(
            'tr_keyboard_teleop harus dijalankan di terminal interaktif '
            '(gunakan "ros2 run", bukan "ros2 launch").'
        )
        return

    rclpy.init(args=args)
    node = TRKeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sys.stdout.write('\n')
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
