#!/usr/bin/env python3
# Copyright 2021 Emerson Knapp
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import threading

from neato_msgs.msg import NeatoWheelCommand
import rclpy
from rclpy.node import Node

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


USAGE = """
This node takes keypresses from the keyboard to create a current command NeatoWheelCommand -
and repeatedly publishes them at a constant rate.
It works best with a US keyboard layout.
---------------------------
Moving around:
   q    w   e
   a    s   d

q/a : increase/decrease left wheel distance
w/s : increase/decrease right wheel distance
e/d : increase/decrease speed
Anything else clears command to 0
CTRL-C to quit
"""

bindings = {
    'q': lambda l, r, s, a: (l + 1.0, r, s, a),
    'a': lambda l, r, s, a: (l - 1.0, r, s, a),
    'w': lambda l, r, s, a: (l, r + 1.0, s, a),
    's': lambda l, r, s, a: (l, r - 1.0, s, a),
    'e': lambda l, r, s, a: (l, r, s * 1.1, a),
    'd': lambda l, r, s, a: (l, r, s * 0.9, a),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class NeatoTeleop(Node):
    def __init__(self):
        super().__init__('neato_teleop')
        self.cmd_pub = self.create_publisher(NeatoWheelCommand, 'neato_cmd', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.lock = threading.Lock()
        self.msg = NeatoWheelCommand()
        self.msg.left_dist = 0.0
        self.msg.right_dist = 0.0
        self.msg.speed = 0.0
        self.msg.accel = 0.0

    def timer_callback(self):
        with self.lock:
            self.cmd_pub.publish(self.msg)

    def set_cmd(self, left_dist, right_dist, speed, accel):
        with self.lock:
            self.msg.left_dist = left_dist
            self.msg.right_dist = right_dist
            self.msg.speed = speed
            self.msg.accel = accel
            self.cmd_pub.publish(self.msg)

    def print_cmd(self):
        print(f'Cmd: (L: {self.msg.left_dist}, R: {self.msg.right_dist}, '
              f'S: {self.msg.speed}, A: {self.msg.accel})')


def main():
    settings = saveTerminalSettings()
    rclpy.init()

    node = NeatoTeleop()

    left_dist = 0.0
    right_dist = 0.0
    speed = 100.0
    accel = 0.0
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
    spin_thread.start()

    try:
        print(USAGE)
        node.set_cmd(left_dist, right_dist, speed, accel)
        node.print_cmd()
        while rclpy.ok():
            key = getKey(settings)
            if key in bindings.keys():
                left_dist, right_dist, speed, accel = bindings[key](
                    left_dist, right_dist, speed, accel)
            else:
                left_dist = right_dist = 0.0
                speed = 100.0
                accel = 0.0

            node.set_cmd(left_dist, right_dist, speed, accel)
            node.print_cmd()

            # Ctrl-C
            if (key == '\x03'):
                break

    except Exception as e:  # NOQA
        print(e)
    finally:
        restoreTerminalSettings(settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
