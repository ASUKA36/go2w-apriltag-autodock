#!/usr/bin/env python3
"""Keyboard teleoperation for the simplified Go2-W model."""

import select
import os
import shutil
import subprocess
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


HELP = """
Go2-W keyboard control
----------------------
W/S : forward/backward    A/D : left/right strafe
Q/E : turn left/right     Space: stop
Z   : lie down (StandDown) X   : stand up
+/- : change speed        Ctrl-C: quit

On WSL, key-down/key-up states are read from Windows directly. Multiple
movement keys may be held together and releasing all keys stops immediately.
"""


class KeyboardTeleop:
    def __init__(self):
        self.linear_speed = rospy.get_param("~linear_speed", 0.45)
        self.angular_speed = rospy.get_param("~angular_speed", 0.9)
        self.rate = rospy.get_param("~publish_rate", 20.0)
        self.release_timeout = rospy.get_param("~release_timeout", 0.18)
        self.initial_repeat_delay = rospy.get_param("~initial_repeat_delay", 0.65)
        self.command = Twist()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.posture_pub = rospy.Publisher("/go2w/posture_cmd", String, queue_size=1, latch=True)
        self.settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        self._previous_keys = set()

    def read_key(self, timeout):
        tty.setraw(sys.stdin.fileno())
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if ready else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key.lower()

    def set_motion(self, x=0.0, y=0.0, yaw=0.0):
        self.command = Twist()
        self.command.linear.x = x
        self.command.linear.y = y
        self.command.angular.z = yaw

    def apply_keys(self, keys):
        x = self.linear_speed * (("w" in keys) - ("s" in keys))
        y = self.linear_speed * (("a" in keys) - ("d" in keys))
        yaw = self.angular_speed * (("q" in keys) - ("e" in keys))
        self.set_motion(x=x, y=y, yaw=yaw)

        rising = keys - self._previous_keys
        if "z" in rising:
            self.set_motion()
            self.posture_pub.publish("lie_down")
            rospy.loginfo("StandDown: lie_down")
        if "x" in rising:
            self.posture_pub.publish("stand_up")
            rospy.loginfo("StandUp: stand")
        if "+" in rising:
            self.linear_speed = min(1.2, self.linear_speed * 1.1)
            self.angular_speed = min(1.8, self.angular_speed * 1.1)
            rospy.loginfo("speed: %.2f m/s, %.2f rad/s", self.linear_speed, self.angular_speed)
        if "-" in rising:
            self.linear_speed = max(0.05, self.linear_speed * 0.9)
            self.angular_speed = max(0.1, self.angular_speed * 0.9)
            rospy.loginfo("speed: %.2f m/s, %.2f rad/s", self.linear_speed, self.angular_speed)
        self._previous_keys = keys
        self.cmd_pub.publish(self.command)

    @staticmethod
    def windows_key_process():
        # GetAsyncKeyState exposes real key-down/key-up state to a WSL process.
        script = r"""
Add-Type -TypeDefinition 'using System.Runtime.InteropServices; public static class WslKeys { [DllImport("user32.dll")] public static extern short GetAsyncKeyState(int key); }'
$keys = [ordered]@{w=87;s=83;a=65;d=68;q=81;e=69;z=90;x=88;' '=32;'+'=187;'-'=189}
while ($true) {
  $down = @()
  foreach ($entry in $keys.GetEnumerator()) {
    if (([WslKeys]::GetAsyncKeyState($entry.Value) -band 0x8000) -ne 0) { $down += $entry.Key }
  }
  [Console]::WriteLine(($down -join ','))
  [Console]::Out.Flush()
  Start-Sleep -Milliseconds 20
}
"""
        return subprocess.Popen(
            ["powershell.exe", "-NoProfile", "-Command", script],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def run_wsl(self):
        process = self.windows_key_process()
        rospy.loginfo("keyboard backend: Windows key-down/key-up (WSL)")
        try:
            while not rospy.is_shutdown() and process.poll() is None:
                ready, _, _ = select.select([process.stdout], [], [], 1.0 / self.rate)
                if not ready:
                    continue
                line = process.stdout.readline().strip()
                keys = set(filter(None, line.split(",")))
                self.apply_keys(keys)
        finally:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()

    def run_terminal(self):
        if self.settings is None:
            raise RuntimeError("terminal keyboard backend requires an interactive TTY")
        rospy.logwarn("keyboard backend: terminal repeat watchdog (key-up is unavailable)")
        active_key = None
        deadline = 0.0
        while not rospy.is_shutdown():
            key = self.read_key(1.0 / self.rate)
            now = time.monotonic()
            if key in "wasdqe":
                repeated = key == active_key and now <= deadline
                active_key = key
                deadline = now + (self.release_timeout if repeated else self.initial_repeat_delay)
                self.apply_keys({key})
            elif key in ("z", "x", "+", "=", "-", "_"):
                normalized = "+" if key in ("+", "=") else "-" if key in ("-", "_") else key
                self.apply_keys({normalized})
                self.apply_keys(set())
            elif key == " " or (active_key and now > deadline):
                active_key = None
                self.apply_keys(set())
            elif key == "\x03":
                break

    def run(self):
        print(HELP)
        try:
            if os.environ.get("WSL_DISTRO_NAME") and shutil.which("powershell.exe"):
                self.run_wsl()
            else:
                self.run_terminal()
        finally:
            self.cmd_pub.publish(Twist())
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rospy.init_node("go2w_keyboard")
    KeyboardTeleop().run()
