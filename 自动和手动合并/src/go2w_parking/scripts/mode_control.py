#!/usr/bin/env python3
"""Keyboard teleop with R-key toggle between manual and auto-dock modes."""

import os
import select
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
Go2-W parking control (manual / auto)
-------------------------------------
R     : toggle MANUAL <-> AUTO
W/S   : forward/backward     A/D : left/right strafe
Q/E   : turn left/right      Space: stop
Z     : lie down             X   : stand up
+/-   : change speed         Ctrl-C: quit

Manual mode publishes /cmd_vel from the keyboard.
Auto mode leaves /cmd_vel to auto_dock.
Switching to MANUAL always sends stand_up so the robot can move again.
"""


class ModeControl:
    MANUAL = "manual"
    AUTO = "auto"

    def __init__(self):
        self.linear_speed = rospy.get_param("~linear_speed", 0.45)
        self.angular_speed = rospy.get_param("~angular_speed", 0.9)
        self.rate = rospy.get_param("~publish_rate", 20.0)
        self.release_timeout = rospy.get_param("~release_timeout", 0.18)
        self.initial_repeat_delay = rospy.get_param("~initial_repeat_delay", 0.65)
        initial = rospy.get_param("~initial_mode", self.MANUAL).strip().lower()
        self.mode = self.AUTO if initial == self.AUTO else self.MANUAL
        self.command = Twist()
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.mode_pub = rospy.Publisher("/control_mode", String, queue_size=1, latch=True)
        self.posture_pub = rospy.Publisher("/go2w/posture_cmd", String, queue_size=1, latch=True)
        self.settings = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
        self._previous_keys = set()
        self.publish_mode(self.mode)
        # Ensure a fresh session starts standing (overrides latched lie_down).
        self.set_posture("stand_up")
        rospy.loginfo("Control mode: %s (press R to toggle, X to stand)", self.mode.upper())

    def publish_mode(self, mode):
        self.mode = mode
        self.mode_pub.publish(String(data=mode))

    def set_posture(self, command):
        self.posture_pub.publish(String(data=command))

    def stop_robot(self):
        self.command = Twist()
        self.cmd_pub.publish(self.command)

    def toggle_mode(self):
        self.stop_robot()
        next_mode = self.AUTO if self.mode == self.MANUAL else self.MANUAL
        self.publish_mode(next_mode)
        # After auto-dock may have lied down; standing is required before manual drive
        # because the planar plugin zeros cmd_vel while prone.
        if next_mode == self.MANUAL:
            self.set_posture("stand_up")
            rospy.loginfo("Control mode -> MANUAL (stand_up)")
        else:
            rospy.loginfo("Control mode -> AUTO")

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
        rising = keys - self._previous_keys
        if "r" in rising:
            self.toggle_mode()
            self._previous_keys = keys
            return

        # Posture works in both modes (needed after auto-dock lie_down).
        if "z" in rising:
            self.set_motion()
            self.cmd_pub.publish(self.command)
            self.set_posture("lie_down")
            rospy.loginfo("StandDown: lie_down")
        if "x" in rising:
            self.set_posture("stand_up")
            rospy.loginfo("StandUp: stand")

        if self.mode != self.MANUAL:
            self._previous_keys = keys
            return

        x = self.linear_speed * (("w" in keys) - ("s" in keys))
        y = self.linear_speed * (("a" in keys) - ("d" in keys))
        yaw = self.angular_speed * (("q" in keys) - ("e" in keys))
        self.set_motion(x=x, y=y, yaw=yaw)

        # Planar plugin ignores cmd_vel while prone; stand before driving.
        if rising & {"w", "s", "a", "d", "q", "e"}:
            self.set_posture("stand_up")

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
        script = r"""
Add-Type -TypeDefinition 'using System.Runtime.InteropServices; public static class WslKeys { [DllImport("user32.dll")] public static extern short GetAsyncKeyState(int key); }'
$keys = [ordered]@{w=87;s=83;a=65;d=68;q=81;e=69;r=82;z=90;x=88;' '=32;'+'=187;'-'=189}
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
            if key == "r":
                self.apply_keys({"r"})
                self.apply_keys(set())
            elif key in "wasdqe":
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
            self.stop_robot()
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == "__main__":
    rospy.init_node("parking_mode_control")
    ModeControl().run()
