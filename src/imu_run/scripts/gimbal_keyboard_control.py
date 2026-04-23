#!/usr/bin/env python3

import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist


HELP_TEXT = """
Gimbal Keyboard Control
-----------------------
w / s : front / back
a / d : left / right
q / e : left-right + front-back diagonal
z / c : left-right + front-back diagonal
space : stop all motion

CTRL-C to quit
"""


KEY_BINDINGS = {
    "w": (0.0, -1.0),
    "s": (0.0, 1.0),
    "a": (1.0, 0.0),
    "d": (-1.0, 0.0),
    "q": (1.0, 1.0),
    "e": (-1.0, 1.0),
    "z": (1.0, -1.0),
    "c": (-1.0, -1.0),
}


def publish_cmd(pub, left_right_rpm, front_back_rpm):
    msg = Twist()
    msg.angular.z = left_right_rpm
    msg.angular.y = front_back_rpm
    pub.publish(msg)


def apply_key_command(key, speed_rpm, left_right_rpm, front_back_rpm):
    normalized_key = key.lower()
    if normalized_key in KEY_BINDINGS:
        left_right_scale, front_back_scale = KEY_BINDINGS[normalized_key]
        return left_right_scale * speed_rpm, front_back_scale * speed_rpm
    if normalized_key == " ":
        return 0.0, 0.0
    return left_right_rpm, front_back_rpm


def format_status_text(left_right_rpm, front_back_rpm):
    return "Current command: left-right={:.2f} rpm, front-back={:.2f} rpm".format(
        left_right_rpm, front_back_rpm
    )


def open_tty_input():
    errors = []

    if sys.stdin.isatty():
        try:
            return os.dup(sys.stdin.fileno()), "stdin", errors
        except OSError as exc:
            errors.append("stdin: {}".format(exc))

    try:
        return os.open("/dev/tty", os.O_RDONLY), "/dev/tty", errors
    except OSError as exc:
        errors.append("/dev/tty: {}".format(exc))

    return None, None, errors


def run_tty_backend(pub, speed_rpm, repeat_rate, tty_fd, source_name):
    settings = termios.tcgetattr(tty_fd)
    left_right_rpm = 0.0
    front_back_rpm = 0.0
    timeout = 1.0 / repeat_rate if repeat_rate > 0.0 else 0.05

    print(HELP_TEXT)
    rospy.loginfo("Using %s for keyboard input", source_name)

    try:
        tty.setcbreak(tty_fd)
        while not rospy.is_shutdown():
            readable, _, _ = select.select([tty_fd], [], [], timeout)
            if readable:
                key = os.read(tty_fd, 1).decode("utf-8", errors="ignore")
                if key == "\x03":
                    break
                left_right_rpm, front_back_rpm = apply_key_command(
                    key, speed_rpm, left_right_rpm, front_back_rpm
                )

            publish_cmd(pub, left_right_rpm, front_back_rpm)
    finally:
        publish_cmd(pub, 0.0, 0.0)
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, settings)
        os.close(tty_fd)


def run_tk_backend(pub, speed_rpm, repeat_rate):
    try:
        import tkinter as tk
    except ImportError as exc:
        rospy.logerr("Tk backend is unavailable: %s", exc)
        return False

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        rospy.logerr("Failed to create Tk keyboard window: %s", exc)
        return False

    root.title("Gimbal Keyboard Control")
    root.geometry("420x260")
    root.resizable(False, False)

    status_var = tk.StringVar(value=format_status_text(0.0, 0.0))
    hint_var = tk.StringVar(
        value="Click this window, then use W/A/S/D/Q/E/Z/C or Space."
    )

    state = {"left_right": 0.0, "front_back": 0.0, "running": True}
    publish_interval = 1.0 / repeat_rate if repeat_rate > 0.0 else 0.05

    def update_command(key):
        state["left_right"], state["front_back"] = apply_key_command(
            key, speed_rpm, state["left_right"], state["front_back"]
        )
        status_var.set(
            format_status_text(state["left_right"], state["front_back"])
        )

    def normalize_tk_key(event):
        if event.char:
            normalized_char = event.char.lower()
            if normalized_char in KEY_BINDINGS or normalized_char == " ":
                return normalized_char

        if event.keysym == "space":
            return " "
        if event.keysym == "Escape":
            return " "
        return None

    def on_key_press(event):
        key = normalize_tk_key(event)
        if key is not None:
            update_command(key)

    def on_focus_out(_event):
        update_command(" ")
        hint_var.set("Window lost focus, motor commands were stopped for safety.")

    def on_focus_in(_event):
        hint_var.set("Use W/A/S/D/Q/E/Z/C or Space. Press Esc to stop.")

    def on_close():
        state["running"] = False

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<KeyPress>", on_key_press)
    root.bind("<FocusOut>", on_focus_out)
    root.bind("<FocusIn>", on_focus_in)
    root.bind("<Button-1>", lambda _event: root.focus_force())

    tk.Label(
        root,
        text="Gimbal Keyboard Control",
        font=("Helvetica", 16, "bold"),
    ).pack(pady=(14, 8))
    tk.Label(
        root,
        text="w/s: front-back   a/d: left-right   q/e/z/c: diagonal   space: stop",
        wraplength=380,
        justify="center",
    ).pack(pady=4)
    tk.Label(
        root,
        textvariable=hint_var,
        wraplength=380,
        justify="center",
        fg="#1f4d6b",
    ).pack(pady=(8, 6))
    tk.Label(
        root,
        textvariable=status_var,
        wraplength=380,
        justify="center",
        fg="#0b6e4f",
    ).pack(pady=(4, 12))
    tk.Label(
        root,
        text="Close the window to stop this node.",
        wraplength=380,
        justify="center",
    ).pack(pady=(0, 10))

    print(HELP_TEXT)
    rospy.loginfo("Using Tk keyboard window because no terminal input is available")

    try:
        root.focus_force()
        while not rospy.is_shutdown() and state["running"]:
            root.update_idletasks()
            root.update()
            publish_cmd(pub, state["left_right"], state["front_back"])
            time.sleep(publish_interval)
    except tk.TclError:
        pass
    finally:
        publish_cmd(pub, 0.0, 0.0)
        try:
            root.destroy()
        except tk.TclError:
            pass

    return True


def main():
    rospy.init_node("gimbal_keyboard_control")

    cmd_topic = rospy.get_param("~cmd_topic", "/imu_cmd_vel")
    speed_rpm = float(rospy.get_param("~speed_rpm", 10.0))
    repeat_rate = float(rospy.get_param("~repeat_rate", 20.0))
    keyboard_backend = rospy.get_param("~keyboard_backend", "auto").strip().lower()

    if keyboard_backend not in ("auto", "tty", "tk"):
        rospy.logwarn(
            "Unsupported keyboard backend '%s', falling back to auto",
            keyboard_backend,
        )
        keyboard_backend = "auto"

    pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)

    rospy.loginfo(
        "Publishing gimbal commands to %s with step speed %.2f rpm",
        cmd_topic,
        speed_rpm,
    )
    rospy.loginfo(
        "Twist mapping: angular.z = left-right motor rpm, angular.y = front-back motor rpm"
    )

    tty_fd = None
    tty_source = None
    tty_errors = []
    if keyboard_backend in ("auto", "tty"):
        tty_fd, tty_source, tty_errors = open_tty_input()

    if tty_fd is not None:
        run_tty_backend(pub, speed_rpm, repeat_rate, tty_fd, tty_source)
        return

    if keyboard_backend == "tty":
        rospy.logerr("TTY backend requested but no terminal input is available: %s", "; ".join(tty_errors))
        return

    if run_tk_backend(pub, speed_rpm, repeat_rate):
        return

    error_parts = tty_errors if tty_errors else ["no terminal input was detected"]
    rospy.logerr(
        "No usable keyboard input backend is available. TTY errors: %s",
        "; ".join(error_parts),
    )


if __name__ == "__main__":
    main()
