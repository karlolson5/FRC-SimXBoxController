"""
Virtual Xbox controller for WPILib robot simulation.

Shows an Xbox-style controller. Click and hold buttons (triggers go to 100%),
drag the stick knobs, or use the keyboard. Right-click any button, then press
a key to remap it (Esc cancels). Controller state is sent to the simulated
robot through the HALSim WebSocket extension (halsim_ws_server) as a standard
joystick, so WPILib's XboxController / CommandXboxController reads it normally.

    pip install websocket-client
    python sim_gamepad_gui.py [--joystick 0] [--url ws://localhost:3300/wpilibws] [--no-ds] [--debug]

By default this GUI also owns the driver station state (Enable + mode), so
turn OFF the Sim GUI (halsim_gui) in the sim dialog, or the two will fight over
the joystick/DS data. If you keep the Sim GUI, pass --no-ds and set up the
robot state there (and leave its Joystick slot unassigned).

Tip: the sim's WebSocket server streams every simulated device back to us
(thousands of msgs/s, we ignore them all). Restrict it to what we need by
setting this environment variable before launching the sim:
    HALSIMWS_FILTERS=DriverStation,Joystick
"""

import argparse
import json
import math
import socket
import sys
import threading
import time
import tkinter as tk

import websocket  # pip install websocket-client

# ---- WPILib XboxController layout -------------------------------------------
# Axes:    0 LeftX, 1 LeftY, 2 LeftTrigger, 3 RightTrigger, 4 RightX, 5 RightY
# Buttons: A, B, X, Y, LB, RB, Back, Start, L3, R3  (WPILib buttons 1-10)
# Stick Y follows WPILib: up = -1, down = +1.
BTN_ORDER = ["A", "B", "X", "Y", "LB", "RB", "Back", "Start", "L3", "R3"]

# D-pad -> POV angle, keyed by (right - left, up - down)
POV_ANGLE = {(0, 1): 0, (1, 1): 45, (1, 0): 90, (1, -1): 135, (0, -1): 180, (-1, -1): 225, (-1, 0): 270, (-1, 1): 315}

# ---- On-screen layout --------------------------------------------------------
# name, label, shape, bounding box, color, default key
CONTROLS = [
    ("LT", "LT", "rect", (105, 20, 205, 62), "#b0bec5", "1"),
    ("RT", "RT", "rect", (435, 20, 535, 62), "#b0bec5", "3"),
    ("LB", "LB", "rect", (95, 70, 215, 102), "#b0bec5", "q"),
    ("RB", "RB", "rect", (425, 70, 545, 102), "#b0bec5", "e"),
    ("Back", "Back", "rect", (266, 162, 314, 192), "#eceff1", "tab"),
    ("Start", "Start", "rect", (326, 162, 374, 192), "#eceff1", "return"),
    ("L3", "L3", "rect", (147, 207, 183, 233), "#eceff1", "z"),
    ("R3", "R3", "rect", (377, 308, 413, 334), "#eceff1", "x"),
    ("Up", "▲", "rect", (232, 226, 258, 252), "#eceff1", "up"),
    ("Down", "▼", "rect", (232, 278, 258, 304), "#eceff1", "down"),
    ("Left", "◀", "rect", (206, 252, 232, 278), "#eceff1", "left"),
    ("Right", "▶", "rect", (258, 252, 284, 278), "#eceff1", "right"),
    ("Y", "Y", "oval", (448, 111, 482, 145), "#fdd835", "i"),
    ("X", "X", "oval", (411, 148, 445, 182), "#64b5f6", "u"),
    ("B", "B", "oval", (485, 148, 519, 182), "#e57373", "k"),
    ("A", "A", "oval", (448, 185, 482, 219), "#81c784", "j"),
]
BODY = [
    110,
    120,
    320,
    105,
    530,
    120,
    595,
    210,
    610,
    350,
    565,
    420,
    500,
    410,
    455,
    340,
    185,
    340,
    140,
    410,
    75,
    420,
    30,
    350,
    45,
    210,
]
STICK_POS = {"L": (165, 155), "R": (395, 255)}
RANGE, KNOB = 30, 14  # knob travel (px) and knob radius (px)
HELD_COLOR = "#ff9800"


class Link:
    """Background WebSocket client: reconnects forever, streams state every 20 ms."""

    def __init__(self, url, joystick, get_state, send_ds, debug=False):
        self.url, self.joystick, self.get_state = url, str(joystick), get_state
        self.send_ds = send_ds
        self.wake = threading.Event()  # set on any input change -> send right away
        self.debug, self.rx, self.t_change = debug, 0, None
        self.connected = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            ws = websocket.WebSocketApp(
                self.url,
                on_open=self._on_open,
                on_message=self._on_message,  # discard sim output, but keep reading
                on_error=lambda *_: None,
                on_close=lambda *_: setattr(self, "connected", False),
            )
            # TCP_NODELAY: without it Nagle + delayed ACK can stall small sends 200ms+
            ws.run_forever(sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),), skip_utf8_validation=True)
            self.connected = False
            time.sleep(1)

    def _on_message(self):
        self.rx += 1

    def _on_open(self, ws):
        self.connected = True
        threading.Thread(target=self._send_loop, args=(ws,), daemon=True).start()

    def _send_loop(self, ws):
        last_report = time.perf_counter()
        while self.connected:
            axes, buttons, pov, enabled, mode = self.get_state()
            ds = {">new_data": True}  # joystick data is only visible after this
            if self.send_ds:
                ds.update(
                    {">ds": True, ">enabled": enabled, ">estop": False, ">autonomous": mode == "auto", ">test": mode == "test"}
                )
            try:
                ws.send(
                    json.dumps(
                        {
                            "type": "Joystick",
                            "device": self.joystick,
                            "data": {">axes": axes, ">buttons": buttons, ">povs": [pov]},
                        }
                    )
                )
                ws.send(json.dumps({"type": "DriverStation", "device": "", "data": ds}))
            except Exception:
                break
            if self.debug:
                now = time.perf_counter()
                t0, self.t_change = self.t_change, None
                if t0:
                    print(f"input -> socket: {(now - t0) * 1000:.1f} ms")
                if now - last_report >= 2:
                    print(f"sim -> GUI: {self.rx / (now - last_report):.0f} msgs/s")
                    self.rx, last_report = 0, now
            self.wake.wait(0.05)  # heartbeat every 50 ms, or immediately on change
            self.wake.clear()


class App:
    def __init__(self, root, url, joystick, send_ds=True, debug=False):
        self.root, self.debug = root, debug
        root.title(f"Sim Xbox Controller (joystick {joystick})")
        self.keys = {c[0]: c[5] for c in CONTROLS}
        self.held = {c[0]: set() for c in CONTROLS}  # who is holding each control
        self.sticks = {"L": (0.0, 0.0), "R": (0.0, 0.0)}
        self.state = ([0.0] * 6, [False] * 10, -1, False, "teleop")  # read by network thread
        self.items, self.knobs = {}, {}
        self.url = url
        self.link = Link(url, joystick, lambda: self.state, send_ds, debug)
        self.mouse_target = self.drag_stick = self.remapping = None

        c = self.canvas = tk.Canvas(root, width=640, height=450, bg="white", highlightthickness=0)
        c.pack()
        c.create_polygon(BODY, smooth=True, fill="#cfd8dc", outline="#607d8b", width=3)

        for name, label, shape, box, color, _ in CONTROLS:
            make = c.create_rectangle if shape == "rect" else c.create_oval
            tag = f"ctl_{name}"
            sid = make(*box, fill=color, outline="#37474f", width=2, tags=(tag,))
            tid = c.create_text((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, font=("Arial", 8, "bold"), tags=(tag,))
            self.items[name] = (sid, tid, color, label)
            c.tag_bind(tag, "<ButtonPress-1>", lambda e, n=name: self.mouse_down(n))
            for right_click in ("<ButtonPress-2>", "<ButtonPress-3>"):  # macOS uses 2
                c.tag_bind(tag, right_click, lambda e, n=name: self.start_remap(n))
            self.refresh_label(name)

        for side, (cx, cy) in STICK_POS.items():
            tag = f"stick_{side}"
            r = RANGE + KNOB
            c.create_oval(cx - r, cy - r, cx + r, cy + r, fill="white", outline="#37474f", width=2, tags=(tag,))
            self.knobs[side] = c.create_oval(0, 0, 0, 0, fill="#455a64", tags=(tag,))
            self.place_knob(side)
            c.tag_bind(tag, "<ButtonPress-1>", lambda e, s=side: self.stick_down(s, e))

        c.bind("<B1-Motion>", self.mouse_move)
        c.bind("<ButtonRelease-1>", self.mouse_up)
        root.bind_all("<KeyPress>", self.key_down)
        root.bind_all("<KeyRelease>", self.key_up)
        root.bind("<FocusOut>", lambda e: e.widget is root and self.release_all())

        self.enabled = tk.BooleanVar(value=False)
        self.mode = tk.StringVar(value="teleop")
        if send_ds:
            strip = tk.Frame(root)
            strip.pack(fill="x", padx=8, pady=4)
            tk.Checkbutton(
                strip,
                text="ENABLE",
                variable=self.enabled,
                fg="#c62828",
                font=("Arial", 10, "bold"),
                command=self.refresh_state,
            ).pack(side="left", padx=(0, 12))
            for text, val in (("Teleop", "teleop"), ("Auto", "auto"), ("Test", "test")):
                tk.Radiobutton(strip, text=text, value=val, variable=self.mode, command=self.refresh_state).pack(side="left")
        self.status = tk.Label(root, anchor="w")
        self.status.pack(fill="x", padx=8, pady=(0, 6))
        self.poll_status()
        if debug:
            print("Tk", root.tk.call("info", "patchlevel"), "| Python", sys.version.split()[0])
            self.lag_check(time.perf_counter() + 0.02)

    def lag_check(self, expected):
        late = (time.perf_counter() - expected) * 1000
        if late > 30:
            print(f"Tk event loop was blocked for ~{late:.0f} ms")
        self.root.after(20, self.lag_check, time.perf_counter() + 0.02)

    # ---- state --------------------------------------------------------------
    def refresh_state(self):
        def h(n):
            return bool(self.held[n])

        lx, ly = self.sticks["L"]
        rx, ry = self.sticks["R"]
        axes = [lx, ly, 1.0 if h("LT") else 0.0, 1.0 if h("RT") else 0.0, rx, ry]
        buttons = [h(n) for n in BTN_ORDER]
        pov = POV_ANGLE.get((h("Right") - h("Left"), h("Up") - h("Down")), -1)
        # single assignment: safe for the network thread
        self.state = (axes, buttons, pov, self.enabled.get(), self.mode.get())
        if self.link.t_change is None:
            self.link.t_change = time.perf_counter()
        self.link.wake.set()

    def set_held(self, name, source, down):
        (self.held[name].add if down else self.held[name].discard)(source)
        sid, _, color, _ = self.items[name]
        self.canvas.itemconfig(sid, fill=HELD_COLOR if self.held[name] else color)
        if self.debug:  # force the redraw now so we can time it
            t = time.perf_counter()
            self.canvas.update_idletasks()
            print(f"canvas redraw: {(time.perf_counter() - t) * 1000:.1f} ms")
        self.refresh_state()

    def release_all(self):
        for name in self.held:
            for src in list(self.held[name]):
                self.set_held(name, src, False)
        self.mouse_target = None
        if self.drag_stick:
            self.set_stick(self.drag_stick, 0.0, 0.0)
            self.drag_stick = None

    # ---- mouse --------------------------------------------------------------
    def mouse_down(self, name):
        self.mouse_target = name
        self.set_held(name, "mouse", True)

    def stick_down(self, side, e):
        self.drag_stick = side
        self.mouse_move(e)

    def mouse_move(self, e):
        if not self.drag_stick:
            return
        cx, cy = STICK_POS[self.drag_stick]
        dx, dy = e.x - cx, e.y - cy
        d = math.hypot(dx, dy)
        if d > RANGE:
            dx, dy = dx * RANGE / d, dy * RANGE / d
        self.set_stick(self.drag_stick, dx / RANGE, dy / RANGE)  # down = +Y, like WPILib

    def mouse_up(self):
        if self.mouse_target:
            self.set_held(self.mouse_target, "mouse", False)
            self.mouse_target = None
        if self.drag_stick:
            self.set_stick(self.drag_stick, 0.0, 0.0)
            self.drag_stick = None

    def set_stick(self, side, x, y):
        self.sticks[side] = (x, y)
        self.place_knob(side)
        self.refresh_state()

    def place_knob(self, side):
        cx, cy = STICK_POS[side]
        x, y = self.sticks[side]
        kx, ky = cx + x * RANGE, cy + y * RANGE
        self.canvas.coords(self.knobs[side], kx - KNOB, ky - KNOB, kx + KNOB, ky + KNOB)

    # ---- keyboard -----------------------------------------------------------
    def refresh_label(self, name):
        _, tid, _, label = self.items[name]
        self.canvas.itemconfig(tid, text=f"{label}\n{self.keys[name] or '-'}")

    def start_remap(self, name):
        self.remapping = name
        self.status.config(text=f"Press a key for {name} (Esc to cancel)...")

    def key_down(self, e):
        key = e.keysym.lower()
        if self.remapping:
            name, self.remapping = self.remapping, None
            if key != "escape":
                for other, k in self.keys.items():  # keep keys unique
                    if k == key and other != name:
                        self.keys[other] = ""
                        self.refresh_label(other)
                self.keys[name] = key
                self.refresh_label(name)
            return
        for name, k in self.keys.items():
            if k == key:
                self.set_held(name, "key", True)

    def key_up(self, e):
        key = e.keysym.lower()
        for name, k in self.keys.items():
            if k == key:
                self.set_held(name, "key", False)

    # ---- status line --------------------------------------------------------
    def poll_status(self):
        if not self.remapping:
            if self.link.connected:
                self.status.config(fg="#2e7d32", text=f"Connected to {self.url}")
            else:
                self.status.config(
                    fg="#c62828", text=f"Waiting for sim at {self.url} ...  " "(right-click a button to remap its key)"
                )
        self.root.after(300, self.poll_status)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://localhost:3300/wpilibws")
    ap.add_argument("--joystick", type=int, default=0, help="driver station joystick slot")
    ap.add_argument("--no-ds", action="store_true", help="don't send enable/mode (use when the Sim GUI owns robot state)")
    ap.add_argument("--debug", action="store_true", help="print input->socket latency and sim message rate")
    args = ap.parse_args()
    root = tk.Tk()
    App(root, args.url, args.joystick, send_ds=not args.no_ds, debug=args.debug)
    root.mainloop()
