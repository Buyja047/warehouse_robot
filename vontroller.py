import socket
import time
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


CMD_PORT = 4210
TLM_PORT = 4211


def now_s():
    return time.time()


class UdpRobotGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Warehouse Robot Controller (UDP)")

        # UDP sockets
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tlm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tlm.bind(("0.0.0.0", TLM_PORT))
        self.sock_tlm.setblocking(False)

        self.esp_ip = tk.StringVar(value="192.168.1.50")
        self.connected = False

        # manual settings
        self.speed = tk.IntVar(value=140)     # base PWM
        self.turn = tk.IntVar(value=90)       # differential PWM

        # UI toggles
        self.plot_enabled = tk.BooleanVar(value=True)

        # telemetry buffers
        self.max_points = 300
        self.t0 = now_s()
        self.ts = deque(maxlen=self.max_points)

        self.tL = deque(maxlen=self.max_points)
        self.mL = deque(maxlen=self.max_points)
        self.pL = deque(maxlen=self.max_points)

        self.tR = deque(maxlen=self.max_points)
        self.mR = deque(maxlen=self.max_points)
        self.pR = deque(maxlen=self.max_points)

        # latest status
        self.latest = {
            "pat": 0, "marker": 0, "state": 0,
            "encL": 0, "encR": 0,
            "errL": 0.0, "errR": 0.0,
            "iL": 0.0, "iR": 0.0,
        }

        # build UI
        self._build_ui()
        self._build_plot()

        # key bindings (arrow control)
        self.root.bind("<Up>",    lambda e: self._key_drive("up"))
        self.root.bind("<Down>",  lambda e: self._key_drive("down"))
        self.root.bind("<Left>",  lambda e: self._key_drive("left"))
        self.root.bind("<Right>", lambda e: self._key_drive("right"))
        self.root.bind("<space>", lambda e: self.send_cmd("STOP"))

        # update loops
        self._ui_update()
        self._net_poll()

    # ---------------- UI ----------------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        # Connection row
        conn = ttk.LabelFrame(top, text="Connection", padding=8)
        conn.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(conn, text="ESP32 IP:").pack(side=tk.LEFT)
        ttk.Entry(conn, textvariable=self.esp_ip, width=16).pack(side=tk.LEFT, padx=(6, 10))

        ttk.Button(conn, text="Connect", command=self.connect).pack(side=tk.LEFT)
        ttk.Button(conn, text="STOP", command=lambda: self.send_cmd("STOP")).pack(side=tk.LEFT, padx=6)
        ttk.Button(conn, text="AUTO", command=lambda: self.send_cmd("AUTO")).pack(side=tk.LEFT)
        ttk.Button(conn, text="FORCE", command=lambda: self.send_cmd("FORCE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(conn, text="RESET", command=lambda: self.send_cmd("RESET")).pack(side=tk.LEFT)

        ttk.Checkbutton(conn, text="Plot", variable=self.plot_enabled).pack(side=tk.RIGHT)

        # Manual control
        man = ttk.LabelFrame(top, text="Manual Drive (Arrow keys or buttons)", padding=8)
        man.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        srow = ttk.Frame(man)
        srow.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(srow, text="Speed (PWM)").pack(side=tk.LEFT)
        ttk.Scale(srow, from_=0, to=255, orient="horizontal", variable=self.speed).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.lbl_speed = ttk.Label(srow, text="140")
        self.lbl_speed.pack(side=tk.LEFT, padx=(6, 0))

        trow = ttk.Frame(man)
        trow.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        ttk.Label(trow, text="Turn (diff)").pack(side=tk.LEFT)
        ttk.Scale(trow, from_=0, to=255, orient="horizontal", variable=self.turn).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
        self.lbl_turn = ttk.Label(trow, text="90")
        self.lbl_turn.pack(side=tk.LEFT, padx=(6, 0))

        brow = ttk.Frame(man)
        brow.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        ttk.Button(brow, text="↑", width=6, command=lambda: self._key_drive("up")).pack(side=tk.LEFT, padx=3)
        ttk.Button(brow, text="↓", width=6, command=lambda: self._key_drive("down")).pack(side=tk.LEFT, padx=3)
        ttk.Button(brow, text="←", width=6, command=lambda: self._key_drive("left")).pack(side=tk.LEFT, padx=3)
        ttk.Button(brow, text="→", width=6, command=lambda: self._key_drive("right")).pack(side=tk.LEFT, padx=3)
        ttk.Button(brow, text="0 (Coast)", command=lambda: self.send_cmd("MAN 0 0")).pack(side=tk.LEFT, padx=10)

        # Parameter tuning
        prm = ttk.LabelFrame(top, text="Runtime Tuning (SET ...)", padding=8)
        prm.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        grid = ttk.Frame(prm)
        grid.pack(side=tk.TOP, fill=tk.X)

        self.param_vars = {
            "kp": tk.StringVar(value="0.30"),
            "ki": tk.StringVar(value="0.08"),
            "ff": tk.StringVar(value="0.35"),
            "base": tk.StringVar(value="450"),
            "steer": tk.StringVar(value="220"),
            "noline": tk.StringVar(value="420"),
            "marker_ms": tk.StringVar(value="80"),
            "stable": tk.StringVar(value="2"),
        }

        fields = [("kp", "Kp_spd"), ("ki", "Ki_spd"), ("ff", "FF_GAIN"),
                  ("base", "BASE_CPS"), ("steer", "STEER_CPS"), ("noline", "NO_LINE_CPS"),
                  ("marker_ms", "MARKER_CONFIRM_MS"), ("stable", "PATTERN_STABLE_N")]

        for i, (key, label) in enumerate(fields):
            r = i // 4
            c = (i % 4) * 2
            ttk.Label(grid, text=label).grid(row=r, column=c, sticky="w", padx=(0, 6), pady=2)
            ttk.Entry(grid, textvariable=self.param_vars[key], width=10).grid(row=r, column=c+1, sticky="w", padx=(0, 16), pady=2)

        btnrow = ttk.Frame(prm)
        btnrow.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        ttk.Button(btnrow, text="Apply All", command=self.apply_all_params).pack(side=tk.LEFT)
        ttk.Button(btnrow, text="GET Params", command=lambda: self.send_cmd("GET")).pack(side=tk.LEFT, padx=8)

        # Status
        st = ttk.LabelFrame(top, text="Status", padding=8)
        st.pack(side=tk.TOP, fill=tk.X, pady=(8, 0))

        self.lbl_status = ttk.Label(st, text="Disconnected")
        self.lbl_status.pack(side=tk.LEFT)

    def _build_plot(self):
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        fig = Figure(figsize=(9, 4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Speed Control (target vs measured)")
        self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel("counts/sec")

        # Lines (no explicit colors requested; matplotlib defaults are fine)
        (self.line_tL,) = self.ax.plot([], [], label="tL")
        (self.line_mL,) = self.ax.plot([], [], label="mL")
        (self.line_tR,) = self.ax.plot([], [], label="tR")
        (self.line_mR,) = self.ax.plot([], [], label="mR")

        self.ax.legend(loc="upper right")
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(fig, master=frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    # ---------------- Networking ----------------
    def connect(self):
        ip = self.esp_ip.get().strip()
        try:
            socket.inet_aton(ip)
        except OSError:
            messagebox.showerror("IP error", "Invalid ESP32 IP address")
            return

        self.connected = True
        # Any UDP command makes ESP remember us for telemetry
        self.send_cmd("STOP")
        self.send_cmd("GET")
        self.lbl_status.configure(text=f"Connected (commanding {ip}:{CMD_PORT}, listening TLM :{TLM_PORT})")

    def send_cmd(self, cmd: str):
        if not self.connected:
            return
        ip = self.esp_ip.get().strip()
        try:
            self.sock_cmd.sendto(cmd.encode("utf-8"), (ip, CMD_PORT))
        except OSError:
            pass

    def _key_drive(self, direction: str):
        sp = int(self.speed.get())
        tr = int(self.turn.get())

        if direction == "up":
            L, R = sp, sp
        elif direction == "down":
            L, R = -sp, -sp
        elif direction == "left":
            L, R = sp - tr, sp + tr
        elif direction == "right":
            L, R = sp + tr, sp - tr
        else:
            return

        L = max(-255, min(255, L))
        R = max(-255, min(255, R))
        self.send_cmd(f"MAN {L} {R}")

    def apply_all_params(self):
        # Sends a batch of SET commands
        for key, var in self.param_vars.items():
            val = var.get().strip()
            if val == "":
                continue
            # Basic validation (float/int strings accepted)
            try:
                float(val)
            except ValueError:
                messagebox.showerror("Param error", f"Invalid value for {key}: {val}")
                return
            self.send_cmd(f"SET {key} {val}")

        self.send_cmd("GET")

    def _poll_telemetry(self):
        # Non-blocking read of all pending packets
        got_any = False
        while True:
            try:
                data, _ = self.sock_tlm.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break

            line = data.decode(errors="ignore").strip()
            if not line:
                continue

            # CSV: tL,mL,pwmL,tR,mR,pwmR,errL,errR,iL,iR,pat,marker,state,encL,encR
            parts = line.split(",")
            if len(parts) < 15:
                continue

            try:
                tL = float(parts[0]); mL = float(parts[1]); pL = int(parts[2])
                tR = float(parts[3]); mR = float(parts[4]); pR = int(parts[5])
                errL = float(parts[6]); errR = float(parts[7])
                iL = float(parts[8]); iR = float(parts[9])
                pat = int(parts[10]); marker = int(parts[11]); state = int(parts[12])
                encL = int(parts[13]); encR = int(parts[14])
            except ValueError:
                continue

            t = now_s() - self.t0
            self.ts.append(t)

            self.tL.append(tL); self.mL.append(mL); self.pL.append(pL)
            self.tR.append(tR); self.mR.append(mR); self.pR.append(pR)

            self.latest.update({
                "pat": pat, "marker": marker, "state": state,
                "encL": encL, "encR": encR,
                "errL": errL, "errR": errR,
                "iL": iL, "iR": iR,
            })

            got_any = True

        return got_any

    # ---------------- Update loops ----------------
    def _ui_update(self):
        # Update slider labels + status text
        self.lbl_speed.configure(text=str(int(self.speed.get())))
        self.lbl_turn.configure(text=str(int(self.turn.get())))

        if self.connected:
            st = self.latest
            self.lbl_status.configure(
                text=(f"pat={st['pat']} marker={st['marker']} state={st['state']}  "
                      f"encL={st['encL']} encR={st['encR']}  "
                      f"errL={st['errL']:.1f} errR={st['errR']:.1f}  "
                      f"iL={st['iL']:.1f} iR={st['iR']:.1f}")
            )

        self.root.after(100, self._ui_update)

    def _net_poll(self):
        got = self._poll_telemetry()

        if self.plot_enabled.get() and got and len(self.ts) > 2:
            xs = list(self.ts)

            self.line_tL.set_data(xs, list(self.tL))
            self.line_mL.set_data(xs, list(self.mL))
            self.line_tR.set_data(xs, list(self.tR))
            self.line_mR.set_data(xs, list(self.mR))

            # Autoscale view
            self.ax.relim()
            self.ax.autoscale_view()

            self.canvas.draw_idle()

        self.root.after(50, self._net_poll)


if __name__ == "__main__":
    root = tk.Tk()
    app = UdpRobotGui(root)
    root.mainloop()
