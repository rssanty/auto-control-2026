import json
import math
import os
import queue
import re
import select
import socket
import subprocess
import sys
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    tk = None
    ttk = None

try:
    import serial
    from serial.tools import list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    list_ports = None
    HAS_SERIAL = False


# ======================================================================================
# Protocolo
# ======================================================================================

LABELS = {
    "F": "Adelante", "B": "Atrás", "L": "Izquierda", "R": "Derecha",
    "G": "Adelante + izquierda", "I": "Adelante + derecha",
    "H": "Atrás + izquierda", "J": "Atrás + derecha", "S": "Detenido",
}

WHEELS = {
    "F": (1.0, 1.0), "B": (-1.0, -1.0), "L": (-1.0, 1.0), "R": (1.0, -1.0),
    "G": (0.4, 1.0), "I": (1.0, 0.4), "H": (-0.4, -1.0), "J": (-1.0, -0.4),
    "S": (0.0, 0.0),
}

ARROW_ANGLE = {"F": 0, "I": 45, "R": 90, "J": 135, "B": 180, "H": 225, "L": 270, "G": 315}

KEYMAP = {
    "w": "F", "up": "F", "s": "B", "down": "B",
    "a": "L", "left": "L", "d": "R", "right": "R",
}


def command_of(fb, lr):
    """fb: 1 adelante, -1 atrás.  lr: 1 derecha, -1 izquierda."""
    if fb > 0 and lr < 0:
        return "G"
    if fb > 0 and lr > 0:
        return "I"
    if fb < 0 and lr < 0:
        return "H"
    if fb < 0 and lr > 0:
        return "J"
    if fb > 0:
        return "F"
    if fb < 0:
        return "B"
    if lr < 0:
        return "L"
    if lr > 0:
        return "R"
    return "S"


# ======================================================================================
# Robot virtual: la misma lógica del sketch de Arduino
# ======================================================================================

class SimRobot:
    FACTOR_GIRO = 0.35
    TIMEOUT = 1.0          # segundos sin comandos de movimiento -> se detiene

    def __init__(self, invert_avance=True, invert_pivote=False):
        self.invert_avance = invert_avance
        self.invert_pivote = invert_pivote
        self.speed = 255
        self.acel = 0
        self.giro = 0
        self.left = 0
        self.right = 0
        self.moving = False
        self.last = 0.0
        self.reading = False
        self.vt = 0
        self.vd = 0

    def feed(self, text, now):
        for ch in text:
            self._char(ch, now)

    def _char(self, c, now):
        if "a" <= c <= "z":
            c = c.upper()

        if self.reading:
            if "0" <= c <= "9" and self.vd < 3:
                self.vt = self.vt * 10 + (ord(c) - 48)
                self.vd += 1
                return
            self.speed = max(0, min(255, self.vt))
            self.reading = False
        if c == "V":
            self.reading = True
            self.vt = 0
            self.vd = 0
            return

        movement = True
        if c == "F":
            self.acel, self.giro = 1, 0
        elif c == "B":
            self.acel, self.giro = -1, 0
        elif c == "L":
            self.acel, self.giro = 0, -1
        elif c == "R":
            self.acel, self.giro = 0, 1
        elif c == "G":
            self.acel, self.giro = 1, -1
        elif c == "I":
            self.acel, self.giro = 1, 1
        elif c == "H":
            self.acel, self.giro = -1, -1
        elif c == "J":
            self.acel, self.giro = -1, 1
        elif c == "S":
            self.acel, self.giro = 0, 0
        elif c == "X":
            self.giro = 0
        elif c == "Y":
            self.acel = 0
        elif c in "123":
            self.speed = {"1": 100, "2": 150, "3": 200}[c]
            movement = False
        else:
            movement = False

        if movement:
            self.last = now
        self._apply()

    def _apply(self):
        if self.acel == 0 and self.giro == 0:
            self._stop()
            return
        if self.acel == 0:
            izq = self.speed if self.giro > 0 else -self.speed
            der = -izq
            if self.invert_pivote:
                izq, der = -izq, -der
        else:
            base = (-self.acel if self.invert_avance else self.acel) * self.speed
            reducida = int(base * self.FACTOR_GIRO)
            izq = reducida if self.giro < 0 else base
            der = reducida if self.giro > 0 else base
        self.left, self.right = izq, der
        self.moving = True

    def _stop(self):
        self.left = self.right = 0
        self.acel = self.giro = 0
        self.moving = False

    def tick(self, now):
        """Devuelve True si el freno de seguridad acaba de actuar."""
        if self.moving and now - self.last > self.TIMEOUT:
            self._stop()
            return True
        return False


# ======================================================================================
# Enlaces (simulador o puerto serie)
# ======================================================================================

class SimLink:
    name = "Simulador"

    def __init__(self, sim):
        self.sim = sim
        self.alive = True

    def write(self, text):
        self.sim.feed(text, time.monotonic())

    def close(self):
        self.alive = False


class SerialLink:
    def __init__(self, port, baud, q):
        self.port = port
        self.baud = baud
        self.q = q
        self.ser = None
        self.alive = False
        self.name = port

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2, write_timeout=1)
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.alive:
            try:
                data = self.ser.read(64)
            except Exception:
                break
            if data:
                self.q.put(("rx", data))
        if self.alive:
            self.alive = False
            self.q.put(("lost", self))

    def write(self, text):
        try:
            self.ser.write(text.encode("ascii", "replace"))
        except Exception:
            if self.alive:
                self.alive = False
                self.q.put(("lost", self))
            raise

    def close(self):
        self.alive = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


MAC_RE = re.compile(r"((?:[0-9A-Fa-f]{2}[:-]){5}[0-9A-Fa-f]{2})")


class BtLink:
    """Bluetooth clásico directo (RFCOMM / SPP), sin puerto COM."""

    def __init__(self, mac, channel, q):
        self.mac = mac.replace("-", ":").upper()
        self.channel = channel
        self.q = q
        self.sock = None
        self.alive = False
        self.name = self.mac

    def open(self):
        if not hasattr(socket, "AF_BLUETOOTH"):
            raise OSError("este Python no soporta Bluetooth directo; usa el modo Puerto COM")
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.settimeout(12)
        try:
            s.connect((self.mac, self.channel))
        except Exception:
            s.close()
            raise
        s.settimeout(None)
        self.sock = s
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.alive:
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.3)
                if ready:
                    data = self.sock.recv(256)
                    if not data:
                        break
                    self.q.put(("rx", data))
            except Exception:
                break
        if self.alive:
            self.alive = False
            self.q.put(("lost", self))

    def write(self, text):
        try:
            self.sock.sendall(text.encode("ascii", "replace"))
        except Exception:
            if self.alive:
                self.alive = False
                self.q.put(("lost", self))
            raise

    def close(self):
        self.alive = False
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass


def parse_pnp(text):
    out = []
    try:
        data = json.loads(text)
    except Exception:
        return out
    if isinstance(data, dict):
        data = [data]
    for d in data or []:
        m = re.search(r"DEV_([0-9A-Fa-f]{12})", str(d.get("InstanceId", "")))
        if m:
            h = m.group(1).upper()
            out.append((d.get("FriendlyName") or "?", ":".join(h[i:i + 2] for i in range(0, 12, 2))))
    return out


def parse_bluetoothctl(text):
    out = []
    for line in text.splitlines():
        m = re.match(r"\s*Device\s+((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})\s+(.*)", line)
        if m:
            out.append((m.group(2).strip(), m.group(1).upper()))
    return out


def list_paired_bt():
    """Dispositivos Bluetooth emparejados: [(nombre, MAC)]."""
    found = []
    try:
        if os.name == "nt":
            cmd = ["powershell", "-NoProfile", "-Command",
                   "Get-PnpDevice -Class Bluetooth | Select-Object FriendlyName,InstanceId | ConvertTo-Json -Compress"]
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=15,
                               creationflags=0x08000000)
            found = parse_pnp(r.stdout)
        else:
            r = subprocess.run(["bluetoothctl", "devices", "Paired"], capture_output=True, text=True, timeout=10)
            found = parse_bluetoothctl(r.stdout)
            if not found:
                r = subprocess.run(["bluetoothctl", "devices"], capture_output=True, text=True, timeout=10)
                found = parse_bluetoothctl(r.stdout)
    except Exception:
        pass
    seen, out = set(), []
    for name, mac in found:
        if mac not in seen:
            seen.add(mac)
            out.append((name, mac))
    return out


class PygameBackend:
    """Lee el mando con pygame (SDL): Xbox, PlayStation y genéricos. Sirve por Bluetooth o USB."""

    def __init__(self):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        self.pg = pygame
        pygame.display.init()
        pygame.joystick.init()
        try:
            from pygame._sdl2 import controller as sdlc
            sdlc.init()
            self.sdlc = sdlc
        except Exception:
            self.sdlc = None
        self.dev = None
        self.kind = None
        self.sig = None

    def _signature(self):
        return (self.sdlc.get_count() if self.sdlc else 0, self.pg.joystick.get_count())

    def scan(self):
        self.pg.event.pump()
        sig = self._signature()
        if sig == self.sig and self.dev is not None:
            return self.name
        self.sig = sig
        self.dev = None
        self.name = None
        if self.sdlc:
            for i in range(self.sdlc.get_count()):
                try:
                    if self.sdlc.is_controller(i):
                        self.dev = self.sdlc.Controller(i)
                        self.kind = "ctrl"
                        self.name = getattr(self.dev, "name", None) or f"Mando {i}"
                        return self.name
                except Exception:
                    continue
        if self.pg.joystick.get_count() > 0:
            j = self.pg.joystick.Joystick(0)
            j.init()
            self.dev = j
            self.kind = "joy"
            self.name = j.get_name()
            return self.name
        return None

    def read(self):
        self.pg.event.pump()
        d = self.dev
        if self.kind == "ctrl":
            def b(i):
                return bool(d.get_button(i))
            return {
                "x": d.get_axis(0) / 32767.0, "y": d.get_axis(1) / 32767.0,
                "dx": int(b(14)) - int(b(13)), "dy": int(b(11)) - int(b(12)),
                "rt": max(0.0, d.get_axis(5) / 32767.0),
                "l1": b(9), "r1": b(10),
            }
        hx, hy = d.get_hat(0) if d.get_numhats() > 0 else (0, 0)
        nb, na = d.get_numbuttons(), d.get_numaxes()
        return {
            "x": d.get_axis(0) if na > 0 else 0.0, "y": d.get_axis(1) if na > 1 else 0.0,
            "dx": hx, "dy": hy,
            "rt": (d.get_axis(5) + 1) / 2 if na > 5 else 0.0,
            "l1": bool(d.get_button(4)) if nb > 4 else False,
            "r1": bool(d.get_button(5)) if nb > 5 else False,
        }


# ======================================================================================
# Configuración guardada
# ======================================================================================

CFG_PATH = os.path.join(os.path.expanduser("~"), ".robot_test_app.json")
DEFAULT_CFG = {
    "mode": "bt", "port": "", "baud": "9600", "speed": 150,
    "invert_lr": False, "invert_fb": False, "auto_reconnect": True,
    "keepalive": True, "keepalive_ms": 250,
    "btdev": "", "chan": 1, "use_pad": True, "pad_trig": True,
}


def load_cfg():
    cfg = dict(DEFAULT_CFG)
    try:
        with open(CFG_PATH, encoding="utf-8") as f:
            cfg.update(json.load(f))
    except Exception:
        pass
    return cfg


def save_cfg(cfg):
    try:
        with open(CFG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


# ======================================================================================
# Interfaz
# ======================================================================================

BG = "#ECEFF1"
OK = "#2E7D32"
BAD = "#C62828"
WARN = "#EF6C00"


def round_rect(c, x1, y1, x2, y2, r, **kw):
    pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
           x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
    return c.create_polygon(pts, smooth=True, **kw)


class App(tk.Tk if tk else object):

    def __init__(self):
        super().__init__()
        self.title("Robot Control - App de prueba")
        self.minsize(1150, 780)
        self.configure(bg=BG)

        self.cfg = load_cfg()
        self.q = queue.Queue()
        self.link = None
        self.sim = None
        self.connecting = False
        self.want_connected = False
        self.reconnect_job = None

        self.keys = set()
        self.release_jobs = {}
        self.mouse = {"F": False, "B": False, "L": False, "R": False}
        self.ui_cmd = "S"
        self.last_sent = "S"
        self.keep_job = None
        self.repeats = 0
        self.speed = int(self.cfg["speed"])
        self.speed_job = None
        self.seq_job = None
        self.phase = 0.0
        self._last_draw = time.monotonic()
        self.port_map = {}
        self.pad = None
        self.pad_name = None
        self.pad_fb = 0
        self.pad_lr = 0
        self.pad_l1 = False
        self.pad_r1 = False
        self.pad_next = 0.0

        self.v_mode = tk.StringVar(value=self.cfg["mode"])
        self.v_port = tk.StringVar(value=self.cfg["port"])
        self.v_btdev = tk.StringVar(value=self.cfg["btdev"])
        self.v_chan = tk.IntVar(value=int(self.cfg["chan"]))
        self.v_pad = tk.BooleanVar(value=self.cfg["use_pad"])
        self.v_pad_trig = tk.BooleanVar(value=self.cfg["pad_trig"])
        self.v_baud = tk.StringVar(value=self.cfg["baud"])
        self.v_auto = tk.BooleanVar(value=self.cfg["auto_reconnect"])
        self.v_inv_lr = tk.BooleanVar(value=self.cfg["invert_lr"])
        self.v_inv_fb = tk.BooleanVar(value=self.cfg["invert_fb"])
        self.v_keep = tk.BooleanVar(value=self.cfg["keepalive"])
        self.v_keep_ms = tk.IntVar(value=int(self.cfg["keepalive_ms"]))
        self.v_show_rep = tk.BooleanVar(value=False)
        self.v_block = tk.BooleanVar(value=False)
        self.v_sim_inv_av = tk.BooleanVar(value=True)
        self.v_sim_inv_piv = tk.BooleanVar(value=False)
        self.v_manual = tk.StringVar()

        self._style()
        self._build()
        self._bind_keys()
        self._layout_conn()
        self.refresh_ports()
        self._init_pad()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self._log("SYS", "App lista. Teclado: W/A/S/D o flechas (combina varias). Espacio = parar.")
        if self.v_mode.get() == "sim":
            self.connect()
        self.after(33, self._loop)

    # ---------------------------------------------------------------- construcción

    def _style(self):
        st = ttk.Style(self)
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure("TFrame", background=BG)
        st.configure("TLabelframe", background=BG)
        st.configure("TLabelframe.Label", background=BG, font=("TkDefaultFont", 10, "bold"))
        st.configure("TLabel", background=BG)
        st.configure("TCheckbutton", background=BG)
        st.configure("TRadiobutton", background=BG)
        st.configure("Big.TButton", font=("TkDefaultFont", 20), padding=(0, 10))
        st.configure("Stop.TButton", font=("TkDefaultFont", 11, "bold"))

    def _build(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=8)
        left.grid(row=0, column=0, sticky="ns")
        center = ttk.Frame(self, padding=8)
        center.grid(row=0, column=1, sticky="nsew")
        right = ttk.Frame(self, padding=8)
        right.grid(row=0, column=2, sticky="ns")

        self._build_left(left)
        self._build_center(center)
        self._build_right(right)

    def _build_left(self, p):
        f = ttk.LabelFrame(p, text="Robot (conexión)", padding=8)
        f.pack(fill="x")
        for r_, (txt, val) in enumerate((("Bluetooth directo (recomendado)", "bt"),
                                         ("Puerto COM / serie", "serial"),
                                         ("Simulador (sin hardware)", "sim"))):
            ttk.Radiobutton(f, text=txt, value=val, variable=self.v_mode,
                            command=self._mode_changed).grid(row=r_, column=0, columnspan=3, sticky="w")
        self.lbl_dev = ttk.Label(f, text="Robot")
        self.lbl_dev.grid(row=3, column=0, sticky="w", pady=(6, 0))
        self.cb_port = ttk.Combobox(f, width=26, textvariable=self.v_btdev)
        self.cb_port.grid(row=3, column=1, pady=(6, 0), padx=4)
        self.btn_refresh = ttk.Button(f, text="↻", width=3, command=self.refresh_ports)
        self.btn_refresh.grid(row=3, column=2, pady=(6, 0))
        self.lbl_extra = ttk.Label(f, text="Canal")
        self.lbl_extra.grid(row=4, column=0, sticky="w")
        self.sp_chan = ttk.Spinbox(f, from_=1, to=30, width=6, textvariable=self.v_chan)
        self.sp_chan.grid(row=4, column=1, sticky="w", padx=4, pady=2)
        self.cb_baud = ttk.Combobox(f, textvariable=self.v_baud, width=10,
                                    values=["9600", "19200", "38400", "57600", "115200"])
        self.cb_baud.grid(row=4, column=1, sticky="w", padx=4, pady=2)
        self.btn_conn = ttk.Button(f, text="Conectar", command=self.toggle_connect)
        self.btn_conn.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(6, 2))
        ttk.Checkbutton(f, text="Reconectar automáticamente", variable=self.v_auto).grid(
            row=6, column=0, columnspan=3, sticky="w")
        sf = ttk.Frame(f)
        sf.grid(row=7, column=0, columnspan=3, sticky="w", pady=(4, 0))
        self.lbl_dot = tk.Label(sf, text="●", fg=BAD, bg=BG, font=("TkDefaultFont", 14))
        self.lbl_dot.pack(side="left")
        self.lbl_status = ttk.Label(sf, text="Sin conexión")
        self.lbl_status.pack(side="left", padx=4)
        ttk.Button(f, text="Abrir ajustes Bluetooth (emparejar)", command=self.open_bt_settings).grid(
            row=8, column=0, columnspan=3, sticky="ew", pady=(4, 0))

        f = ttk.LabelFrame(p, text="Control (mantén apretado)", padding=8)
        f.pack(fill="x", pady=(8, 0))
        turn = ttk.Frame(f)
        turn.grid(row=0, column=0, padx=(0, 14))
        drive = ttk.Frame(f)
        drive.grid(row=0, column=1)
        for d, txt, parent, r, c in (("L", "◀", turn, 0, 0), ("R", "▶", turn, 0, 1),
                                     ("F", "▲", drive, 0, 0), ("B", "▼", drive, 1, 0)):
            b = ttk.Button(parent, text=txt, style="Big.TButton", width=3)
            b.grid(row=r, column=c, padx=2, pady=2)
            b.bind("<ButtonPress-1>", lambda e, d=d: self._mouse(d, True))
            b.bind("<ButtonRelease-1>", lambda e, d=d: self._mouse(d, False))
        ttk.Label(f, text="Teclado: A/D o ◀▶ giran · W/S o ▲▼ avanzan\n"
                          "Combina teclas (ej. W + A = curva). Espacio = parar.",
                  justify="left").grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))

        f = ttk.LabelFrame(p, text="Velocidad", padding=8)
        f.pack(fill="x", pady=(8, 0))
        self.lbl_speed = ttk.Label(f, text="")
        self.lbl_speed.pack(anchor="w")
        self.scale = ttk.Scale(f, from_=0, to=255, orient="horizontal", length=260)
        self.scale.set(self.speed)
        self.scale.configure(command=self._on_speed)
        self.scale.pack(fill="x")
        pf = ttk.Frame(f)
        pf.pack(anchor="w", pady=(4, 0))
        for txt, v in (("Lento", 100), ("Medio", 150), ("Rápido", 200), ("Máx", 255)):
            ttk.Button(pf, text=txt, width=7, command=lambda v=v: self._set_speed(v)).pack(side="left", padx=2)
        self._update_speed_label()

        f = ttk.LabelFrame(p, text="Opciones", padding=8)
        f.pack(fill="x", pady=(8, 0))
        ttk.Checkbutton(f, text="Invertir ◀ ▶", variable=self.v_inv_lr, command=self._invert_changed).pack(anchor="w")
        ttk.Checkbutton(f, text="Invertir ▲ ▼", variable=self.v_inv_fb, command=self._invert_changed).pack(anchor="w")
        kf = ttk.Frame(f)
        kf.pack(anchor="w", pady=(4, 0))
        ttk.Checkbutton(kf, text="Reenviar cada", variable=self.v_keep, command=self._keep_changed).pack(side="left")
        ttk.Spinbox(kf, from_=50, to=2000, increment=50, width=5, textvariable=self.v_keep_ms,
                    command=self._keep_changed).pack(side="left", padx=3)
        ttk.Label(kf, text="ms").pack(side="left")

    def _build_center(self, p):
        p.rowconfigure(0, weight=1)
        p.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(p, bg="#FFFFFF", highlightthickness=1, highlightbackground="#B0BEC5")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.lbl_action = ttk.Label(p, text="Detenido", font=("TkDefaultFont", 22, "bold"), anchor="center")
        self.lbl_action.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        self.lbl_info = ttk.Label(p, text="", anchor="center")
        self.lbl_info.grid(row=2, column=0, sticky="ew")

        f = ttk.LabelFrame(p, text="Mando (Bluetooth)", padding=8)
        f.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        self.lbl_pad = ttk.Label(f, text="Mando: buscando…", font=("TkDefaultFont", 10, "bold"))
        self.lbl_pad.grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Checkbutton(f, text="Usar mando", variable=self.v_pad, command=self._pad_toggle).grid(
            row=1, column=0, sticky="w")
        ttk.Checkbutton(f, text="Gatillo R2 = velocidad", variable=self.v_pad_trig).grid(
            row=1, column=1, sticky="w", padx=10)
        ttk.Button(f, text="Buscar mando", command=self._scan_pad_now).grid(row=1, column=2, padx=4)
        ttk.Button(f, text="Ajustes Bluetooth", command=self.open_bt_settings).grid(row=1, column=3)
        ttk.Label(f, text="Cruceta o palanca izquierda: mover (se combinan)  ·  L1 / R1: velocidad − / +",
                  justify="left").grid(row=2, column=0, columnspan=4, sticky="w", pady=(4, 0))

    def _build_right(self, p):
        f = ttk.LabelFrame(p, text="Robot virtual (simulador)", padding=8)
        f.pack(fill="x")
        self.bar_l = ttk.Progressbar(f, maximum=510, length=240)
        self.bar_r = ttk.Progressbar(f, maximum=510, length=240)
        self.lbl_ml = ttk.Label(f, text="Motor izq: 0")
        self.lbl_mr = ttk.Label(f, text="Motor der: 0")
        self.lbl_ml.pack(anchor="w")
        self.bar_l.pack(fill="x")
        self.lbl_mr.pack(anchor="w", pady=(4, 0))
        self.bar_r.pack(fill="x")
        self.lbl_sim = ttk.Label(f, text="", justify="left")
        self.lbl_sim.pack(anchor="w", pady=(6, 0))
        ttk.Checkbutton(f, text="INVERTIR_AVANCE", variable=self.v_sim_inv_av,
                        command=self._sim_opts).pack(anchor="w")
        ttk.Checkbutton(f, text="INVERTIR_PIVOTE", variable=self.v_sim_inv_piv,
                        command=self._sim_opts).pack(anchor="w")

        f = ttk.LabelFrame(p, text="Pruebas", padding=8)
        f.pack(fill="x", pady=(8, 0))
        self.btn_seq = ttk.Button(f, text="▶ Secuencia de prueba", command=self.toggle_sequence)
        self.btn_seq.pack(fill="x")
        ttk.Button(f, text="■ PARAR (S)", style="Stop.TButton", command=self.stop_all).pack(fill="x", pady=(4, 0))
        ttk.Checkbutton(f, text="Bloquear envío (probar freno de seguridad)",
                        variable=self.v_block).pack(anchor="w", pady=(6, 0))
        mf = ttk.Frame(f)
        mf.pack(fill="x", pady=(6, 0))
        e = ttk.Entry(mf, textvariable=self.v_manual, width=16)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", lambda ev: self.send_manual())
        ttk.Button(mf, text="Enviar", command=self.send_manual).pack(side="left", padx=(4, 0))

        f = ttk.LabelFrame(p, text="Registro", padding=8)
        f.pack(fill="both", expand=True, pady=(8, 0))
        tf = ttk.Frame(f)
        tf.pack(fill="both", expand=True)
        self.txt = tk.Text(tf, width=36, height=14, state="disabled", wrap="word", font=("Courier", 9))
        sb = ttk.Scrollbar(tf, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.txt.tag_configure("TX", foreground="#1565C0")
        self.txt.tag_configure("RX", foreground="#2E7D32")
        self.txt.tag_configure("SYS", foreground="#6D4C41")
        self.txt.tag_configure("ERR", foreground=BAD)
        bf = ttk.Frame(f)
        bf.pack(fill="x", pady=(4, 0))
        ttk.Checkbutton(bf, text="Mostrar reenvíos", variable=self.v_show_rep).pack(side="left")
        ttk.Button(bf, text="Limpiar", command=self._clear_log).pack(side="right")

    # ---------------------------------------------------------------- teclado y mouse

    def _bind_keys(self):
        self.bind_all("<KeyPress>", self._key_press)
        self.bind_all("<KeyRelease>", self._key_release)

    def _typing(self):
        w = self.focus_get()
        return isinstance(w, (tk.Entry, ttk.Entry, ttk.Combobox, ttk.Spinbox))

    def _key_press(self, e):
        k = e.keysym.lower()
        if k == "space" and not self._typing():
            self.stop_all()
            return
        if k not in KEYMAP or self._typing():
            return
        job = self.release_jobs.pop(k, None)
        if job:
            self.after_cancel(job)
        if k not in self.keys:
            self.keys.add(k)
            self.refresh()

    def _key_release(self, e):
        k = e.keysym.lower()
        if k not in KEYMAP:
            return
        old = self.release_jobs.pop(k, None)
        if old:
            self.after_cancel(old)
        self.release_jobs[k] = self.after(30, lambda: self._do_release(k))

    def _do_release(self, k):
        self.release_jobs.pop(k, None)
        if k in self.keys:
            self.keys.discard(k)
            self.refresh()

    def _mouse(self, d, down):
        self.mouse[d] = down
        self.refresh()

    def stop_all(self):
        self.keys.clear()
        for j in self.release_jobs.values():
            self.after_cancel(j)
        self.release_jobs.clear()
        for d in self.mouse:
            self.mouse[d] = False
        self.stop_sequence()
        self.apply_move(0, 0)
        self.send("S")

    # ---------------------------------------------------------------- movimiento

    def _dir_active(self, d):
        pad = {"F": self.pad_fb > 0, "B": self.pad_fb < 0, "R": self.pad_lr > 0, "L": self.pad_lr < 0}[d]
        return self.mouse[d] or pad or any(KEYMAP[k] == d for k in self.keys)

    def refresh(self):
        if self.seq_job:
            return
        fb = int(self._dir_active("F")) - int(self._dir_active("B"))
        lr = int(self._dir_active("R")) - int(self._dir_active("L"))
        self.apply_move(fb, lr)

    def apply_move(self, fb, lr):
        ui = command_of(fb, lr)
        if ui == self.ui_cmd:
            return
        self.ui_cmd = ui
        sent = command_of(-fb if self.v_inv_fb.get() else fb, -lr if self.v_inv_lr.get() else lr)
        self.last_sent = sent
        self.send(sent)
        self._schedule_keepalive()
        self.lbl_action.configure(text=LABELS[ui])

    def _schedule_keepalive(self):
        if self.keep_job:
            self.after_cancel(self.keep_job)
            self.keep_job = None
        if self.last_sent != "S" and self.v_keep.get():
            self.keep_job = self.after(self._keep_ms(), self._keepalive)

    def _keep_ms(self):
        try:
            return max(50, min(2000, int(self.v_keep_ms.get())))
        except (tk.TclError, ValueError):
            return 250

    def _keepalive(self):
        self.keep_job = None
        if self.last_sent != "S" and self.v_keep.get():
            self.repeats += 1
            self.send(self.last_sent, quiet=True)
            self.keep_job = self.after(self._keep_ms(), self._keepalive)

    def _keep_changed(self):
        self._schedule_keepalive()

    def _invert_changed(self):
        self.stop_all()
        self._save()

    # ---------------------------------------------------------------- velocidad

    def _on_speed(self, v):
        self.speed = int(float(v))
        self._update_speed_label()
        if self.speed_job is None:
            self.speed_job = self.after(80, self._send_speed)

    def _set_speed(self, v):
        self.speed = max(0, min(255, int(v)))
        self._update_speed_label()
        self.scale.set(self.speed)
        if self.speed_job is None:
            self.speed_job = self.after(80, self._send_speed)

    def _send_speed(self):
        self.speed_job = None
        self.send(f"V{self.speed}\n")
        self._save()

    def _update_speed_label(self):
        self.lbl_speed.configure(text=f"Velocidad: {self.speed}  ({round(self.speed * 100 / 255)} %)")

    # ---------------------------------------------------------------- envío

    def send(self, text, quiet=False):
        shown = text.replace("\n", "⏎")
        if self.v_block.get():
            if not quiet:
                self._log("SYS", f"(envío bloqueado) {shown}")
            return False
        if not self.link:
            if not quiet:
                self._log("SYS", f"sin conexión, no se envió {shown}")
            return False
        try:
            self.link.write(text)
        except Exception:
            return False
        if not quiet or self.v_show_rep.get():
            self._log("TX", shown if not quiet else shown + "  (reenvío)")
        return True

    def send_manual(self):
        t = self.v_manual.get()
        if t:
            self.send(t.replace("\\n", "\n"))
            self.v_manual.set("")

    # ---------------------------------------------------------------- conexión

    def _layout_conn(self):
        mode = self.v_mode.get()
        for w in (self.lbl_dev, self.cb_port, self.btn_refresh, self.lbl_extra, self.sp_chan, self.cb_baud):
            w.grid_remove()
        if mode == "bt":
            self.lbl_dev.configure(text="Robot")
            self.cb_port.configure(textvariable=self.v_btdev)
            self.lbl_extra.configure(text="Canal")
            for w in (self.lbl_dev, self.cb_port, self.btn_refresh, self.lbl_extra, self.sp_chan):
                w.grid()
        elif mode == "serial":
            self.lbl_dev.configure(text="Puerto")
            self.cb_port.configure(textvariable=self.v_port)
            self.lbl_extra.configure(text="Baudios")
            for w in (self.lbl_dev, self.cb_port, self.btn_refresh, self.lbl_extra, self.cb_baud):
                w.grid()

    def _mode_changed(self):
        if self.link:
            self.disconnect()
        self._layout_conn()
        self.refresh_ports()
        if self.v_mode.get() == "sim":
            self.connect()
        self._save()

    def refresh_ports(self):
        mode = self.v_mode.get()
        if mode == "serial":
            values = []
            self.port_map = {}
            if HAS_SERIAL:
                for p in list_ports.comports():
                    label = f"{p.device} - {p.description}"
                    values.append(label)
                    self.port_map[label] = p.device
            else:
                self._log("SYS", "Para usar el modo Puerto COM instala pyserial:  pip install pyserial")
            self.cb_port.configure(values=values)
        elif mode == "bt":
            self.cb_port.configure(values=["Buscando dispositivos emparejados…"])
            threading.Thread(target=lambda: self.q.put(("paired", list_paired_bt())), daemon=True).start()

    def _fill_paired(self, devices):
        values = [f"{name} - {mac}" for name, mac in devices]
        self.cb_port.configure(values=values)
        if not values:
            self._log("SYS", "No encontré dispositivos emparejados. Empareja el robot en los ajustes "
                             "Bluetooth, o escribe su MAC (AA:BB:CC:DD:EE:FF).")
            return
        if not self.v_btdev.get().strip():
            for v in values:
                if any(k in v.lower() for k in ("robot", "hc-0", "bt", "arduino")):
                    self.v_btdev.set(v)
                    break

    def _port_value(self):
        v = self.v_port.get().strip()
        return self.port_map.get(v, v.split(" - ")[0].strip())

    def open_bt_settings(self):
        try:
            if os.name == "nt":
                os.startfile("ms-settings:bluetooth")
            elif sys.platform == "darwin":
                subprocess.Popen(["open", "x-apple.systempreferences:com.apple.preferences.Bluetooth"])
            else:
                subprocess.Popen(["blueman-manager"])
        except Exception:
            self._log("SYS", "Abre los ajustes Bluetooth de tu sistema y empareja el dispositivo.")

    def toggle_connect(self):
        if self.link or self.connecting:
            self.disconnect()
        else:
            self.connect()

    def connect(self, auto=False):
        if self.link or (self.connecting and not auto):
            return
        self.want_connected = True
        self.connecting = True
        self._set_status("Conectando…" if not auto else "Reconectando…", WARN)
        self.btn_conn.configure(text="Cancelar")
        mode = self.v_mode.get()

        if mode == "sim":
            self.sim = SimRobot(self.v_sim_inv_av.get(), self.v_sim_inv_piv.get())
            self._opened(SimLink(self.sim))
            return

        if mode == "bt":
            m = MAC_RE.search(self.v_btdev.get())
            if not m:
                self._open_failed("elige el robot de la lista o escribe su MAC (AA:BB:CC:DD:EE:FF)", fatal=True)
                return
            mac = m.group(1)
            try:
                chan = int(self.v_chan.get())
            except (ValueError, tk.TclError):
                chan = 1

            def make():
                return BtLink(mac, chan, self.q)
        else:
            if not HAS_SERIAL:
                self._open_failed("pyserial no está instalado (pip install pyserial)", fatal=True)
                return
            port = self._port_value()
            if not port:
                self._open_failed("elige un puerto", fatal=True)
                return
            try:
                baud = int(self.v_baud.get())
            except ValueError:
                baud = 9600

            def make():
                return SerialLink(port, baud, self.q)

        def work():
            try:
                link = make()
                link.open()
                self.q.put(("opened", link))
            except Exception as ex:
                self.q.put(("open_error", str(ex)))

        threading.Thread(target=work, daemon=True).start()

    def _opened(self, link):
        self.connecting = False
        self.link = link
        self._set_status(f"Conectado ({link.name})", OK)
        self.btn_conn.configure(text="Desconectar")
        self._log("SYS", f"Conectado a {link.name}")
        self.send(f"V{self.speed}\n")
        self._save()

    def _open_failed(self, msg, fatal=False):
        self.connecting = False
        self._set_status("Sin conexión", BAD)
        self.btn_conn.configure(text="Conectar")
        self._log("ERR", f"No se pudo conectar: {msg}")
        if fatal:
            self.want_connected = False
        else:
            self._maybe_reconnect()

    def _maybe_reconnect(self):
        if self.want_connected and self.v_auto.get() and self.v_mode.get() in ("bt", "serial"):
            self._set_status("Reconectando…", WARN)
            if self.reconnect_job:
                self.after_cancel(self.reconnect_job)
            self.reconnect_job = self.after(3000, lambda: self.connect(auto=True))

    def disconnect(self):
        self.want_connected = False
        if self.reconnect_job:
            self.after_cancel(self.reconnect_job)
            self.reconnect_job = None
        self.stop_all()
        if self.link:
            self.link.close()
            self.link = None
            self._log("SYS", "Desconectado")
        self.connecting = False
        self._set_status("Sin conexión", BAD)
        self.btn_conn.configure(text="Conectar")

    def _lost(self, link):
        if link is not self.link:
            return
        self.link.close()
        self.link = None
        self._log("ERR", "Se perdió la conexión")
        self._set_status("Conexión perdida", BAD)
        self.btn_conn.configure(text="Conectar")
        self._maybe_reconnect()

    def _set_status(self, text, color):
        self.lbl_status.configure(text=text)
        self.lbl_dot.configure(fg=color)

    def _sim_opts(self):
        if self.sim:
            self.sim.invert_avance = self.v_sim_inv_av.get()
            self.sim.invert_pivote = self.v_sim_inv_piv.get()

    # ---------------------------------------------------------------- mando

    def _init_pad(self):
        try:
            self.pad = PygameBackend()
        except Exception as ex:
            self.pad = None
            self.lbl_pad.configure(text="Mando: no disponible (instala pygame:  pip install pygame)")
            self._log("SYS", f"Mando desactivado: {ex}")
            return
        self.pad_next = 0.0
        self._set_pad_label(None)

    def _set_pad_label(self, name):
        if name:
            self.lbl_pad.configure(text=f"Mando: {name}")
        else:
            self.lbl_pad.configure(text="Mando: no detectado (empárejalo en los ajustes Bluetooth y apriétale un botón)")

    def _scan_pad_now(self):
        self.pad_next = 0.0
        self.pad_name = None
        self._log("SYS", "Buscando mando…")

    def _pad_toggle(self):
        self._log("SYS", "Mando " + ("activado" if self.v_pad.get() else "desactivado"))
        if not self.v_pad.get():
            self._pad_set(0, 0)
        self._save()

    def _pad_set(self, fb, lr):
        if (fb, lr) != (self.pad_fb, self.pad_lr):
            self.pad_fb, self.pad_lr = fb, lr
            self.refresh()

    def _poll_pad(self):
        if not self.pad or not self.v_pad.get():
            self._pad_set(0, 0)
            return
        now = time.monotonic()
        if now >= self.pad_next:
            self.pad_next = now + 1.0
            try:
                name = self.pad.scan()
            except Exception:
                name = None
            if name != self.pad_name:
                self.pad_name = name
                self._set_pad_label(name)
                self._log("SYS", f"Mando conectado: {name}" if name else "Mando desconectado")
        if not self.pad_name:
            self._pad_set(0, 0)
            return
        try:
            st = self.pad.read()
        except Exception:
            self.pad_name = None
            self._set_pad_label(None)
            self._pad_set(0, 0)
            return
        dead = 0.5
        lr = 1 if st["x"] > dead else -1 if st["x"] < -dead else 0
        fb = 1 if st["y"] < -dead else -1 if st["y"] > dead else 0
        if st["dx"]:
            lr = st["dx"]
        if st["dy"]:
            fb = st["dy"]
        self._pad_set(fb, lr)
        if st["r1"] and not self.pad_r1:
            self._set_speed(self.speed + 25)
        if st["l1"] and not self.pad_l1:
            self._set_speed(self.speed - 25)
        self.pad_r1, self.pad_l1 = st["r1"], st["l1"]
        if self.v_pad_trig.get() and st["rt"] > 0.05:
            v = int(st["rt"] * 255)
            if abs(v - self.speed) >= 8:
                self._set_speed(v)

    # ---------------------------------------------------------------- secuencia de prueba

    def toggle_sequence(self):
        if self.seq_job:
            self.stop_sequence()
            self.apply_move(0, 0)
        else:
            steps = []
            for fb, lr in ((1, 0), (-1, 0), (0, -1), (0, 1), (1, -1), (1, 1), (-1, -1), (-1, 1)):
                steps.append((fb, lr, 1200))
                steps.append((0, 0, 400))
            self._log("SYS", "Secuencia de prueba: F B L R G I H J")
            self.btn_seq.configure(text="■ Detener secuencia")
            self.seq_job = "run"
            self._seq_step(steps, 0)

    def _seq_step(self, steps, i):
        if self.seq_job is None:
            return
        if i >= len(steps):
            self.stop_sequence()
            self.apply_move(0, 0)
            self._log("SYS", "Secuencia terminada")
            return
        fb, lr, ms = steps[i]
        self.apply_move(fb, lr)
        self.seq_job = self.after(ms, lambda: self._seq_step(steps, i + 1))

    def stop_sequence(self):
        if self.seq_job and self.seq_job != "run":
            self.after_cancel(self.seq_job)
        self.seq_job = None
        self.btn_seq.configure(text="▶ Secuencia de prueba")

    # ---------------------------------------------------------------- registro

    def _log(self, tag, msg):
        ts = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
        self.txt.configure(state="normal")
        self.txt.insert("end", f"{ts} {tag:<3} {msg}\n", tag)
        lines = int(self.txt.index("end-1c").split(".")[0])
        if lines > 500:
            self.txt.delete("1.0", f"{lines - 500}.0")
        self.txt.see("end")
        self.txt.configure(state="disabled")

    def _clear_log(self):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")

    # ---------------------------------------------------------------- bucle principal

    def _loop(self):
        try:
            while True:
                kind, val = self.q.get_nowait()
                if kind == "rx":
                    self._log("RX", val.decode("ascii", "replace").strip() or repr(val))
                elif kind == "lost":
                    self._lost(val)
                elif kind == "opened":
                    if self.want_connected:
                        self._opened(val)
                    else:
                        val.close()
                elif kind == "open_error":
                    self._open_failed(val)
                elif kind == "paired":
                    self._fill_paired(val)
        except queue.Empty:
            pass

        self._poll_pad()

        if self.sim and isinstance(self.link, SimLink):
            if self.sim.tick(time.monotonic()):
                self._log("SYS", "SIM: no llegaron comandos -> freno de seguridad, robot detenido")
            self._update_sim_panel(True)
        else:
            self._update_sim_panel(False)

        self._draw()
        self.after(33, self._loop)

    def _update_sim_panel(self, active):
        if active:
            s = self.sim
            self.bar_l.configure(value=s.left + 255)
            self.bar_r.configure(value=s.right + 255)
            self.lbl_ml.configure(text=f"Motor izq: {s.left:+d}")
            self.lbl_mr.configure(text=f"Motor der: {s.right:+d}")
            self.lbl_sim.configure(text=f"velocidad={s.speed}  acel={s.acel:+d}  giro={s.giro:+d}\n"
                                        f"{'en movimiento' if s.moving else 'detenido'}")
        else:
            self.bar_l.configure(value=255)
            self.bar_r.configure(value=255)
            self.lbl_ml.configure(text="Motor izq: -")
            self.lbl_mr.configure(text="Motor der: -")
            self.lbl_sim.configure(text="(solo en modo simulador)")
        self.lbl_info.configure(
            text=f"último enviado: {self.last_sent}   ·   reenvíos: {self.repeats}   ·   "
                 f"velocidad: {self.speed}")

    # ---------------------------------------------------------------- dibujo del robot

    def _draw(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 50 or h < 50:
            return
        u = min(w, h) / 2 * 0.95
        cx, cy = w / 2, h / 2
        cmd = self.ui_cmd
        moving = cmd != "S"

        now = time.monotonic()
        dt = now - self._last_draw
        self._last_draw = now
        if moving:
            self.phase += dt * u * (0.5 + 1.5 * self.speed / 255)

        left, right = WHEELS[cmd]
        self._wheel(cx - 0.52 * u, cy, u, left)
        self._wheel(cx + 0.52 * u, cy, u, right)

        round_rect(c, cx - 0.42 * u, cy - 0.55 * u, cx + 0.42 * u, cy + 0.55 * u, 0.12 * u,
                   fill="#37474F", outline="")
        round_rect(c, cx - 0.34 * u, cy - 0.47 * u, cx + 0.34 * u, cy - 0.08 * u, 0.08 * u,
                   fill="#546E7A", outline="")

        look_x = (-1 if cmd in "LGH" else 1 if cmd in "RIJ" else 0) * 0.025 * u
        look_y = (-1 if cmd in "FGI" else 1 if cmd in "BHJ" else 0) * 0.025 * u
        for side in (-1, 1):
            ex, ey = cx + side * 0.15 * u, cy - 0.27 * u
            c.create_oval(ex - 0.09 * u, ey - 0.09 * u, ex + 0.09 * u, ey + 0.09 * u, fill="white", outline="")
            px, py = ex + look_x, ey + look_y
            c.create_oval(px - 0.045 * u, py - 0.045 * u, px + 0.045 * u, py + 0.045 * u, fill="#111", outline="")

        bx, by, br = cx, cy + 0.18 * u, 0.17 * u
        if moving:
            c.create_oval(bx - br, by - br, bx + br, by + br, fill="#4CAF50", outline="")
        else:
            c.create_oval(bx - br, by - br, bx + br, by + br, fill="#E53935", outline="")
            c.create_rectangle(bx - 0.07 * u, by - 0.07 * u, bx + 0.07 * u, by + 0.07 * u, fill="white", outline="")

        led = "#69F0AE" if self.link else "#FF5252"
        c.create_oval(cx - 0.05 * u, cy + 0.44 * u - 0.05 * u, cx + 0.05 * u, cy + 0.44 * u + 0.05 * u,
                      fill=led, outline="")

        if moving:
            ang = math.radians(ARROW_ANGLE[cmd])
            aw, ah = 0.14 * u, 0.16 * u
            local = [(0, -ah), (aw, 0), (aw * 0.4, 0), (aw * 0.4, ah * 0.9),
                     (-aw * 0.4, ah * 0.9), (-aw * 0.4, 0), (-aw, 0)]
            pts = []
            for x, y in local:
                y -= 0.80 * u
                rx = x * math.cos(ang) - y * math.sin(ang)
                ry = x * math.sin(ang) + y * math.cos(ang)
                pts += [cx + rx, cy + ry]
            c.create_polygon(pts, fill="#F5A623", outline="")

    def _wheel(self, wx, wy, u, d):
        c = self.canvas
        hw, hh = 0.10 * u, 0.36 * u
        top, bot = wy - hh, wy + hh
        round_rect(c, wx - hw, top, wx + hw, bot, 0.06 * u,
                   fill="#2E7D32" if d != 0 else "#263238", outline="")
        gap = 0.12 * u
        off = (-d * self.phase) % gap
        y = top - gap
        while y < bot:
            ya = max(y + off, top)
            yb = min(y + off + gap * 0.3, bot)
            if yb > ya:
                c.create_rectangle(wx - hw, ya, wx + hw, yb, fill="#1B2A20" if d != 0 else "#37474F", outline="")
            y += gap

    # ---------------------------------------------------------------- cierre

    def _save(self):
        self.cfg.update({
            "mode": self.v_mode.get(), "port": self.v_port.get(), "baud": self.v_baud.get(),
            "speed": self.speed, "invert_lr": self.v_inv_lr.get(), "invert_fb": self.v_inv_fb.get(),
            "auto_reconnect": self.v_auto.get(), "keepalive": self.v_keep.get(),
            "keepalive_ms": self._keep_ms(),
            "btdev": self.v_btdev.get(), "chan": self.v_chan.get(),
            "use_pad": self.v_pad.get(), "pad_trig": self.v_pad_trig.get(),
        })
        save_cfg(self.cfg)

    def on_close(self):
        try:
            self.send("S")
        except Exception:
            pass
        self._save()
        if self.link:
            self.link.close()
        self.destroy()


def main():
    if tk is None:
        print("Falta tkinter. En Ubuntu/WSL:  sudo apt install python3-tk")
        raise SystemExit(1)
    App().mainloop()


if __name__ == "__main__":
    main()
