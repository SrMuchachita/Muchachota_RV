#!/usr/bin/env python3
"""
RD90 Pro - Monitor de IMU por Serial (solo lectura)

Se conecta al puerto COM del ESP32-S3 (el mismo que usa el monitor serial
de idf.py) y parsea las líneas que el firmware ya imprime:

    I (12345) RD90Pro: [RD90R-260008] IMU: pitch=0.38° roll=-0.14° temp=26.1°C

No escribe nada al dispositivo (no usa Modbus, no envía comandos).
Solo lee texto plano del puerto serie y lo muestra.

Requisitos:
    pip install pyserial

Uso:
    python imu_monitor_gui.py
"""

import re
import math
import time
import queue
import threading
import collections
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

DEFAULT_BAUD = "115200"

# Ej: "I (12345) RD90Pro: [RD90R-260008] IMU: pitch=0.38° roll=-0.14° temp=26.1°C"
IMU_LINE_RE = re.compile(
    r"\[([^\]]+)\]\s*IMU:\s*pitch=(-?[\d.]+)°\s*roll=(-?[\d.]+)°\s*temp=(-?[\d.]+)°C"
)

# ── Tema (mismo estilo que RD90PRO_V9_CONSOLA_VIRTUAL.py, versión "yellow") ──
BG    = "#0D1117"
CARD  = "#070809"
CARD2 = "#0C0D0F"
AMBER = "#FFD000"
GREEN = "#39FF6A"
RED   = "#FF4C4C"
WHITE = "#FFF5D0"
GRAY  = "#8B7A45"

FT = ("Consolas", 16, "bold")
FS = ("Consolas", 12, "bold")
FL = ("Consolas", 9)
FM = ("Consolas", 9)

# Rango de inclinación que corresponde al radio completo del indicador (°)
LEVEL_RANGE_DEG = 15.0

# ── Geometría del "chasis" 3D (robot de 4 ruedas visto como caja) ────────────
# Eje local: x=adelante, y=izquierda, z=arriba
BOX_L2, BOX_W2, BOX_H2 = 1.0, 0.6, 0.18  # medio-largo, medio-ancho, medio-alto

BOX_VERTICES = [
    (-BOX_L2, -BOX_W2, -BOX_H2),  # 0
    ( BOX_L2, -BOX_W2, -BOX_H2),  # 1
    ( BOX_L2,  BOX_W2, -BOX_H2),  # 2
    (-BOX_L2,  BOX_W2, -BOX_H2),  # 3
    (-BOX_L2, -BOX_W2,  BOX_H2),  # 4
    ( BOX_L2, -BOX_W2,  BOX_H2),  # 5
    ( BOX_L2,  BOX_W2,  BOX_H2),  # 6
    (-BOX_L2,  BOX_W2,  BOX_H2),  # 7
]

# Caras con orden CCW visto desde afuera (para culling por producto cruz)
BOX_FACES = {
    "bottom":  ([0, 3, 2, 1], "#2A2200"),
    "top":     ([4, 5, 6, 7], AMBER),
    "front":   ([1, 2, 6, 5], "#3AA6FF"),
    "back":    ([0, 4, 7, 3], "#4A4230"),
    "right":   ([3, 7, 6, 2], "#6B5A10"),
    "left":    ([0, 1, 5, 4], "#6B5A10"),
}

# Ángulo fijo de cámara para dar el efecto isométrico
VIEW_YAW_DEG  = -35.0
VIEW_TILT_DEG = 20.0

# Amplificación visual de la caja 3D: da un pequeño empuje a inclinaciones
# casi nulas (reposo) sin distorsionar giros reales grandes. Se satura suave
# (tanh), nunca "salta" de golpe al tope como haría un clamp lineal duro.
ROBOT3D_GAIN = 1.0
ROBOT3D_MAX_DEG = 75.0


def _amplify_for_display(angle_deg):
    x = angle_deg * ROBOT3D_GAIN / ROBOT3D_MAX_DEG
    return ROBOT3D_MAX_DEG * math.tanh(x)


# ── Traza de pitch tipo "grabadora de vuelo" (cinta corrediza) ──────────────
TRACE_PANEL_W = 800
TRACE_PANEL_H = 160
TRACE_STEP_PX = 10  # separación horizontal entre muestras consecutivas (más alto = recorrido más rápido)
TRACE_MAXLEN = TRACE_PANEL_W // TRACE_STEP_PX


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_vec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


_R_VIEW = _mat_mul(_rot_x(math.radians(VIEW_TILT_DEG)), _rot_z(math.radians(VIEW_YAW_DEG)))

# Giro fijo de 180° del modelo completo (como si el robot diera media vuelta),
# a pedido: no basta con invertir la flecha, hay que rotar la caja entera.
_R_FLIP_180 = _rot_z(math.radians(180.0))


def project_robot_box(pitch_deg, roll_deg):
    """Rota la caja del robot según pitch/roll y proyecta a 2D.

    Devuelve (vertices_view, faces_visibles_ordenadas_de_lejos_a_cerca).
    vertices_view: lista de (screen_x, screen_y, depth) por índice de vértice.
    faces_visibles: lista de (color, [(screen_x, screen_y), ...]) para dibujar en orden.
    """
    r_obj = _mat_mul(_rot_y(math.radians(pitch_deg)), _rot_x(math.radians(roll_deg)))
    r_obj = _mat_mul(_R_FLIP_180, r_obj)
    r_total = _mat_mul(_R_VIEW, r_obj)

    vv = [_mat_vec(r_total, v) for v in BOX_VERTICES]
    # screen_x = vx, screen_y = -vz (arriba en 3D = arriba en pantalla), depth = vy
    screen = [(v[0], -v[2], v[1]) for v in vv]

    visible = []
    for _name, (idxs, color) in BOX_FACES.items():
        p0, p1, p2 = vv[idxs[0]], vv[idxs[1]], vv[idxs[2]]
        normal = _cross(_sub(p1, p0), _sub(p2, p0))
        if normal[1] >= 0:
            continue  # cara de atrás, no se dibuja
        avg_depth = sum(vv[i][1] for i in idxs) / 4.0
        pts = [(screen[i][0], screen[i][1]) for i in idxs]
        visible.append((avg_depth, color, pts))

    # Pintor: dibujar primero lo más lejano (depth mayor = más lejos de la cámara)
    visible.sort(key=lambda t: t[0], reverse=True)
    faces_out = [(color, pts) for _, color, pts in visible]

    # Flecha de "frente": desde el centro de la cara frontal hacia +x, más allá de la caja
    front_center_local = (BOX_L2, 0.0, 0.0)
    front_tip_local = (BOX_L2 * 1.7, 0.0, 0.0)
    fc = _mat_vec(r_total, front_center_local)
    ft = _mat_vec(r_total, front_tip_local)
    arrow = ((fc[0], -fc[2]), (ft[0], -ft[2]))

    return screen, faces_out, arrow


def get_ports():
    if not SERIAL_AVAILABLE:
        return []
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


class SerialReader:
    """Lee líneas del puerto serie en un hilo separado y las mete en una cola."""

    def __init__(self, data_queue: "queue.Queue"):
        self.data_queue = data_queue
        self._ser = None
        self._thread = None
        self._stop = threading.Event()

    @property
    def connected(self):
        return self._ser is not None and self._ser.is_open

    def connect(self, port, baud):
        self.disconnect()
        self._ser = serial.Serial(port, int(baud), timeout=1)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._thread = None

    def _run(self):
        while not self._stop.is_set() and self._ser is not None:
            try:
                raw = self._ser.readline()
            except Exception as e:
                self.data_queue.put(("error", str(e)))
                break
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            self.data_queue.put(("line", line))


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("RD90 Pro — Monitor IMU (solo lectura)")
        self.root.configure(bg=BG)
        self.root.geometry("1120x1050")

        self.port_var = tk.StringVar(value="")
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        self.data_queue = queue.Queue()
        self.reader = SerialReader(self.data_queue)
        self.pitch_trace = collections.deque(maxlen=TRACE_MAXLEN)

        self._ui()
        self._refresh_ports()
        self._poll_queue()

    # ── UI ───────────────────────────────────────────────────
    def _ui(self):
        h = tk.Frame(self.root, bg=BG)
        h.pack(fill="x", padx=15, pady=(12, 4))
        tk.Label(h, text="⬡  MONITOR IMU", font=FT, fg=AMBER, bg=BG).pack(side="left")
        tk.Label(h, text="solo lectura · Serial", font=FL, fg=GRAY, bg=BG).pack(side="left", padx=8)

        conn = tk.Frame(self.root, bg=CARD, padx=10, pady=8)
        conn.pack(fill="x", padx=15, pady=(0, 8))

        tk.Label(conn, text="Puerto:", font=FL, fg=GRAY, bg=CARD).grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(conn, textvariable=self.port_var, width=10, font=FM)
        self.port_cb.grid(row=0, column=1, padx=(4, 10))

        tk.Label(conn, text="Baud:", font=FL, fg=GRAY, bg=CARD).grid(row=0, column=2, sticky="w")
        ttk.Combobox(conn, textvariable=self.baud_var, width=8, font=FM,
                     values=["9600", "19200", "38400", "57600", "115200"]
                     ).grid(row=0, column=3, padx=(4, 10))

        tk.Button(conn, text="↺", font=FL, bg=CARD2, fg=GRAY, relief="flat",
                  cursor="hand2", command=self._refresh_ports).grid(row=0, column=4, padx=2)

        self.conn_btn = tk.Button(conn, text="CONECTAR", font=FS, bg=AMBER, fg=BG,
                                   relief="flat", cursor="hand2", padx=8,
                                   command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=5, padx=(6, 0))

        self.status_var = tk.StringVar(value="Sin conexión")
        tk.Label(self.root, textvariable=self.status_var, font=FL, fg=GRAY,
                  bg=CARD2, anchor="w", padx=10, pady=4).pack(fill="x", padx=15)

        # ── Visualizaciones: inclinación pegada a la izquierda, robot 3D
        # pegado a la derecha, vista lateral centrada arriba en el medio ────
        viz_row = tk.Frame(self.root, bg=BG)
        viz_row.pack(fill="x", padx=15, pady=8)
        viz_row.columnconfigure(0, weight=1)
        viz_row.columnconfigure(1, weight=0)
        viz_row.columnconfigure(2, weight=1)

        level_card = tk.Frame(viz_row, bg=CARD, padx=10, pady=10)
        level_card.grid(row=0, column=0, sticky="w")
        tk.Label(level_card, text="INCLINACIÓN", font=FL, fg=GRAY, bg=CARD).pack()
        self.canvas = tk.Canvas(level_card, width=240, height=240, bg=CARD2, highlightthickness=0)
        self.canvas.pack(pady=(4, 0))
        self._draw_level(0.0, 0.0)

        side_card = tk.Frame(viz_row, bg=CARD, padx=10, pady=10)
        side_card.grid(row=0, column=1, padx=8)
        tk.Label(side_card, text="VISTA LATERAL (pitch)", font=FL, fg=GRAY, bg=CARD).pack()
        self.side_canvas = tk.Canvas(side_card, width=500, height=150, bg=CARD2, highlightthickness=0)
        self.side_canvas.pack(pady=(4, 0))
        self._draw_side_view(0.0)

        robot_card = tk.Frame(viz_row, bg=CARD, padx=10, pady=10)
        robot_card.grid(row=0, column=2, sticky="e")
        tk.Label(robot_card, text="ROBOT 3D (4 ruedas)", font=FL, fg=GRAY, bg=CARD).pack()
        self.robot_canvas = tk.Canvas(robot_card, width=240, height=240, bg=CARD2, highlightthickness=0)
        self.robot_canvas.pack(pady=(4, 0))
        self._draw_robot3d(0.0, 0.0)

        # ── Traza de pitch tipo "grabadora de vuelo" (cinta corrediza) ────
        trace_card = tk.Frame(self.root, bg=CARD, padx=10, pady=10)
        trace_card.pack(padx=15, pady=(0, 8))
        tk.Label(trace_card, text="TRAZA DE PITCH (perfil, cinta corrediza)",
                 font=FL, fg=GRAY, bg=CARD).pack()
        self.trace_canvas = tk.Canvas(trace_card, width=TRACE_PANEL_W, height=TRACE_PANEL_H,
                                       bg=CARD2, highlightthickness=0)
        self.trace_canvas.pack(pady=(4, 0))
        self._draw_pitch_trace()

        # ── Lecturas numéricas ─────────────────────────────────
        num = tk.Frame(self.root, bg=BG)
        num.pack(fill="x", padx=15, pady=(8, 4))
        self.svars = {k: tk.StringVar(value="—") for k in ("pitch", "roll", "temp", "device")}
        for i, (label, key, unit, col) in enumerate([
            ("Pitch", "pitch", "°", AMBER),
            ("Roll", "roll", "°", AMBER),
            ("Temp", "temp", "°C", GREEN),
        ]):
            f = tk.Frame(num, bg=CARD2, padx=6, pady=4)
            f.grid(row=0, column=i, padx=4, sticky="ew")
            num.columnconfigure(i, weight=1)
            tk.Label(f, text=label, font=FL, fg=GRAY, bg=CARD2).pack()
            vf = tk.Frame(f, bg=CARD2); vf.pack()
            tk.Label(vf, textvariable=self.svars[key], font=("Consolas", 15, "bold"),
                     fg=col, bg=CARD2).pack(side="left")
            tk.Label(vf, text=unit, font=FL, fg=GRAY, bg=CARD2).pack(side="left", padx=2)

        dev_row = tk.Frame(self.root, bg=BG)
        dev_row.pack(fill="x", padx=15, pady=(0, 8))
        tk.Label(dev_row, text="Dispositivo:", font=FL, fg=GRAY, bg=BG).pack(side="left")
        tk.Label(dev_row, textvariable=self.svars["device"], font=FL, fg=WHITE, bg=BG).pack(side="left", padx=4)

        # ── Log crudo ────────────────────────────────────────
        log_card = tk.Frame(self.root, bg=CARD, padx=10, pady=6)
        log_card.pack(fill="both", expand=True, padx=15, pady=(0, 12))
        tk.Label(log_card, text="  LOG SERIAL", font=FL, fg=GRAY, bg=CARD, anchor="w").pack(fill="x")
        self.log_txt = tk.Text(log_card, bg=CARD2, fg=AMBER, font=FM, relief="flat",
                                state="disabled", height=8)
        self.log_txt.pack(fill="both", expand=True, pady=(4, 0))

    def _refresh_ports(self):
        ports = get_ports()
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ── conexión ─────────────────────────────────────────────
    def _toggle_connect(self):
        if self.reader.connected:
            self.reader.disconnect()
            self.conn_btn.configure(text="CONECTAR", bg=AMBER)
            self.status_var.set("Desconectado")
            return

        if not SERIAL_AVAILABLE:
            self.status_var.set("Falta pyserial: pip install pyserial")
            return

        port = self.port_var.get()
        baud = self.baud_var.get()
        if not port:
            self.status_var.set("Selecciona un puerto")
            return
        try:
            self.reader.connect(port, baud)
            self.conn_btn.configure(text="DESCONECTAR", bg=RED, fg=WHITE)
            self.status_var.set(f"Conectado a {port} @ {baud} bps")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

    # ── cola de datos del hilo lector ─────────────────────────
    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.data_queue.get_nowait()
                if kind == "error":
                    self.status_var.set(f"Error de puerto: {payload}")
                    self.reader.disconnect()
                    self.conn_btn.configure(text="CONECTAR", bg=AMBER, fg=BG)
                elif kind == "line":
                    self._handle_line(payload)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _handle_line(self, line):
        self._log(line)
        m = IMU_LINE_RE.search(line)
        if not m:
            return
        device, pitch, roll, temp = m.groups()
        pitch, roll, temp = float(pitch), float(roll), float(temp)
        roll = -roll  # signo invertido a pedido: quedaba al revés para el usuario
        self.svars["pitch"].set(f"{pitch:+.2f}")
        self.svars["roll"].set(f"{roll:+.2f}")
        self.svars["temp"].set(f"{temp:.1f}")
        self.svars["device"].set(device)
        self._draw_level(pitch, roll)
        self._draw_robot3d(pitch, roll)
        self._draw_side_view(pitch)
        self._push_pitch_trace(pitch)

    def _log(self, msg):
        self.log_txt.configure(state="normal")
        self.log_txt.insert("end", f"{msg}\n")
        self.log_txt.see("end")
        # Mantener solo las últimas ~300 líneas
        if float(self.log_txt.index("end-1c").split(".")[0]) > 300:
            self.log_txt.delete("1.0", "2.0")
        self.log_txt.configure(state="disabled")

    # ── indicador de nivel (burbuja) ───────────────────────────
    def _draw_level(self, pitch, roll):
        cv = self.canvas
        cv.delete("all")
        cx, cy, r = 120, 120, 100

        cv.create_oval(cx - r, cy - r, cx + r, cy + r, outline=GRAY, width=2)
        cv.create_line(cx - r, cy, cx + r, cy, fill=GRAY, dash=(2, 3))
        cv.create_line(cx, cy - r, cx, cy + r, fill=GRAY, dash=(2, 3))
        # anillo de referencia "nivel" al 20% del radio
        cv.create_oval(cx - r * 0.15, cy - r * 0.15, cx + r * 0.15, cy + r * 0.15,
                        outline=GREEN, dash=(2, 2))

        # Roll mueve la burbuja horizontalmente, Pitch verticalmente
        # (pitch positivo = "nariz arriba" -> burbuja sube en pantalla)
        scale = r / LEVEL_RANGE_DEG
        bx = cx + max(-r, min(r, roll * scale))
        by = cy - max(-r, min(r, pitch * scale))

        # Clamp para que la burbuja no salga del círculo
        dx, dy = bx - cx, by - cy
        dist = (dx * dx + dy * dy) ** 0.5
        if dist > r - 10:
            k = (r - 10) / dist if dist else 0
            bx, by = cx + dx * k, cy + dy * k

        level = (roll ** 2 + pitch ** 2) ** 0.5 < 0.5
        color = GREEN if level else AMBER
        rad = 10
        cv.create_oval(bx - rad, by - rad, bx + rad, by + rad, fill=color, outline="")

    # ── robot 3D (caja con flecha de frente) ────────────────────
    def _draw_robot3d(self, pitch, roll):
        cv = self.robot_canvas
        cv.delete("all")
        cx, cy, scale = 120, 130, 70

        def to_canvas(pt):
            return (cx + pt[0] * scale, cy + pt[1] * scale)

        # Amplificación visual: en reposo el robot suele inclinarse <1°, lo cual
        # es casi imperceptible al rotar la caja. Se exagera solo para el dibujo
        # (los valores numéricos mostrados siguen siendo los reales, sin amplificar).
        vis_pitch = _amplify_for_display(pitch)
        vis_roll = _amplify_for_display(roll)
        _screen, faces, arrow = project_robot_box(vis_pitch, vis_roll)

        for color, pts in faces:
            poly = []
            for p in pts:
                sx, sy = to_canvas(p)
                poly.extend([sx, sy])
            cv.create_polygon(poly, fill=color, outline=CARD2, width=2)

        # Flecha indicando "adelante"
        p0 = to_canvas(arrow[0])
        p1 = to_canvas(arrow[1])
        cv.create_line(*p0, *p1, fill=GREEN, width=3, arrow=tk.LAST, arrowshape=(12, 14, 6))
        cv.create_text(p1[0], p1[1] - 12, text="FRENTE", fill=GREEN, font=FL)

    # ── vista lateral 2D (perfil del robot, rota con el pitch) ──────────
    def _draw_side_view(self, pitch):
        cv = self.side_canvas
        cv.delete("all")
        cx, cy = 250, 90
        scale = 1.6

        # Línea de referencia fija (nivel real), no rota con el robot
        cv.create_line(30, cy + 40, 470, cy + 40, fill=GRAY, dash=(3, 3))
        cv.create_text(30, cy + 40 - 12, text="nivel", fill=GRAY, font=FL, anchor="w")

        vis_pitch = _amplify_for_display(pitch)
        theta = math.radians(vis_pitch)  # signo invertido a pedido

        def rot(x, y):
            c, s = math.cos(theta), math.sin(theta)
            return (cx + (x * c - y * s) * scale, cy + (x * s + y * c) * scale)

        half_len, half_h, wheel_r = 65, 12, 13
        body = [(-half_len, -half_h), (half_len, -half_h),
                (half_len, half_h), (-half_len, half_h)]
        body_screen = [rot(x, y) for x, y in body]
        poly = [c for p in body_screen for c in p]
        cv.create_polygon(poly, fill=AMBER, outline=CARD2, width=2)

        for wx in (-half_len + 18, half_len - 18):
            wcx, wcy = rot(wx, half_h)
            cv.create_oval(wcx - wheel_r, wcy - wheel_r, wcx + wheel_r, wcy + wheel_r,
                            fill="#2A2200", outline=GRAY, width=2)

        # Flecha de "frente" en el extremo derecho de la silueta
        fx0, fy0 = rot(half_len, -half_h / 2)
        fx1, fy1 = rot(half_len + 30, -half_h / 2)
        cv.create_line(fx0, fy0, fx1, fy1, fill=GREEN, width=3, arrow=tk.LAST, arrowshape=(10, 12, 5))
        cv.create_text(fx1 + 5, fy1, text="FRENTE", fill=GREEN, font=FL, anchor="w")

    # ── traza de pitch: cinta corrediza, pendiente del trazo = pitch actual ──
    def _draw_pitch_trace(self):
        cv = self.trace_canvas
        cv.delete("all")
        w, h = TRACE_PANEL_W, TRACE_PANEL_H
        cy0 = h / 2

        # Línea de referencia fija (nivel), no se mueve con el trazo
        cv.create_line(0, cy0, w, cy0, fill=GRAY, dash=(3, 3))
        cv.create_text(6, cy0 - 10, text="nivel", fill=GRAY, font=FL, anchor="w")

        x, y = 0.0, cy0
        pts = [(x, y)]
        for pitch in self.pitch_trace:
            vis = _amplify_for_display(pitch)
            rad = math.radians(vis)
            x += TRACE_STEP_PX * math.cos(rad)
            y += TRACE_STEP_PX * math.sin(rad)  # signo invertido a pedido: positivo = sube en pantalla
            y = max(4, min(h - 4, y))  # no salir de los bordes del panel
            pts.append((x, y))

        if len(pts) >= 2:
            flat = [c for p in pts for c in p]
            cv.create_line(*flat, fill=AMBER, width=2, smooth=True)

        # Punto "actual": círculo grande y sólido, sin líneas que compitan con él
        lx, ly = pts[-1]
        cv.create_oval(lx - 9, ly - 9, lx + 9, ly + 9, fill=GREEN, outline=WHITE, width=2)

    def _push_pitch_trace(self, pitch):
        self.pitch_trace.append(pitch)
        self._draw_pitch_trace()


def main():
    if not SERIAL_AVAILABLE:
        print("⚠  pip install pyserial")
    root = tk.Tk()
    style = ttk.Style()
    style.theme_use("default")
    style.configure("TCombobox", fieldbackground=CARD2, background=CARD2,
                     foreground=WHITE, selectbackground=CARD2, selectforeground=WHITE)
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
