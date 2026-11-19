#!/usr/bin/env python3
"""
RD90 Pro - Traza de Pitch por Serial (solo lectura)

GUI independiente y liviano, enfocado solo en la traza de pitch tipo
"grabadora de vuelo" (cinta corrediza): un punto avanza constantemente,
y la pendiente del trazo que deja atrás representa el pitch en cada
instante (pitch=0° -> línea recta; pitch=45° -> el trazo se inclina 45°).

Se conecta al puerto COM del ESP32-S3 y parsea las líneas que el firmware
ya imprime:

    I (12345) RD90Pro: [RD90R-260008] IMU: pitch=0.38° roll=-0.14° temp=26.1°C

No escribe nada al dispositivo (no usa Modbus, no envía comandos).

Requisitos:
    pip install pyserial

Uso:
    python pitch_trace_gui.py
"""

import re
import math
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

# ── Tema (mismo estilo que imu_monitor_gui.py) ───────────────────────────────
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

# ── Amplificación visual: pequeño empuje a inclinaciones casi nulas (reposo)
# sin distorsionar giros reales grandes. Se satura suave (tanh). ────────────
TRACE_GAIN = 1.0
TRACE_MAX_DEG = 75.0


def _amplify_for_display(angle_deg):
    x = angle_deg * TRACE_GAIN / TRACE_MAX_DEG
    return TRACE_MAX_DEG * math.tanh(x)


# ── Traza de pitch tipo "grabadora de vuelo" (cinta corrediza) ──────────────
TRACE_PANEL_W = 900
TRACE_PANEL_H = 260
TRACE_STEP_PX = 10  # separación horizontal entre muestras consecutivas
TRACE_MAXLEN = TRACE_PANEL_W // TRACE_STEP_PX


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
        self.root.title("RD90 Pro — Traza de Pitch (solo lectura)")
        self.root.configure(bg=BG)
        self.root.geometry("940x460")

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
        tk.Label(h, text="⬡  TRAZA DE PITCH", font=FT, fg=AMBER, bg=BG).pack(side="left")
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

        # ── Traza de pitch ───────────────────────────────────
        trace_card = tk.Frame(self.root, bg=CARD, padx=10, pady=10)
        trace_card.pack(padx=15, pady=8)
        tk.Label(trace_card, text="TRAZA DE PITCH (perfil, cinta corrediza)",
                 font=FL, fg=GRAY, bg=CARD).pack()
        self.trace_canvas = tk.Canvas(trace_card, width=TRACE_PANEL_W, height=TRACE_PANEL_H,
                                       bg=CARD2, highlightthickness=0)
        self.trace_canvas.pack(pady=(4, 0))
        self._draw_pitch_trace()

        # ── Lectura numérica de pitch ─────────────────────────
        num = tk.Frame(self.root, bg=CARD2, padx=6, pady=4)
        num.pack(padx=15, pady=(0, 10))
        tk.Label(num, text="Pitch:", font=FL, fg=GRAY, bg=CARD2).pack(side="left")
        self.pitch_var = tk.StringVar(value="—")
        tk.Label(num, textvariable=self.pitch_var, font=("Consolas", 15, "bold"),
                 fg=AMBER, bg=CARD2).pack(side="left", padx=4)
        tk.Label(num, text="°", font=FL, fg=GRAY, bg=CARD2).pack(side="left")

    def _refresh_ports(self):
        ports = get_ports()
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ── conexión ─────────────────────────────────────────────
    def _toggle_connect(self):
        if self.reader.connected:
            self.reader.disconnect()
            self.conn_btn.configure(text="CONECTAR", bg=AMBER, fg=BG)
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
        m = IMU_LINE_RE.search(line)
        if not m:
            return
        _device, pitch, _roll, _temp = m.groups()
        pitch = float(pitch)
        self.pitch_var.set(f"{pitch:+.2f}")
        self.pitch_trace.append(pitch)
        self._draw_pitch_trace()

    # ── traza de pitch: cinta corrediza, pendiente del trazo = pitch actual ──
    def _draw_pitch_trace(self):
        cv = self.trace_canvas
        cv.delete("all")
        w, h = TRACE_PANEL_W, TRACE_PANEL_H
        cy0 = h / 2

        cv.create_line(0, cy0, w, cy0, fill=GRAY, dash=(3, 3))
        cv.create_text(6, cy0 - 10, text="nivel", fill=GRAY, font=FL, anchor="w")

        x, y = 0.0, cy0
        pts = [(x, y)]
        for pitch in self.pitch_trace:
            vis = _amplify_for_display(pitch)
            rad = math.radians(vis)
            x += TRACE_STEP_PX * math.cos(rad)
            y += TRACE_STEP_PX * math.sin(rad)  # positivo = sube en pantalla
            y = max(4, min(h - 4, y))
            pts.append((x, y))

        if len(pts) >= 2:
            flat = [c for p in pts for c in p]
            cv.create_line(*flat, fill=AMBER, width=2, smooth=True)

        # Punto "actual": círculo grande y sólido, sin líneas que compitan con él
        lx, ly = pts[-1]
        cv.create_oval(lx - 9, ly - 9, lx + 9, ly + 9, fill=GREEN, outline=WHITE, width=2)


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
