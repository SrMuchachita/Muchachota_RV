#!/usr/bin/env python3
"""
GUI para subir (flashear) el firmware a una ESP32-S3 usando esptool.

Requiere: pip install esptool customtkinter

Uso:
    python flash_esp32s3.py
"""

import os
import queue
import re
import sys
import time
import zlib
import struct
import asyncio
import threading
from tkinter import filedialog

try:
    import customtkinter as ctk
except ImportError:
    ctk = None

try:
    import esptool
except ImportError:
    esptool = None

try:
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    from bleak import BleakScanner, BleakClient
    BLEAK_OK = True
except ImportError:
    BleakScanner = BleakClient = None
    BLEAK_OK = False

ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")

def _detect_script_dir():
    if getattr(sys, "frozen", False):
        exe_dir = os.path.dirname(os.path.abspath(sys.executable))
        # En un .app de macOS el ejecutable vive en Nombre.app/Contents/MacOS/;
        # usamos la carpeta que contiene el .app, no esa ruta interna.
        if sys.platform == "darwin" and exe_dir.endswith(os.path.join("Contents", "MacOS")):
            app_bundle = os.path.dirname(os.path.dirname(exe_dir))
            return os.path.dirname(app_bundle)
        return exe_dir
    return os.path.dirname(os.path.abspath(__file__))


SCRIPT_DIR = _detect_script_dir()

FLASH_MODE = "dio"
FLASH_FREQ = "80m"
FLASH_SIZE = "16MB"
BAUD_OPTIONS = ["921600", "460800", "230400", "115200"]

# El target de esptool, la frecuencia de flash y los offsets dependen del chip
# Y de la tabla de particiones de cada proyecto (partition_table.csv), por eso
# las direcciones son editables en la UI: esto son solo valores por defecto.
CHIP_PROFILES = {
    "ESP32-S3": {
        "esptool_chip": "esp32s3", "flash_freq": "80m",
        "offsets": {"bootloader": "0x0", "partition_table": "0x8000", "app": "0x10000", "ota_data_initial": ""},
    },
    "ESP32-P4": {
        "esptool_chip": "esp32p4", "flash_freq": "40m",
        "offsets": {"bootloader": "0x2000", "partition_table": "0x10000", "app": "0x20000", "ota_data_initial": "0x18000"},
    },
    "ESP32": {
        "esptool_chip": "esp32", "flash_freq": "40m",
        "offsets": {"bootloader": "0x1000", "partition_table": "0x8000", "app": "0x10000", "ota_data_initial": ""},
    },
    "ESP32-S2": {
        "esptool_chip": "esp32s2", "flash_freq": "40m",
        "offsets": {"bootloader": "0x1000", "partition_table": "0x8000", "app": "0x10000", "ota_data_initial": ""},
    },
    "ESP32-C3": {
        "esptool_chip": "esp32c3", "flash_freq": "40m",
        "offsets": {"bootloader": "0x0", "partition_table": "0x8000", "app": "0x10000", "ota_data_initial": ""},
    },
    "ESP32-C6": {
        "esptool_chip": "esp32c6", "flash_freq": "40m",
        "offsets": {"bootloader": "0x0", "partition_table": "0x8000", "app": "0x10000", "ota_data_initial": ""},
    },
}
CHIP_OPTIONS = list(CHIP_PROFILES.keys())
DEFAULT_CHIP = "ESP32-S3"

# Paleta estilo Visual Studio Code (Dark+)
COL_BG = "#1e1e1e"          # fondo tipo editor
COL_PANEL = "#252526"        # fondo tipo sidebar
COL_CARD = "#2d2d30"         # fondo tipo panel/tarjeta
COL_INPUT = "#3c3c3c"        # fondo de inputs/combos
COL_BORDER = "#3c3c3c"
COL_ACCENT = "#007acc"       # azul insignia de VS Code
COL_ACCENT_HOVER = "#1177bb"
COL_ACCENT_DIM = "#094771"   # azul de selección
COL_TEXT = "#d4d4d4"
COL_TEXT_MUTED = "#858585"
COL_OK = "#89d185"
COL_DANGER = "#f14c4c"
COL_WARN = "#cca700"

UI_FONT = "Segoe UI"
MONO_FONT = "Consolas"


def _list_bin_files(folder):
    """Lista .bin en 'folder' y en sus subcarpetas de primer nivel.

    ESP-IDF suele dejar el build asi: build/app.bin, build/ota_data_initial.bin,
    build/bootloader/bootloader.bin, build/partition_table/partition-table.bin.
    Buscar un nivel de subcarpetas permite apuntar directo a esa carpeta build/
    sin tener que copiar los .bin sueltos a mano.
    """
    found = []
    try:
        entries = list(os.scandir(folder))
    except OSError:
        return found

    for entry in entries:
        if entry.is_file() and entry.name.lower().endswith(".bin"):
            found.append(entry.path)
        elif entry.is_dir():
            try:
                for sub in os.scandir(entry.path):
                    if sub.is_file() and sub.name.lower().endswith(".bin"):
                        found.append(sub.path)
            except OSError:
                pass

    return sorted(found)


def detect_bin_files(folder):
    """Busca en 'folder' (y un nivel de subcarpetas) los .bin y los clasifica por rol.

    - bootloader: nombre contiene 'bootloader'
    - partition table: nombre contiene 'partition'
    - ota_data_initial: nombre contiene 'ota_data_initial' u 'otadata' (opcional, solo en proyectos con OTA)
    - app: el .bin restante
    """
    bin_paths = _list_bin_files(folder)

    def find_by_keyword(*keywords):
        return next(
            (p for p in bin_paths if any(k in os.path.basename(p).lower() for k in keywords)),
            None,
        )

    bootloader = find_by_keyword("bootloader")
    partition_table = find_by_keyword("partition")
    ota_data_initial = find_by_keyword("ota_data_initial", "otadata")

    remaining = [p for p in bin_paths if p not in (bootloader, partition_table, ota_data_initial)]
    app = remaining[0] if remaining else None

    return {
        "bootloader": bootloader,
        "partition_table": partition_table,
        "ota_data_initial": ota_data_initial,
        "app": app,
        "extra": remaining[1:],
        "all_found": [os.path.relpath(p, folder) for p in bin_paths],
    }


class QueueWriter:
    """Redirige stdout/stderr a una cola para mostrarlo en el widget de texto."""

    def __init__(self, q):
        self.q = q

    def write(self, text):
        if text:
            self.q.put(text)

    def flush(self):
        pass


# ── OTA por BLE: contrato con el firmware (ver ble_ota_gui.py / main2.c) ──────
OTA_SVC_UUID  = "1000454c-422d-4c52-5443-30394452905a"
OTA_DATA_UUID = "1100454c-422d-4c52-5443-30394452905a"
OTA_CTRL_UUID = "1200454c-422d-4c52-5443-30394452905a"
BLE_NAME_HINT = "RD90PRO-BLE"

OTA_CMD_START, OTA_CMD_END, OTA_CMD_ABORT = 0x01, 0x02, 0x03
OTA_ST_READY, OTA_ST_DONE, OTA_ST_ERROR, OTA_ST_PROG = 0x10, 0x11, 0x12, 0x13


class BleOtaWorker:
    """Loop asyncio en un hilo aparte para hablar BLE con bleak. La UI le encarga
    corrutinas y recibe eventos (tuplas) por una cola, igual que el log."""

    def __init__(self, ui_queue):
        self.q = ui_queue
        self.client = None
        self._abort = False
        self._ready = None
        self._done = None
        self._error_ev = None
        self._err_code = None
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def log(self, msg):
        self.q.put(("log", msg))

    def request_abort(self):
        self._abort = True

    # ── Escaneo ──────────────────────────────────────────────────────────────
    async def scan(self, timeout=6.0):
        self.log(f"BLE: escaneando {timeout:.0f} s...")
        try:
            devs = await BleakScanner.discover(timeout=timeout)
        except Exception as e:
            self.log(f"BLE: error al escanear: {e}")
            self.q.put(("devices", []))
            return
        items = [(d.name or "(sin nombre)", d.address) for d in devs]
        items.sort(key=lambda x: (BLE_NAME_HINT.lower() not in (x[0] or "").lower(), x[0].lower()))
        self.q.put(("devices", items))
        self.log(f"BLE: {len(items)} dispositivos encontrados.")

    # ── Conexion ─────────────────────────────────────────────────────────────
    def _on_disconnect(self, _client):
        self.q.put(("connected", False))
        self.log("BLE: desconectado.")

    async def connect(self, address):
        try:
            self.client = BleakClient(address, disconnected_callback=self._on_disconnect)
            await self.client.connect()
            self.q.put(("connected", True))
            self.log(f"BLE: conectado a {address} (MTU={getattr(self.client, 'mtu_size', '?')})")
        except Exception as e:
            self.client = None
            self.q.put(("connected", False))
            self.log(f"BLE: fallo al conectar: {e}")

    async def disconnect(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
        except Exception as e:
            self.log(f"BLE: error al desconectar: {e}")
        finally:
            self.q.put(("connected", False))

    # ── Notificaciones del robot ─────────────────────────────────────────────
    def _on_ctrl(self, _sender, data):
        if not data:
            return
        code = data[0]
        if code == OTA_ST_READY and self._ready:
            self._ready.set()
        elif code == OTA_ST_DONE and self._done:
            self._done.set()
        elif code == OTA_ST_ERROR:
            self._err_code = data[1] if len(data) > 1 else 0
            if self._error_ev:
                self._error_ev.set()
            self.log(f"BLE OTA: el robot reporto ERROR (code={self._err_code}).")
        elif code == OTA_ST_PROG and len(data) >= 5:
            n = struct.unpack("<I", data[1:5])[0]
            self.log(f"BLE OTA: robot confirmo {n} bytes.")

    async def _await(self, ev, timeout):
        t_ok = asyncio.create_task(ev.wait())
        t_err = asyncio.create_task(self._error_ev.wait())
        _, pending = await asyncio.wait({t_ok, t_err}, timeout=timeout,
                                        return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()
        if self._error_ev.is_set():
            raise RuntimeError(f"el robot reporto error (code={self._err_code})")
        if not ev.is_set():
            raise TimeoutError("sin respuesta del robot (timeout)")

    # ── Secuencia OTA completa (modo confiable = con respuesta) ───────────────
    async def do_ota(self, path, use_response=True):
        if not (self.client and self.client.is_connected):
            self.log("BLE: no estas conectado.")
            self.q.put(("ota_done", (False, "sin conexion")))
            return
        try:
            has_ota = any(c.uuid.lower() == OTA_DATA_UUID
                          for s in self.client.services for c in s.characteristics)
        except Exception:
            has_ota = True
        if not has_ota:
            self.log("BLE: este robot NO expone el servicio OTA (firmware viejo).")
            self.q.put(("ota_done", (False, "firmware sin servicio OTA")))
            return
        try:
            with open(path, "rb") as f:
                blob = f.read()
        except Exception as e:
            self.log(f"BLE: no se pudo leer el .bin: {e}")
            self.q.put(("ota_done", (False, "archivo ilegible")))
            return

        size = len(blob)
        crc = zlib.crc32(blob) & 0xFFFFFFFF
        self.log(f"BLE OTA: {os.path.basename(path)} | {size} bytes | CRC32=0x{crc:08x}")

        self._ready = asyncio.Event()
        self._done = asyncio.Event()
        self._error_ev = asyncio.Event()
        self._err_code = None
        self._abort = False

        try:
            await self.client.start_notify(OTA_CTRL_UUID, self._on_ctrl)

            await self.client.write_gatt_char(
                OTA_CTRL_UUID, bytes([OTA_CMD_START]) + struct.pack("<I", size), response=True)
            self.log("BLE OTA: START enviado (el robot borra flash, puede tardar)...")
            await self._await(self._ready, timeout=30)
            self.log("BLE OTA: robot listo. Transmitiendo...")

            mtu = getattr(self.client, "mtu_size", 23) or 23
            chunk = max(mtu - 3, 20)
            sent = 0
            i = 0
            t0 = time.time()
            last_ui = 0.0
            while sent < size:
                if self._abort:
                    await self.client.write_gatt_char(OTA_CTRL_UUID, bytes([OTA_CMD_ABORT]), response=True)
                    self.log("BLE OTA: cancelado por el usuario.")
                    self.q.put(("ota_done", (False, "cancelado")))
                    await self._safe_stop_notify()
                    return
                part = blob[sent:sent + chunk]
                await self.client.write_gatt_char(OTA_DATA_UUID, part, response=use_response)
                sent += len(part)
                i += 1
                if not use_response and (i % 20 == 0):
                    await asyncio.sleep(0.003)
                now = time.time()
                if now - last_ui > 0.1 or sent >= size:
                    speed = sent / max(now - t0, 1e-3)
                    self.q.put(("progress", (sent, size, speed)))
                    last_ui = now

            await self.client.write_gatt_char(
                OTA_CTRL_UUID, bytes([OTA_CMD_END]) + struct.pack("<I", crc), response=True)
            self.log("BLE OTA: END enviado. Esperando confirmacion...")
            try:
                await self._await(self._done, timeout=20)
                self.log("BLE OTA: OK, completo. El robot se reinicia.")
                self.q.put(("ota_done", (True, "completo")))
            except Exception as e:
                self.log(f"BLE OTA: sin confirmacion clara ({e}). Si reinicio, probablemente OK.")
                self.q.put(("ota_done", (True, "fin - verifica la version")))
            await self._safe_stop_notify()
        except Exception as e:
            self.log(f"BLE OTA: error: {e}")
            self.q.put(("ota_done", (False, str(e))))
            await self._safe_stop_notify()

    async def _safe_stop_notify(self):
        try:
            if self.client and self.client.is_connected:
                await self.client.stop_notify(OTA_CTRL_UUID)
        except Exception:
            pass


class FlashApp:
    def __init__(self, root):
        self.root = root
        root.title("Flash ESP32 (esptool)")
        root.geometry("760x640")
        root.minsize(600, 480)
        root.configure(fg_color=COL_BG)

        self.log_queue = queue.Queue()
        self.flashing = False
        self.bin_folder = SCRIPT_DIR

        # Estado del OTA por BLE
        self.ble_queue = queue.Queue()
        self.ble_worker = BleOtaWorker(self.ble_queue) if BLEAK_OK else None
        self.ble_connected = False
        self.ble_busy = False
        self.ble_t_start = None
        self.ble_dev_addr = {}      # etiqueta mostrada -> direccion BLE
        self.ble_bin_path = None    # .bin elegido a mano (si None, usa el [app] detectado)

        ui = ctk.CTkFont(family=UI_FONT, size=12)
        ui_bold = ctk.CTkFont(family=UI_FONT, size=12, weight="bold")
        mono = ctk.CTkFont(family=MONO_FONT, size=11)

        outer = ctk.CTkFrame(root, corner_radius=6, fg_color=COL_PANEL,
                              border_width=1, border_color=COL_BORDER)
        outer.pack(fill="both", expand=True, padx=10, pady=10)

        # Encabezado
        header = ctk.CTkFrame(outer, fg_color="transparent")
        header.pack(fill="x", padx=16, pady=(14, 6))
        ctk.CTkLabel(header, text="ESP32 Flash Tool", text_color=COL_TEXT,
                     font=ctk.CTkFont(family=UI_FONT, size=16, weight="bold")).pack(anchor="w")

        folder_row = ctk.CTkFrame(header, fg_color="transparent")
        folder_row.pack(fill="x", pady=(2, 0))
        self.folder_label = ctk.CTkLabel(folder_row, text=self.bin_folder, text_color=COL_TEXT_MUTED,
                                          font=mono, anchor="w")
        self.folder_label.pack(side="left", fill="x", expand=True)
        self._button(folder_row, "Cambiar carpeta", self.choose_folder, primary=False, width=130, height=22).pack(side="right")

        # Selector de modo: USB (flasheo por cable) / BLE (OTA inalambrico).
        # Cambia la interfaz mostrando solo lo del modo elegido; el LOG queda siempre.
        mode_row = ctk.CTkFrame(outer, fg_color="transparent")
        mode_row.pack(fill="x", padx=16, pady=(4, 2))
        ctk.CTkLabel(mode_row, text="Modo de conexión", text_color=COL_TEXT_MUTED, font=ui_bold).pack(side="left")
        self.mode_seg = ctk.CTkSegmentedButton(
            mode_row, values=["USB", "BLE"], command=self._set_mode,
            fg_color=COL_INPUT, selected_color=COL_ACCENT, selected_hover_color=COL_ACCENT_HOVER,
            unselected_color=COL_CARD, unselected_hover_color="#45494e", text_color=COL_TEXT,
            font=ctk.CTkFont(family=UI_FONT, size=12, weight="bold"))
        self.mode_seg.set("USB")
        self.mode_seg.pack(side="left", padx=10)

        # Card: conexion (USB)
        conn_card = self._card(outer, "CONEXIÓN")
        self.conn_card = conn_card

        row0 = ctk.CTkFrame(conn_card, fg_color="transparent")
        row0.pack(fill="x", padx=12, pady=(8, 6))
        ctk.CTkLabel(row0, text="Chip", text_color=COL_TEXT_MUTED, font=ui, width=70, anchor="w").pack(side="left")
        self.chip_combo = self._combobox(row0, width=220, values=CHIP_OPTIONS)
        self.chip_combo.set(DEFAULT_CHIP)
        self.chip_combo.configure(command=lambda _choice: self.refresh_files())
        self.chip_combo.pack(side="left", padx=(4, 8))

        row1 = ctk.CTkFrame(conn_card, fg_color="transparent")
        row1.pack(fill="x", padx=12, pady=(0, 6))
        ctk.CTkLabel(row1, text="Puerto", text_color=COL_TEXT_MUTED, font=ui, width=70, anchor="w").pack(side="left")
        self.port_combo = self._combobox(row1, width=220)
        self.port_combo.pack(side="left", padx=(4, 8))
        self._button(row1, "Actualizar", self.refresh_ports, primary=False, width=110).pack(side="left")

        row2 = ctk.CTkFrame(conn_card, fg_color="transparent")
        row2.pack(fill="x", padx=12, pady=(0, 10))
        ctk.CTkLabel(row2, text="Baudrate", text_color=COL_TEXT_MUTED, font=ui, width=70, anchor="w").pack(side="left")
        self.baud_combo = self._combobox(row2, width=220, values=BAUD_OPTIONS)
        self.baud_combo.set(BAUD_OPTIONS[0])
        self.baud_combo.pack(side="left", padx=(4, 8))

        # Card: archivos
        files_card = self._card(outer, "ARCHIVOS DETECTADOS", "Re-escanear", self.refresh_files)
        self.files_card = files_card

        self.files_list = ctk.CTkFrame(files_card, fg_color="transparent")
        self.files_list.pack(fill="x", padx=12, pady=(4, 6))
        self.files = {}
        self.refresh_files()

        # Flashear + progreso
        action = ctk.CTkFrame(outer, fg_color="transparent")
        self.action_frame = action
        self.flash_btn = self._button(action, "Flashear", self.start_flash, primary=True, width=140, height=32)
        self.flash_btn.pack(side="left")
        self.progress = ctk.CTkProgressBar(action, mode="indeterminate", height=4, corner_radius=0,
                                            fg_color=COL_INPUT, progress_color=COL_ACCENT,
                                            border_width=0)
        self.progress.pack(side="left", fill="x", expand=True, padx=(12, 0))
        self.progress.set(0)

        # Card: OTA por BLE (actualizacion inalambrica, usa el .bin [app] detectado)
        ble_card = self._card(outer, "ACTUALIZAR POR BLE (OTA)")
        self.ble_card = ble_card

        brow0 = ctk.CTkFrame(ble_card, fg_color="transparent")
        brow0.pack(fill="x", padx=12, pady=(8, 6))
        self.ble_scan_btn = self._button(brow0, "Buscar", self.ble_scan, primary=False, width=90)
        self.ble_scan_btn.pack(side="left")
        self.ble_dev_combo = self._combobox(brow0, width=300)
        self.ble_dev_combo.pack(side="left", padx=(6, 8))

        brow1 = ctk.CTkFrame(ble_card, fg_color="transparent")
        brow1.pack(fill="x", padx=12, pady=(0, 6))
        self.ble_conn_btn = self._button(brow1, "Conectar", self.ble_connect, primary=False, width=110)
        self.ble_conn_btn.pack(side="left")
        self.ble_state = ctk.CTkLabel(brow1, text="● Desconectado", text_color=COL_DANGER, font=ui_bold)
        self.ble_state.pack(side="left", padx=10)

        browf = ctk.CTkFrame(ble_card, fg_color="transparent")
        browf.pack(fill="x", padx=12, pady=(0, 6))
        self.ble_pick_btn = self._button(browf, "Seleccionar .bin", self.ble_pick_file, primary=False, width=140)
        self.ble_pick_btn.pack(side="left")

        brow2 = ctk.CTkFrame(ble_card, fg_color="transparent")
        brow2.pack(fill="x", padx=12, pady=(0, 6))
        self.ble_update_btn = self._button(brow2, "Actualizar por BLE", self.ble_update, primary=True, width=160, height=30)
        self.ble_update_btn.pack(side="left")
        self.ble_cancel_btn = self._button(brow2, "Cancelar", self.ble_cancel, primary=False, width=90)
        self.ble_cancel_btn.pack(side="left", padx=(8, 0))
        self.ble_progress = ctk.CTkProgressBar(brow2, mode="determinate", height=10, corner_radius=0,
                                               fg_color=COL_INPUT, progress_color=COL_OK, border_width=0)
        self.ble_progress.pack(side="left", fill="x", expand=True, padx=(12, 0))
        self.ble_progress.set(0)

        brow3 = ctk.CTkFrame(ble_card, fg_color="transparent")
        brow3.pack(fill="x", padx=12, pady=(0, 10))
        self.ble_info = ctk.CTkLabel(brow3, text="Envía el .bin marcado como [app] arriba", text_color=COL_TEXT_MUTED, font=mono)
        self.ble_info.pack(side="left")
        self.ble_time = ctk.CTkLabel(brow3, text="Tiempo: 0:00", text_color=COL_WARN, font=ui_bold)
        self.ble_time.pack(side="right")

        # Log (visible en ambos modos)
        self.log_label_widget = ctk.CTkLabel(outer, text="LOG", text_color=COL_TEXT_MUTED, font=ui_bold)
        self.log_label_widget.pack(anchor="w", padx=16, pady=(8, 2))
        self.log_text = ctk.CTkTextbox(
            outer, corner_radius=4, fg_color=COL_BG, text_color=COL_TEXT,
            border_width=1, border_color=COL_BORDER, font=(MONO_FONT, 10),
            wrap="word",
        )
        self.log_text.pack(fill="both", expand=True, padx=16, pady=(0, 14))
        self.log_text.configure(state="disabled")

        self.refresh_ports()

        if esptool is None:
            self.log("Falta el paquete 'esptool'. Instalalo con: pip install esptool\n")
            self.flash_btn.configure(state="disabled")

        if not BLEAK_OK:
            self.log("Para OTA por BLE instala: pip install bleak\n")
        self._ble_refresh_buttons()
        self._set_mode("USB")

        self.root.after(100, self.poll_log_queue)

    # ---------- helpers de estilo ----------

    def _card(self, master, title, action_text=None, action_command=None):
        card = ctk.CTkFrame(master, corner_radius=4, fg_color=COL_CARD,
                             border_width=1, border_color=COL_BORDER)
        title_row = ctk.CTkFrame(card, fg_color="transparent")
        title_row.pack(fill="x", padx=12, pady=(8, 0))
        ctk.CTkLabel(title_row, text=title, text_color=COL_TEXT_MUTED,
                     font=ctk.CTkFont(family=UI_FONT, size=11, weight="bold")).pack(side="left")
        if action_text:
            self._button(title_row, action_text, action_command, primary=False, width=100, height=22).pack(side="right")
        return card

    def _button(self, master, text, command, primary=True, width=100, height=30):
        if primary:
            return ctk.CTkButton(
                master, text=text, command=command, width=width, height=height,
                corner_radius=3, fg_color=COL_ACCENT, hover_color=COL_ACCENT_HOVER,
                text_color="#ffffff", font=ctk.CTkFont(family=UI_FONT, size=12, weight="bold"),
            )
        return ctk.CTkButton(
            master, text=text, command=command, width=width, height=height,
            corner_radius=3, fg_color=COL_INPUT, hover_color="#45494e",
            border_width=1, border_color=COL_BORDER, text_color=COL_TEXT,
            font=ctk.CTkFont(family=UI_FONT, size=12),
        )

    def _combobox(self, master, width=200, values=None):
        return ctk.CTkComboBox(
            master, width=width, values=values or [], state="readonly",
            corner_radius=3, fg_color=COL_INPUT, border_color=COL_BORDER, border_width=1,
            button_color=COL_INPUT, button_hover_color="#45494e",
            text_color=COL_TEXT, dropdown_fg_color=COL_CARD,
            dropdown_hover_color=COL_ACCENT_DIM, dropdown_text_color=COL_TEXT,
            font=ctk.CTkFont(family=UI_FONT, size=12),
        )

    # ---------- logica ----------

    def _set_mode(self, mode):
        """Muestra solo las tarjetas del modo elegido (USB o BLE). El LOG queda fijo."""
        self.mode = mode
        for w in (self.conn_card, self.files_card, self.action_frame, self.ble_card):
            w.pack_forget()
        if mode == "USB":
            self.conn_card.pack(fill="x", padx=16, pady=6, before=self.log_label_widget)
            self.files_card.pack(fill="x", padx=16, pady=4, before=self.log_label_widget)
            self.action_frame.pack(fill="x", padx=16, pady=(4, 6), before=self.log_label_widget)
        else:
            self.ble_card.pack(fill="x", padx=16, pady=6, before=self.log_label_widget)
            self._ble_update_info()

    def choose_folder(self):
        chosen = filedialog.askdirectory(initialdir=self.bin_folder, title="Carpeta con los .bin")
        if chosen:
            self.bin_folder = chosen
            self.folder_label.configure(text=self.bin_folder)
            self.refresh_files()

    def refresh_files(self):
        for widget in self.files_list.winfo_children():
            widget.destroy()

        self.files = detect_bin_files(self.bin_folder)
        mono = ctk.CTkFont(family=MONO_FONT, size=11)

        chip_profile = CHIP_PROFILES.get(self.chip_combo.get(), CHIP_PROFILES[DEFAULT_CHIP])
        default_offsets = chip_profile["offsets"]

        self.offset_entries = {}

        role_labels = [
            ("bootloader", "bootloader"),
            ("partition_table", "partition table"),
            ("app", "app"),
            ("ota_data_initial", "ota data"),
        ]
        for role, label in role_labels:
            path = self.files.get(role)
            if role == "ota_data_initial" and path is None:
                continue  # archivo opcional: solo se muestra si existe

            row = ctk.CTkFrame(self.files_list, fg_color="transparent", height=22)
            row.pack(fill="x", pady=1)

            entry = ctk.CTkEntry(
                row, width=72, height=20, corner_radius=3,
                fg_color=COL_INPUT, border_color=COL_BORDER, border_width=1,
                text_color=COL_TEXT, font=mono, justify="center",
            )
            entry.insert(0, default_offsets.get(role, "") or "")
            entry.pack(side="left")
            self.offset_entries[role] = entry

            ok = path is not None
            status = "OK" if ok else "FALTA"
            color = COL_OK if ok else COL_DANGER
            name = os.path.relpath(path, self.bin_folder) if path else "(no encontrado)"
            ctk.CTkLabel(row, text=f"[{label}]", text_color=COL_TEXT_MUTED, font=mono, anchor="w", width=110, height=18).pack(side="left", padx=(6, 0))
            ctk.CTkLabel(row, text=name, text_color=COL_TEXT, font=mono, anchor="w", height=18).pack(side="left", fill="x", expand=True, padx=(4, 0))
            ctk.CTkLabel(row, text=status, text_color=color, height=18,
                         font=ctk.CTkFont(family=MONO_FONT, size=11, weight="bold")).pack(side="right")

        if self.files["extra"]:
            extra_names = ", ".join(os.path.relpath(p, self.bin_folder) for p in self.files["extra"])
            ctk.CTkLabel(self.files_list, text=f"Aviso: .bin adicionales ignorados: {extra_names}",
                         text_color=COL_WARN, font=mono, height=16).pack(anchor="w", pady=(2, 0))

        if not self.files["all_found"]:
            ctk.CTkLabel(self.files_list, text="No se encontraron archivos .bin en esta carpeta.",
                         text_color=COL_DANGER, font=mono, height=16).pack(anchor="w", pady=(2, 0))

        if hasattr(self, "ble_update_btn"):
            self._ble_refresh_buttons()
        self._ble_update_info()

    def refresh_ports(self):
        ports = []
        if serial is not None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.configure(values=ports)
        self.port_combo.set(ports[0] if ports else "")

    def log(self, text):
        text = ANSI_ESCAPE_RE.sub("", text)
        text = text.replace("\r\n", "\n").replace("\r", "\n")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def poll_log_queue(self):
        try:
            while True:
                text = self.log_queue.get_nowait()
                self.log(text)
        except queue.Empty:
            pass

        try:
            while True:
                kind, payload = self.ble_queue.get_nowait()
                self._handle_ble_event(kind, payload)
        except queue.Empty:
            pass

        if self.ble_busy and self.ble_t_start:
            self.ble_time.configure(text=f"Tiempo: {self._mmss(time.time() - self.ble_t_start)}")

        self.root.after(100, self.poll_log_queue)

    def start_flash(self):
        if self.flashing:
            return

        port = self.port_combo.get()
        if not port:
            self.log("Seleccioná un puerto antes de flashear.\n")
            return

        missing = [role for role in ("bootloader", "partition_table", "app") if not self.files.get(role)]
        if missing:
            self.log(f"Faltan archivos requeridos en la carpeta: {', '.join(missing)}\n")
            return

        offsets = {role: entry.get().strip() for role, entry in self.offset_entries.items()}
        missing_offsets = [role for role in ("bootloader", "partition_table", "app") if not offsets.get(role)]
        if missing_offsets:
            self.log(f"Faltan direcciones (offset) para: {', '.join(missing_offsets)}\n")
            return

        self.flashing = True
        self.flash_btn.configure(state="disabled")
        self.progress.start()
        chip_name = self.chip_combo.get()
        self.log(f"\n--- Iniciando flasheo de {chip_name} en {port} @ {self.baud_combo.get()} baudios ---\n")

        thread = threading.Thread(
            target=self.run_flash, args=(port, self.baud_combo.get(), chip_name, offsets), daemon=True
        )
        thread.start()

    def run_flash(self, port, baud, chip_name, offsets):
        profile = CHIP_PROFILES.get(chip_name, CHIP_PROFILES[DEFAULT_CHIP])
        cmd = [
            "--chip", profile["esptool_chip"],
            "--port", port,
            "--baud", baud,
            "--before", "default_reset",
            "--after", "hard_reset",
            "write_flash",
            "--flash_mode", FLASH_MODE,
            "--flash_freq", profile["flash_freq"],
            "--flash_size", FLASH_SIZE,
        ]
        for role in ("bootloader", "partition_table", "app", "ota_data_initial"):
            path = self.files.get(role)
            offset = offsets.get(role)
            if path and offset:
                cmd += [offset, path]

        writer = QueueWriter(self.log_queue)
        old_stdout, old_stderr = sys.stdout, sys.stderr
        sys.stdout, sys.stderr = writer, writer
        try:
            esptool.main(cmd)
            self.log_queue.put("\n--- Flasheo terminado con exito ---\n")
        except SystemExit as e:
            if e.code not in (0, None):
                self.log_queue.put(f"\n--- esptool salio con codigo {e.code} ---\n")
            else:
                self.log_queue.put("\n--- Flasheo terminado con exito ---\n")
        except Exception as e:
            self.log_queue.put(f"\n--- ERROR: {e} ---\n")
        finally:
            sys.stdout, sys.stderr = old_stdout, old_stderr
            self.root.after(0, self.finish_flash)

    def finish_flash(self):
        self.flashing = False
        self.flash_btn.configure(state="normal")
        self.progress.stop()
        self.progress.set(0)

    # ---------- OTA por BLE ----------

    @staticmethod
    def _mmss(secs):
        secs = int(secs)
        return f"{secs // 60}:{secs % 60:02d}"

    def ble_scan(self):
        if self.ble_worker is None or self.ble_busy:
            return
        self._ble_set_busy(True)
        self.ble_worker.submit(self.ble_worker.scan())

    def ble_connect(self):
        if self.ble_worker is None:
            return
        if self.ble_connected:
            self.ble_worker.submit(self.ble_worker.disconnect())
            return
        label = self.ble_dev_combo.get()
        addr = self.ble_dev_addr.get(label)
        if not addr:
            self.log("BLE: elegí un dispositivo en la lista primero.\n")
            return
        self._ble_set_busy(True)
        self.ble_worker.submit(self.ble_worker.connect(addr))

    def ble_pick_file(self):
        if self.ble_busy:
            return
        path = filedialog.askopenfilename(
            title="Elige el firmware .bin para enviar por BLE",
            filetypes=[("Firmware ESP32 (*.bin)", "*.bin"), ("Todos", "*.*")],
            initialdir=self.bin_folder)
        if path:
            self.ble_bin_path = path
            self.log(f"BLE: archivo elegido: {path}\n")
        self._ble_update_info()
        self._ble_refresh_buttons()

    def _ble_update_info(self):
        if not hasattr(self, "ble_info"):
            return
        path = self.ble_bin_path or self.files.get("app")
        if path:
            src = "elegido" if self.ble_bin_path else "auto [app]"
            self.ble_info.configure(text=f"Enviará: {os.path.basename(path)}  ({src})")
        else:
            self.ble_info.configure(text="Elige un .bin (o apunta la carpeta al build/)")

    def ble_update(self):
        if self.ble_worker is None or self.ble_busy or not self.ble_connected:
            return
        app_path = self.ble_bin_path or self.files.get("app")
        if not app_path:
            self.log("BLE: elige un .bin con 'Seleccionar .bin' (o apunta la carpeta al build/).\n")
            return
        self.ble_t_start = time.time()
        self.ble_time.configure(text="Tiempo: 0:00")
        self.ble_progress.set(0)
        self._ble_set_busy(True)
        self.log(f"\n--- OTA por BLE: {os.path.basename(app_path)} ---\n")
        self.ble_worker.submit(self.ble_worker.do_ota(app_path, True))

    def ble_cancel(self):
        if self.ble_worker and self.ble_busy:
            self.ble_worker.request_abort()
            self.log("BLE: cancelando...\n")

    def _ble_set_busy(self, busy):
        self.ble_busy = busy
        self._ble_refresh_buttons()

    def _ble_refresh_buttons(self):
        btns = (getattr(self, "ble_scan_btn", None), getattr(self, "ble_conn_btn", None),
                getattr(self, "ble_pick_btn", None), getattr(self, "ble_update_btn", None),
                getattr(self, "ble_cancel_btn", None))
        if self.ble_worker is None:
            for b in btns:
                if b:
                    b.configure(state="disabled")
            return
        busy = self.ble_busy
        self.ble_scan_btn.configure(state="disabled" if busy else "normal")
        self.ble_conn_btn.configure(state="normal" if (not busy or self.ble_connected) else "disabled",
                                    text="Desconectar" if self.ble_connected else "Conectar")
        self.ble_pick_btn.configure(state="disabled" if busy else "normal")
        can_upd = self.ble_connected and (not busy) and bool(self.ble_bin_path or self.files.get("app"))
        self.ble_update_btn.configure(state="normal" if can_upd else "disabled")
        self.ble_cancel_btn.configure(state="normal" if busy else "disabled")

    def _handle_ble_event(self, kind, payload):
        if kind == "log":
            self.log(str(payload) + "\n")
        elif kind == "devices":
            self.ble_dev_addr = {f"{n}   [{a}]": a for (n, a) in payload}
            labels = list(self.ble_dev_addr.keys())
            self.ble_dev_combo.configure(values=labels)
            if labels:
                self.ble_dev_combo.set(labels[0])
            self._ble_set_busy(False)
        elif kind == "connected":
            self.ble_connected = bool(payload)
            self.ble_state.configure(
                text="● Conectado" if self.ble_connected else "● Desconectado",
                text_color=COL_OK if self.ble_connected else COL_DANGER)
            self._ble_set_busy(False)
        elif kind == "progress":
            sent, total, speed = payload
            self.ble_progress.set((sent / total) if total else 0)
            pct = (sent / total * 100) if total else 0
            self.ble_info.configure(
                text=f"{pct:.1f}%  ·  {speed/1024:.1f} KB/s  ·  {sent}/{total} B")
        elif kind == "ota_done":
            ok, msg = payload
            self._ble_set_busy(False)
            total = (time.time() - self.ble_t_start) if self.ble_t_start else 0
            self.ble_time.configure(text=f"Tiempo: {self._mmss(total)}")
            if ok:
                self.ble_progress.set(1.0)
                self.log(f"--- OTA por BLE OK: {msg}  (tiempo {self._mmss(total)}) ---\n")
            else:
                self.log(f"--- OTA por BLE FALLO: {msg}  (tiempo {self._mmss(total)}) ---\n")
            self.ble_t_start = None


def main():
    if ctk is None:
        print("Falta el paquete 'customtkinter'. Instalalo con:\n    pip install customtkinter")
        sys.exit(1)

    ctk.set_appearance_mode("dark")
    root = ctk.CTk()
    FlashApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
