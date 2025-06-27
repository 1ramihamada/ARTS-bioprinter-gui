#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import wx
import os
import serial
import socket
import serial.tools.list_ports
import time
import threading
import math
import logging

import matplotlib
matplotlib.use("WXAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas

from PIL import Image

ASSETS_PATH = "/home/rami/Downloads/Prusa/assets"

USEFUL_GCODE_COMMANDS = [
    "M114   (Get current position)",
    "M503   (Report current settings)",
    "M105   (Read extruder hotend temp)",
    "M112   (Emergency stop)",
    "G28    (Home all axes)",
    "G90    (Absolute positioning)",
    "G91    (Relative positioning)",
    "G1 X10 Y5 F3000 (Move X+10, Y+5 at feed=3000)",
    "G92 X0 Y0 Z0 (Set the current position (0,0,0))",
    "M420 S1 (Activate mesh bed leveling)"
]

# We'll keep global x_coords, y_coords, z_coords, F_coords
x_coords = []
y_coords = []
z_coords = []
F_coords = []

def auto_find_nordson_ports(n=2, baud=115200, timeout=1.0):
    ports = []
    for p in serial.tools.list_ports.comports():
        try:
            s = serial.Serial(p.device, baud, timeout=timeout)
            time.sleep(1)
            s.write(b'\x05')          # ENQ
            time.sleep(0.1)
            if s.read(1) == b'\x06':  # ACK
                ports.append(p.device)
            s.close()
        except:
            pass
        if len(ports) >= n:
            break
    return ports

def run_both(f1, f2):
    t1 = threading.Thread(target=f1)
    t2 = threading.Thread(target=f2)
    t1.start()
    t2.start()
    t1.join()
    t2.join()

class MultiDispenserController:
    def __init__(self, mode='single', port=None):
        self.mode = mode
        if mode == 'single':
            self.new = DispenserController(port) if port else DispenserController()
            self.controllers = [self.new]
        else:
            ports = auto_find_nordson_ports(2)
            if len(ports) < 2:
                raise RuntimeError(f"Need 2 ports, found {ports}")
            self.new = DispenserController(ports[0])
            self.old = DispenserController(ports[1])
            self.controllers = [self.new, self.old]

    def _parallel(self, fn, *args):
        t = threading.Thread(target=fn, args=args)
        t.daemon = True
        t.start()
        return t

    def dispenser_callback(self, cmd):
        # launch one thread per controller, then wait for both
        threads = [ self._parallel(d.dispenser_callback, cmd)
                    for d in self.controllers ]
        for t in threads: t.join()

    def emergency_stop(self):
        threads = [ self._parallel(d.emergency_stop)
                    for d in self.controllers ]
        for t in threads: t.join()

# --- Dispenser Controller Code (unchanged) ---
class DispenserController:
    def __init__(self, port: str = None):
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger('DispenserController')
        self.ser = None
        self.port = port
        self.connect()
        self.is_timed_mode = True
        self.logger.debug('Dispenser Controller Started')

    def connect(self):
        # if a specific port was requested, try it first
        if self.port:
            try:
                self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
                self.logger.info(f'Connected to {self.port}')
                time.sleep(2)
                return
            except Exception as e:
                self.logger.error(f"Error opening {self.port}: {e}")
                # fall back to scanning

        available_ports = list(serial.tools.list_ports.comports())
        if not available_ports:
            self.logger.error("No serial ports found. Running in simulation mode.")
            return
        self.logger.info("Available serial ports:")
        for port in available_ports:
            self.logger.info(f"  {port.device}: {port.description}")
        for port in available_ports:
            try:
                self.ser = serial.Serial(
                    port.device,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1
                )
                self.logger.debug(f'Connected to {port.device}')  # Debug-level message, won't show at WARNING level
                time.sleep(2)
                return
            except serial.SerialException as e:
#               self.logger.error(f"Error connecting to {port.device}: {e}")
                continue
        if not self.ser:
            self.logger.error("Could not connect to any port. Running in simulation mode.")

    def calculate_checksum_ascii(self, data_str):
        checksum = (0 - sum(data_str.encode('ascii'))) & 0xFF
        return checksum

    def send_command(self, command_code, data):
        if self.ser:
            try:

                # clear out any stale data
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

                # 1) send ENQ
                self.ser.write(b'\x05')
                time.sleep(0.1)
                ack_response = self.ser.read(1)
                if ack_response != b'\x06':
                    self.logger.warning("Did not receive ACK after ENQ.")
                    # return
                    # ↑ do NOT return here; we’ll still send the packet

                STX = b'\x02'
                ETX = b'\x03'

                command_str = command_code
                data_str = data if data else ''

                length = len(command_str) + len(data_str)
                length_hex = f"{length:02X}"
                length_str = length_hex

                checksum_input_str = length_str + command_str + data_str
                checksum_value = self.calculate_checksum_ascii(checksum_input_str)
                checksum_str = f"{checksum_value:02X}"

                packet = (
                    STX +
                    length_str.encode('ascii') +
                    command_str.encode('ascii') +
                    data_str.encode('ascii') +
                    checksum_str.encode('ascii') +
                    ETX
                )
                self.ser.write(packet)
                time.sleep(0.1)
                response = self.ser.read(100)
                if response:
                    self.check_response(response)
                else:
                    self.logger.warning("No response from device after sending command.")
            except serial.SerialException as e:
                self.logger.error(f"Error communicating with device: {e}")
        else:
            self.logger.info(f'Simulated command: {command_code} {data}')
            self.logger.info('Simulated successful response')

    def read_pressure(self) -> float:
        """
        Query the dispenser for its current pressure (BAR).
        Returns the pressure as a float, or raises on parse error.
        """
        if not self.ser:
            raise RuntimeError("No serial connection for reading pressure")
        # clear any old data
        self.ser.reset_input_buffer()
        # send the ASCII query (model-dependent; yours supports "PR?\r\n")
        self.ser.write(b"PR?\r\n")
        time.sleep(0.1)
        # read one line of response, e.g. "PR=2.10"
        line = self.ser.readline().decode("ascii", errors="ignore").strip()
        if not line.startswith("PR="):
            raise ValueError(f"Unexpected pressure reply: {line!r}")
        try:
            return float(line.split("=",1)[1])
        except ValueError as e:
            raise ValueError(f"Could not parse pressure from {line!r}") from e

    def read_vacuum(self) -> float:
        """
        Query the dispenser for its current vacuum (in inHg).
        Returns the vacuum as a float, or raises on parse error.
        """
        if not self.ser:
            raise RuntimeError("No serial connection for reading vacuum")
        self.ser.reset_input_buffer()
        self.ser.write(b"VU?\r\n")
        time.sleep(0.1)
        line = self.ser.readline().decode("ascii", errors="ignore").strip()
        if not line.startswith("VU="):
            raise ValueError(f"Unexpected vacuum reply: {line!r}")
        try:
            return float(line.split("=",1)[1])
        except ValueError as e:
            raise ValueError(f"Could not parse vacuum from {line!r}") from e

    def dispenser_callback(self, command_str):
        if command_str == 'start':
            self.send_command('DI  ', '')
        elif command_str == 'stop':
            self.send_command('DI  ', '')
        elif command_str.startswith('pressure '):
            try:
                pressure_value = float(command_str.split(' ')[1])
                if 0.0 <= pressure_value <= 650.0:
                    formatted_pressure = f"{int(pressure_value * 1000):04d}"
                    self.send_command('PS  ', formatted_pressure)
                else:
                    self.logger.error(f'Invalid pressure: {pressure_value}')
            except (IndexError, ValueError):
                self.logger.error('Invalid pressure command.')
        elif command_str.startswith('vacuum '):
            try:
                vacuum_value = float(command_str.split(' ')[1])
                if 0.0 <= vacuum_value <= 300.0:
                    formatted_vacuum = f"{int(vacuum_value * 10):04d}"
                    self.send_command('VS  ', formatted_vacuum)
                else:
                    self.logger.error(f'Invalid vacuum: {vacuum_value}')
            except (IndexError, ValueError):
                self.logger.error('Invalid vacuum command.')
        else:
            self.logger.error(f'Unknown command: {command_str}')

    def check_response(self, response):
        index = 0
        while index < len(response):
            if response[index] == 0x02:  # STX
                index += 1
                if index + 1 >= len(response):
                    break
                length_str = response[index:index+2].decode('ascii', errors='ignore')
                index += 2
                try:
                    length = int(length_str, 16)
                except ValueError:
                    break
                if index + length + 2 > len(response):
                    break
                command_and_data = response[index:index+length].decode('ascii', errors='ignore')
                index += length
                checksum_str = response[index:index+2].decode('ascii', errors='ignore')
                index += 2
                if index >= len(response) or response[index] != 0x03:
                    break
                index += 1  # ETX
                checksum_input_str = length_str + command_and_data
                calculated_checksum = self.calculate_checksum_ascii(checksum_input_str)
                try:
                    received_checksum = int(checksum_str, 16)
                except ValueError:
                    continue
                if received_checksum != calculated_checksum:
                    continue
                command_code = command_and_data[:2]
                if command_code == 'A0':
                    self.logger.info('Received A0 success')
                    if self.ser:
                        self.ser.write(b'\x04')
                        time.sleep(0.1)
                elif command_code == 'A2':
                    self.logger.info('Received A2 failure')
                    if self.ser:
                        self.ser.write(b'\x04')
                        time.sleep(0.1)
                else:
                    self.logger.info(f'Received response: {command_and_data}')
            else:
                index += 1

    def emergency_stop(self):
        """
        Perform an emergency stop for the extruder and printer.
        - Sends the 'DO  ' command to the dispenser to stop dispensing.
        - Sends the 'M112' command to the printer for an emergency stop.
        """
        self.logger.info("Emergency Stop triggered!")
        try:
            # Stop the extruder
            if self.ser:
                self.send_command('DO  ', '')  # RS-232 command to stop dispensing
                self.logger.info("Dispenser stopped successfully.")
            else:
                self.logger.warning("Dispenser serial connection not available. Running in simulation mode.")

            # Stop the printer
            if hasattr(self, 'printer_serial') and self.printer_serial:
                self.printer_serial.write(b"M112\n")  # G-code for printer emergency stop
                self.printer_serial.flush()
                self.logger.info("Printer emergency stop command (M112) issued successfully.")
            else:
                self.logger.warning("Printer serial connection not available. Cannot send M112.")
        except Exception as e:
            self.logger.error(f"Error during emergency stop: {e}")


    def destroy(self):
        if self.ser:
            self.ser.close()
        self.logger.info('Dispenser Controller Stopped')

    def set_pressure(self, bar: float):
        if not (0.0 <= bar <= 650.0):
            raise ValueError("Pressure out of range")
        self.dispenser_callback(f"pressure {bar}")

    def set_vacuum(self, h2o: float):
        if not (0.0 <= h2o <= 300.0):
            raise ValueError("Vacuum out of range")
        self.dispenser_callback(f"vacuum {h2o}")

# -----------------------------------------------------------------------
# A separate top-level Frame that captures arrow keys to move the printer
# -----------------------------------------------------------------------
class AxisControlFrame(wx.Frame):
    def __init__(self, parent, mainframe, disp_value=10.0):
        """
        :param parent: The parent window (BioPrinterMainFrame).
        :param mainframe: A reference to the main frame so we can call e.g. mainframe.send_gcode
        :param disp_value: The X/Y/Z movement step size in mm
        """
        super().__init__(parent, title="Axis Control", size=(400, 300))
        self.mainframe = mainframe
        self.disp_value = disp_value

        panel = wx.Panel(self)
        vbox = wx.BoxSizer(wx.VERTICAL)
        panel.SetSizer(vbox)

        label = wx.StaticText(
            panel, 
            label=(
                "Use Arrow Keys (X/Y), W/S (Z) to Move Axes.\n\n"
                f"Current displacement: {self.disp_value} mm"
            )
        )
        vbox.Add(label, 0, wx.ALL|wx.CENTER, 15)

        # Bind key events
        panel.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        panel.SetFocus()  # Make sure the panel can receive keys

        self.CenterOnScreen()
        self.Show()

    def on_key_down(self, event):
        keycode = event.GetKeyCode()
        feedrate_xy = 3000
        feedrate_z = 600
        d = self.disp_value

        # ESC -> close the window
        if keycode == wx.WXK_ESCAPE:
            self.Close()
            return

        if keycode == wx.WXK_UP:
            self.mainframe.send_gcode("G91")
            self.mainframe.send_gcode(f"G1 Y{ d} F{feedrate_xy}")
            self.mainframe.send_gcode("G90")
        elif keycode == wx.WXK_DOWN:
            self.mainframe.send_gcode("G91")
            self.mainframe.send_gcode(f"G1 Y{-d} F{feedrate_xy}")
            self.mainframe.send_gcode("G90")
        elif keycode == wx.WXK_LEFT:
            self.mainframe.send_gcode("G91")
            self.mainframe.send_gcode(f"G1 X{-d} F{feedrate_xy}")
            self.mainframe.send_gcode("G90")
        elif keycode == wx.WXK_RIGHT:
            self.mainframe.send_gcode("G91")
            self.mainframe.send_gcode(f"G1 X{ d} F{feedrate_xy}")
            self.mainframe.send_gcode("G90")
        elif keycode in (ord('w'), ord('W')):
            self.mainframe.send_gcode("G91")
            self.mainframe.send_gcode(f"G1 Z{ d} F{feedrate_z}")
            self.mainframe.send_gcode("G90")
        elif keycode in (ord('s'), ord('S')):
            self.mainframe.send_gcode("G91")
            self.mainframe.send_gcode(f"G1 Z{-d} F{feedrate_z}")
            self.mainframe.send_gcode("G90")
        else:
            event.Skip()

# -----------------------------------------------------------------------
class BioPrinterMainFrame(wx.Frame):
    """
    The main application window for ARTS BioPrinter.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.logger = logging.getLogger("BioPrinterMainFrame")
        self.logger.setLevel(logging.INFO)

        self.SetTitle("ARTS BioPrinter")
        self.SetSize((1400, 800))
        self.Centre()

        self.remote_socket = None
        # default mode
        self.dispenser_mode = 'single'   # or load from settings
        # if you want to let the user pick port+mode upfront, pop up a dialog here

        self.dispenser = MultiDispenserController(
            mode=self.dispenser_mode,
            port=None   # or '/dev/ttyUSB1' if you want to pin it
        )
        self.printer_serial = None
        self.is_paused = False
        self.is_printing = False
        self.stop_flag = False

        # timed‐dispense feature defaults
        self.timed_dispense_enabled = False
        self.timed_dispense_interval = 1.0  # seconds


        # Some parameters

        # read the live pressure/vacuum into your defaults
        try:
            # single‐mode
            self.pressure = self.dispenser.new.read_pressure()
            self.vacuum   = self.dispenser.new.read_vacuum()
        except Exception as e:
            # e.g. in simulation mode or on a comms error
            self.logger.warning(f"Could not read pressure/vacuum: {e}")
            self.pressure = 0.0
            self.vacuum   = 0.0
        self.initial_delay = 1.0
        self.travel_delay = 0.05
        self.dispense_delay = 0.05

        # MULTI-LAYER PLOT SETTINGS
        self.num_layers = 1
        self.layer_height = 0.3

        # ── Registration state ──
        self.corners = [None]*4               # will hold 4 tuples (x, y, z)
        self.corner_plots = []                # matplotlib “line” or “scatter” handles
        self.reg_origin = None                # (x0, y0) → lower‐left corner
        self.reg_angle = None                 # rotation angle in radians
        self.reg_origin_z = None
        self.initial_position = None
        self.initial_position_z = None
        self.raw_start = None

        self.colors = [
            "red","orange","yellow","lime","cyan","blue","blueviolet","violet","magenta"
        ]

        self.init_layout()

    def send_remote_gcode(self, cmd):
        if self.remote_socket:
            try:
                self.remote_socket.sendall((cmd.strip() + "\n").encode())
                self.log(f"[REMOTE GCODE -> PRINTER] {cmd}")
                # Optionally, read and log the response:
                response = self.remote_socket.recv(1024).decode().strip()
                self.log(f"[REMOTE RESPONSE] {response}")
            except Exception as e:
                self.log(f"Error sending remote G-code: {e}")
        else:
            self.log("No remote connection established. Click Remote Connect first.")

    def on_interval_print(self, event):
        if self.is_printing:
            self.log("Already printing!")
            return
        self.is_printing = True
        self.stop_flag   = False
        self.log("Interval print starting…")
        t = threading.Thread(target=self.run_interval_print)
        t.daemon = True
        t.start()

    def on_remote_connect(self, event):
        # Default connection info
        default_ip = "artslab-desktop.mammut-betta.ts.net"
        default_port = "8888"

        # Ask for remote host
        dlg = wx.TextEntryDialog(self, "Enter remote host (IP or hostname):", "Remote Connect", default_ip)
        if dlg.ShowModal() == wx.ID_OK:
            remote_host = dlg.GetValue().strip()
        else:
            dlg.Destroy()
            return
        dlg.Destroy()

        # Ask for remote port
        dlg2 = wx.TextEntryDialog(self, "Enter remote port:", "Remote Connect", default_port)
        if dlg2.ShowModal() == wx.ID_OK:
            try:
                remote_port = int(dlg2.GetValue().strip())
            except ValueError:
                self.log("Invalid port number.")
                dlg2.Destroy()
                return
        else:
            dlg2.Destroy()
            return
        dlg2.Destroy()

        try:
            self.remote_socket = socket.create_connection((remote_host, remote_port), timeout=5)
            self.log(f"Connected to remote printer at {remote_host}:{remote_port}")
        except Exception as e:
            self.log(f"Remote connection failed: {e}")
            self.remote_socket = None

    def init_layout(self):
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(main_sizer)

        # -- TOP ROW --
        top_panel = wx.Panel(self)
        top_sizer = wx.BoxSizer(wx.HORIZONTAL)
        top_panel.SetSizer(top_sizer)
        main_sizer.Add(top_panel, 0, wx.EXPAND|wx.ALL, 5)

        port_choices = []
        ports = list(serial.tools.list_ports.comports())
        if ports:
            port_choices = [p.device for p in ports]
        default_port = "/dev/ttyACM0" if "/dev/ttyACM0" in port_choices else None

        self.cbo_ports = wx.ComboBox(top_panel, style=wx.CB_READONLY, choices=port_choices)
        if default_port:
            self.cbo_ports.SetValue(default_port)
        else:
            if port_choices:
                self.cbo_ports.SetSelection(0)
        top_sizer.Add(wx.StaticText(top_panel, label="Port:"), 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)
        top_sizer.Add(self.cbo_ports, 0, wx.RIGHT, 5)

        lbl_at = wx.StaticText(top_panel, label="@")
        top_sizer.Add(lbl_at, 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)

        baud_choices = ["2400","9600","19200","38400","57600","115200","250000"]
        self.cbo_baud = wx.ComboBox(top_panel, style=wx.CB_READONLY, choices=baud_choices, value="115200")
        top_sizer.Add(self.cbo_baud, 0, wx.RIGHT, 5)

        btn_refresh = wx.Button(top_panel, label="Refresh Ports")
        btn_refresh.Bind(wx.EVT_BUTTON, self.on_refresh_ports)
        top_sizer.Add(btn_refresh, 0, wx.RIGHT, 5)

        btn_connect = wx.Button(top_panel, label="Connect")
        btn_connect.Bind(wx.EVT_BUTTON, self.on_printer_connect)
        top_sizer.Add(btn_connect, 0, wx.RIGHT, 5)

        btn_remote_connect = wx.Button(top_panel, label="Remote Connect")
        btn_remote_connect.Bind(wx.EVT_BUTTON, self.on_remote_connect)
        top_sizer.Add(btn_remote_connect, 0, wx.RIGHT, 5)

        # ── New “Registration” button ──
        btn_registration = wx.Button(top_panel, label="Registration")
        btn_registration.Bind(wx.EVT_BUTTON, self.on_open_registration)
        top_sizer.Add(btn_registration, 0, wx.RIGHT, 5)

        # -- BOTTOM ROW --
        layer_panel = wx.Panel(self)
        layer_sizer = wx.BoxSizer(wx.HORIZONTAL)
        layer_panel.SetSizer(layer_sizer)
        main_sizer.Add(layer_panel, 0, wx.EXPAND | wx.ALL, 5)

        self.txt_layer_input = wx.TextCtrl(layer_panel, value=str(self.num_layers), size=(50, -1))
        layer_sizer.Add(wx.StaticText(layer_panel, label="Layers:"), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        layer_sizer.Add(self.txt_layer_input, 0, wx.RIGHT, 5)

        btn_set_layers = wx.Button(layer_panel, label="Set Layers")
        btn_set_layers.Bind(wx.EVT_BUTTON, self.on_set_layers)
        layer_sizer.Add(btn_set_layers, 0, wx.RIGHT, 5)

        btn_set_temp = wx.Button(layer_panel, label="Set Temp")
        btn_set_temp.Bind(wx.EVT_BUTTON, self.on_set_temp)
        layer_sizer.Add(btn_set_temp, 0, wx.RIGHT, 5)

        btn_read_temp = wx.Button(layer_panel, label="Read Temp")
        btn_read_temp.Bind(wx.EVT_BUTTON, self.on_read_temp)
        layer_sizer.Add(btn_read_temp, 0, wx.RIGHT, 5)

        btn_load_txt = wx.Button(layer_panel, label="Load File")
        btn_load_txt.Bind(wx.EVT_BUTTON, self.on_load_txt_file)
        layer_sizer.Add(btn_load_txt, 0, wx.RIGHT, 5)

        # ── Two print buttons ──
        btn_reg_print = wx.Button(layer_panel, label="Registration Print")
        btn_reg_print.Bind(wx.EVT_BUTTON, self.on_registration_print)
        layer_sizer.Add(btn_reg_print, 0, wx.RIGHT, 5)

        btn_simple_print = wx.Button(layer_panel, label="Regular Print")
        btn_simple_print.Bind(wx.EVT_BUTTON, self.on_simple_print)
        layer_sizer.Add(btn_simple_print, 0, wx.RIGHT, 5)

        # ← NEW: Interval Print
        # btn_interval_print = wx.Button(layer_panel, label="Interval Print")
        # btn_interval_print.Bind(wx.EVT_BUTTON, self.on_interval_print)
        # layer_sizer.Add(btn_interval_print, 0, wx.RIGHT, 5)

        # -- Middle row: arrow panel + console + 3D plot
        row2_sizer = wx.BoxSizer(wx.HORIZONTAL)
        main_sizer.Add(row2_sizer, 1, wx.EXPAND)

        left_col = wx.BoxSizer(wx.VERTICAL)
        left_panel = wx.Panel(self)
        left_panel.SetSizer(left_col)
        row2_sizer.Add(left_panel, 0, wx.EXPAND|wx.ALL, 5)

        self.arrow_panel = ArrowControlPanel(left_panel, self)
        left_col.Add(self.arrow_panel, 0, wx.EXPAND)

        self.txt_console = wx.TextCtrl(left_panel, style=wx.TE_MULTILINE|wx.TE_READONLY)
        left_col.Add(self.txt_console, 1, wx.EXPAND|wx.TOP, 5)

        self.plot_panel = wx.Panel(self)
        row2_sizer.Add(self.plot_panel, 1, wx.EXPAND|wx.ALL, 5)

        self.fig = plt.Figure()
        light_gray = (0.9, 0.9, 0.9)
        self.fig.patch.set_facecolor(light_gray)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(light_gray)

        self.canvas = FigureCanvas(self.plot_panel, -1, self.fig)
        plot_sizer = wx.BoxSizer(wx.VERTICAL)
        plot_sizer.Add(self.canvas, 1, wx.EXPAND)
        self.plot_panel.SetSizer(plot_sizer)

        # Optionally set an initial displacement in the arrow panel after creation:
        wx.CallAfter(self.arrow_panel.set_active_displacement, 10.0)

    # -------------------------------------------------------------------------
    # Logging, G-code, etc.
    # -------------------------------------------------------------------------
    def log(self, msg):
        wx.CallAfter(self.txt_console.AppendText, msg + "\n")

    def on_set_temp(self, event):
        dlg = wx.TextEntryDialog(self, "Enter extruder temperature (°C):", "Set Extruder Temp", "70")
        if dlg.ShowModal() == wx.ID_OK:
            try:
                temp = float(dlg.GetValue().strip())
                self.send_gcode(f"M104 S{int(temp)}")
                self.log(f"Set extruder temperature to {temp:.1f} °C")
            except ValueError:
                self.log("Invalid temperature entered.")
        dlg.Destroy()

    def on_read_temp(self, event):
        self.send_gcode("M105")
        if self.printer_serial and self.printer_serial.is_open:
            try:
                response = self.printer_serial.readline().decode("utf-8").strip()
                self.log(f"[PRINTER TEMP RAW] {response}")
                if "T:" in response:
                    parts = response.split()
                    for part in parts:
                        if part.startswith("T:"):
                            temp_vals = part[2:].split("/")
                            current_c = float(temp_vals[0])
                            target_c = float(temp_vals[1])
                            current_f = (current_c * 9/5) + 32
                            target_f = (target_c * 9/5) + 32
                            self.log(f"Hotend Temp: {current_c:.1f}°C / {target_c:.1f}°C")
                            self.log(f"              {current_f:.1f}°F / {target_f:.1f}°F")
                            return
            except Exception as e:
                self.log(f"Failed to read temperature: {e}")

    def on_open_registration(self, event):
        """
        Opens a pop‐up dialog containing registration fields, etc.
        """
        # 1) open registration dialog
        dlg = RegistrationDialog(self, title="Registration")
        dlg.Show()

        # 2) also open the arrow‐control window (AxisControlFrame) and leave it up
        #    until the user explicitly closes it.
        AxisControlFrame(parent=self, mainframe=self, disp_value=self.arrow_panel.disp_value)

    def on_register_corner(self, idx: int):
        """
        Sends M114 to the printer, parses X/Y/Z with regex,
        stores corner in self.corners[idx], and updates the plot.
        """
        if not (self.printer_serial and self.printer_serial.is_open):
            self.log("Printer not connected. Cannot register corner.")
            return

        # 1) Send M114
        try:
            self.printer_serial.reset_input_buffer()
            self.printer_serial.write(b"M114\n")
        except Exception as e:
            self.log(f"Failed to send M114: {e}")
            return

        # 2) Read one line of response
        line = self.printer_serial.readline().decode("utf-8").strip()
        if not line:
            self.log("No response to M114.")
            return

        # 3) Parse X, Y, Z with regex
        try:
            mx = re.search(r"X:\s*([-+]?[0-9]*\.?[0-9]+)", line)
            my = re.search(r"Y:\s*([-+]?[0-9]*\.?[0-9]+)", line)
            mz = re.search(r"Z:\s*([-+]?[0-9]*\.?[0-9]+)", line)
            if not (mx and my and mz):
                raise ValueError(f"Incomplete M114 reply: “{line}”")
            x = float(mx.group(1))
            y = float(my.group(1))
            z = float(mz.group(1))
        except Exception as e:
            self.log(f"Error parsing M114 (“{line}”): {e}")
            return

        # 4) Store and show in the mainframe’s list of corners
        self.corners[idx] = (x, y, z)
        self.log(f"Corner {idx+1} registered at X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

        # 5) Re-draw the corner markers on the 3D plot
        self._update_corner_plot()

    def on_register_initial(self):
        """
        Sends M114 to the printer, parses X/Y/Z, and logs the “initial position.”
        """
        if not (self.printer_serial and self.printer_serial.is_open):
            self.log("Printer not connected. Cannot register initial position.")
            return

        try:
            self.printer_serial.reset_input_buffer()
            self.printer_serial.write(b"M114\n")

            # Give the printer time to respond
            time.sleep(0.2)

            # Read all incoming lines until one has position info
            response_lines = []
            for _ in range(5):  # Limit to prevent hanging
                line = self.printer_serial.readline().decode("utf-8").strip()
                if line:
                    response_lines.append(line)
                    if "X:" in line and "Y:" in line and "Z:" in line:
                        break
            else:
                self.log("No valid position response received from M114.")
                return

            # Now parse the best matching line
            line = response_lines[-1]
            mx = re.search(r"X:\s*([-+]?[0-9]*\.?[0-9]+)", line)
            my = re.search(r"Y:\s*([-+]?[0-9]*\.?[0-9]+)", line)
            mz = re.search(r"Z:\s*([-+]?[0-9]*\.?[0-9]+)", line)
            if not (mx and my and mz):
                self.log(f"Error parsing M114 reply: “{line}”")
                return

            x = float(mx.group(1))
            y = float(my.group(1))
            z = float(mz.group(1))

            self.initial_position   = (x, y)
            self.initial_position_z = z

            self.log(f"Registered initial position at X={x:.2f}, Y={y:.2f}, Z={z:.2f}.")
            self.plot_initial()  # Redraw with updated star
        except Exception as e:
            self.log(f"Failed to send or parse M114: {e}")

    def _update_corner_plot(self):
        """
        Redraw corner markers (blue dots) and connect them in polygon order.
        If fewer than 2 corners are set, only scatter the existing ones.
        """
        # Remove old corner plots
        for h in self.corner_plots:
            try:
                h.remove()
            except:
                pass
        self.corner_plots.clear()

        # Collect all corners that are not None
        pts = [c for c in self.corners if c is not None]
        if not pts:
            self.canvas.draw()
            return

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]  # all four have nearly same Z (mold height)

        # 1) Scatter blue dots (size=50)
        sc = self.ax.scatter(xs, ys, zs, color="blue", s=50, depthshade=True)
        self.corner_plots.append(sc)

        # 2) If ≥2 corners, connect them in order around centroid
        if len(pts) > 1:
            # Compute centroid (in XY)
            cx = sum(xs)/len(xs)
            cy = sum(ys)/len(ys)
            # Sort points by angle from centroid 
            # WRONG
            angles = [math.atan2(p[1]-cy, p[0]-cx) for p in pts]
            sorted_idx = sorted(range(len(pts)), key=lambda i: angles[i])
            sorted_pts = [pts[i] for i in sorted_idx] + [pts[sorted_idx[0]]]  # close loop

            poly_x = [p[0] for p in sorted_pts]
            poly_y = [p[1] for p in sorted_pts]
            poly_z = [p[2] for p in sorted_pts]

            line = self.ax.plot(poly_x, poly_y, poly_z, color="blue", linewidth=1.5)
            self.corner_plots.extend(line)

        self.canvas.draw()

    def on_compute_transformation(self, event):
        # 1) Make sure all four corners are there
        if any(c is None for c in self.corners):
            self.log("Need all 4 corners before computing transformation.")
            return

        # 2) Unpack BL, BR, TR, TL
        bl = np.array(self.corners[0][:2])
        br = np.array(self.corners[1][:2])
        tr = np.array(self.corners[2][:2])
        tl = np.array(self.corners[3][:2])

        # 3) Build mold‐frame axes
        x_axis = br - bl    # bottom‐left → bottom‐right
        y_axis = tl - bl    # bottom‐left → top‐left

        # 4) Unit vectors
        x_unit = x_axis / np.linalg.norm(x_axis)
        y_unit = y_axis / np.linalg.norm(y_axis)

        # 5) Rotation matrix from raw→mold
        R = np.column_stack((x_unit, y_unit))   # 2×2

        # 6) Store everything
        self.transform_R   = R
        self.reg_origin    = (bl[0], bl[1])
        self.reg_origin_z  = self.corners[0][2]
        self.mold_center   = tuple(np.mean([c[:2] for c in self.corners], axis=0))

        self.log("Computed transform:")
        self.log(f"  Origin (BL) = {self.reg_origin}, Z₀ = {self.reg_origin_z:.2f}")
        self.log(f"  R =\n{R}")
        self.log(f"  Mold center = {self.mold_center}")

    def on_clear_corners(self, event):
        """Clear all stored corners and remove markers from the plot."""
        self.corners = [None]*4
        if hasattr(self, 'corner_txt'):
            for txt in self.corner_txt:
                txt.SetValue("")
        self.reg_origin = None
        self.reg_angle = None
        self._update_corner_plot()
        self.log("All corners cleared.")

    def send_gcode(self, cmd):
        # If a remote connection is established, use it.
        if self.remote_socket:
            try:
                # Send the command with a newline (adjust to \r\n if your printer requires it)
                self.remote_socket.sendall((cmd.strip() + "\n").encode("utf-8"))
                self.log(f"[REMOTE GCODE -> PRINTER] {cmd}")
                # Optionally, try to read a response.
                response = self.remote_socket.recv(1024).decode().strip()
                if response:
                    self.log(f"[REMOTE RESPONSE] {response}")
            except Exception as e:
                self.log(f"Error sending remote G-code: {e}")
        # If a local printer connection is available, use that.
        elif self.printer_serial and self.printer_serial.is_open:
            line = (cmd.strip() + "\n").encode("utf-8")
            self.printer_serial.write(line)
            self.printer_serial.flush()
            self.log(f"[GCODE -> PRINTER] {cmd}")

            # ✅ Read until we get "ok"
            while True:
                response = self.printer_serial.readline().decode('utf-8').strip()
                if response:
                    self.log(f"[PRINTER RESPONSE] {response}")
                    if "ok" in response.lower():
                        break
        # Otherwise, log an error (or choose not to do anything).
        else:
            self.log(f"[ERROR] No connection available for G-code: {cmd}")

    def on_refresh_ports(self, event):
        ports = list(serial.tools.list_ports.comports())
        port_choices = [p.device for p in ports]
        self.cbo_ports.Clear()
        self.cbo_ports.AppendItems(port_choices)
        if "/dev/ttyACM0" in port_choices:
            self.cbo_ports.SetValue("/dev/ttyACM0")
        elif port_choices:
            self.cbo_ports.SetSelection(0)

    def on_printer_connect(self, event):
        port = self.cbo_ports.GetValue()
        try:
            baud = int(self.cbo_baud.GetValue())
        except ValueError:
            baud = 115200
        if not port:
            self.log("No port selected.")
            return
        try:
            self.printer_serial = serial.Serial(port, baud, timeout=2)
            time.sleep(2)
            self.log(f"Connected to printer on {port} @ {baud}")
        except Exception as e:
            self.log(f"Failed to connect: {e}")
            self.printer_serial = None

    def on_load_txt_file(self, event):
        global x_coords, y_coords, z_coords, F_coords

        dlg = wx.FileDialog(
            self,
            "Open .txt file with X/Y/Z/F lines",
            wildcard="Text files (*.txt)|*.txt|All files|*.*",
            style=wx.FD_OPEN | wx.FD_FILE_MUST_EXIST
        )
        if dlg.ShowModal() == wx.ID_CANCEL:
            return
        path = dlg.GetPath()
        dlg.Destroy()

        try:
            # 1) Read raw (global) points
            raw = []
            with open(path, "r") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 4:
                        x, y, z, fv = parts
                        raw.append((float(x), float(y), float(z), float(fv)))

            if not raw:
                self.log("No valid lines found in the file.")
                return

            # 2) Save full raw G-code and remember its very first point
            self.raw_gcode = raw
            self.raw_start = (raw[0][0], raw[0][1])
            self.log(f"Loaded {len(raw)} raw points (in global bed coords).")

            # 3) Reset any previous preview data
            x_coords.clear(); y_coords.clear(); z_coords.clear(); F_coords.clear()

            # 4) Always show the raw cloud until we transform it later
            for x, y, z, f in raw:
                x_coords.append(x)
                y_coords.append(y)
                z_coords.append(z)
                F_coords.append(f)
            self.log(f"Preview: plotting {len(raw)} raw points.")

            # 5) Kick off your unified plot routine
            self.plot_initial()

        except Exception as e:
            self.log(f"Error loading file: {e}")

    def on_registration_print(self, event):
        """Print using the registration+transform logic (old run_printing)."""
        if self.is_printing:
            self.log("Already printing!")
            return
        self.is_printing = True
        self.stop_flag   = False
        self.log("Registration print starting…")
        t = threading.Thread(target=self.run_registration_print)  # your existing transformed version
        t.daemon = True
        t.start()

    def on_simple_print(self, event):
        """Print in raw/global coords using x_coords/y_coords lists."""
        if self.is_printing:
            self.log("Already printing!")
            return
        self.is_printing = True
        self.stop_flag   = False
        self.log("Regular print starting…")
        t = threading.Thread(target=self.run_simple_print)
        t.daemon = True
        t.start()

    def is_corner(self, i, angle_threshold_degrees=45):
        """
        Returns True if the angle between segments (i-1 to i) and (i to i+1)
        is sharper than the given threshold (default = 45 degrees).
        """
        if i <= 0 or i >= len(x_coords) - 1:
            return False

        # Get vectors
        vx1 = x_coords[i] - x_coords[i - 1]
        vy1 = y_coords[i] - y_coords[i - 1]
        vx2 = x_coords[i + 1] - x_coords[i]
        vy2 = y_coords[i + 1] - y_coords[i]

        # Skip degenerate zero-length vectors
        if vx1 == 0 and vy1 == 0 or vx2 == 0 and vy2 == 0:
            return False

        # Compute angle between vectors
        dot = vx1 * vx2 + vy1 * vy2
        mag1 = (vx1**2 + vy1**2) ** 0.5
        mag2 = (vx2**2 + vy2**2) ** 0.5
        cos_theta = dot / (mag1 * mag2)

        # Clamp to avoid math domain errors from floating-point errors
        cos_theta = max(-1.0, min(1.0, cos_theta))

        angle_rad = math.acos(cos_theta)
        angle_deg = math.degrees(angle_rad)

        return angle_deg < angle_threshold_degrees  # Sharp if less than threshold

    @staticmethod
    def calculate_travel_parameters(current_point, next_point, desired_feedrate):
        # Calculate Euclidean distance
        dx = next_point[0] - current_point[0]
        dy = next_point[1] - current_point[1]
        dz = next_point[2] - current_point[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Calculate time required for the move: time (sec) = (60 * distance) / feedrate
        delay = (60 * distance) / desired_feedrate
        min_delay = 0.05  # enforce at least 50 ms delay for very short moves
        if delay < min_delay:
            delay = min_delay
        return delay
    
    def run_simple_print(self):
        self.log("Applying initial delay and moving to start...")
        if not x_coords:
            self.log("No coordinates loaded.")
            self.is_printing = False
            return

        self.send_gcode("G90")              # Absolute positioning
        self.send_gcode("G92 X0 Y0 Z0")     # Set current as origin

        self.send_gcode(f"G1 X{x_coords[0]} Y{y_coords[0]} Z{z_coords[0]} F{F_coords[0]}")
        time.sleep(self.initial_delay)

        self.log("Starting print...")
        self.dispensing = False

        for i in range(len(x_coords)):
            if self.stop_flag:
                self.log("Printing Stopped by user/emergency stop.")
                self.is_printing = False
                return
            
            # Start dispenser once at first move
            if i == 0 and not self.dispensing:
                self.dispenser.dispenser_callback('start')
                self.dispensing = True

            # stop dispensing if Z = 0.700
            if abs(z_coords[i] - 0.700) < 0.001:
                if self.dispensing:
                    # Stop dispensing when at travel height (z = 0.700)
                    self.dispenser.dispenser_callback('stop')
                    self.dispensing = False

            else:
                if not self.dispensing:
                    self.dispenser.dispenser_callback('start')
                    self.dispensing = True

            self.send_gcode(f"G1 X{x_coords[i]} Y{y_coords[i]} Z{z_coords[i]} F{F_coords[i]}")

            # Calculate an adaptive delay for this move if a next point exists
            if i < len(x_coords) - 1:
                current_point = (x_coords[i], y_coords[i], z_coords[i])
                next_point = (x_coords[i+1], y_coords[i+1], z_coords[i+1])
                adaptive_delay = self.calculate_travel_parameters(current_point, next_point, F_coords[i])
            else:
                adaptive_delay = self.travel_delay  # use default for last point

            if self.is_corner(i):
                self.log(f"Corner detected at index {i}. Using no delay.")
                continue
            else:
                self.log("Apaptive")
                time.sleep(adaptive_delay)
        
        self.dispenser.dispenser_callback('start')
        self.send_gcode("G90")
        if self.printer_serial:
            self.printer_serial.flush()
        self.log("Printing done.")
        self.is_printing = False

    def run_interval_print(self):
        import time, threading

        self.log("Applying initial delay and moving to start…")
        if not x_coords:
            self.log("No coordinates loaded.")
            self.is_printing = False
            return

        # helper: non-blocking dispenser toggle
        def async_toggle(command):
            for disp in self.dispenser.controllers:
                threading.Thread(
                    target=disp.dispenser_callback,
                    args=(command,),
                    daemon=True
                ).start()

        # 1) Prep the printer
        self.send_gcode("G90")              # absolute positioning
        self.send_gcode("G92 X0 Y0 Z0")     # zero origin
        self.send_gcode(f"G1 X{x_coords[0]} Y{y_coords[0]} Z{z_coords[0]} F{F_coords[0]}")
        time.sleep(self.initial_delay)

        # 2) Kick off the dispenser
        async_toggle('start')
        self.dispensing = True
        self.log("  → Dispenser ON @ t=0")

        interval = self.timed_dispense_interval
        elapsed  = 0.0

        # 3) Main print loop
        for i in range(len(x_coords)):
            if self.stop_flag:
                self.log("Printing stopped by user.")
                self.is_printing = False
                return

            # a) send the move
            self.send_gcode(f"G1 X{x_coords[i]} Y{y_coords[i]} Z{z_coords[i]} F{F_coords[i]}")

            # b) compute how long that _should_ take
            if i < len(x_coords) - 1:
                delay = self.calculate_travel_parameters(
                    (x_coords[i],   y_coords[i],   z_coords[i]),
                    (x_coords[i+1], y_coords[i+1], z_coords[i+1]),
                    F_coords[i]
                )
            else:
                delay = self.travel_delay

            # c) actually wait
            time.sleep(delay)

            # d) update our accumulator and toggle if needed
            if self.timed_dispense_enabled:
                elapsed += delay
                while elapsed >= interval:
                    cmd = 'stop' if self.dispensing else 'start'
                    async_toggle(cmd)
                    self.dispensing = not self.dispensing
                    elapsed -= interval
                    state = "OFF" if not self.dispensing else "ON"
                    self.log(f"  → Dispenser {state} @ +{interval:.2f}s")

        # 4) wrap up
        if self.dispensing:
            async_toggle('stop')
            self.log("  → Dispenser OFF (end)")

        self.send_gcode("G90")
        if self.printer_serial:
            self.printer_serial.flush()
        self.log("Interval print done.")
        self.is_printing = False

    def run_registration_print(self):
        self.silent_mode = True
        # 0) Pre-flight checks
        if not hasattr(self, 'raw_gcode') or not self.raw_gcode:
            self.log("No G-code loaded. Please load a file first.")
            self.is_printing = False
            return

        if self.reg_origin is None or self.transform_R is None or self.reg_origin_z is None:
            self.log("Error: Please register corners and compute transform before printing.")
            self.is_printing = False
            return

        # 1) Grab your anchor and matrix
        x_start, y_start = self.raw_start
        x0, y0          = self.initial_position
        z0              = self.initial_position_z
        R               = self.transform_R

        # 2) Home X/Y then jump to your start corner
        self.send_gcode("G28 X Y")
        time.sleep(0.5)

        self.send_gcode("G90")
        # lift off bed
        self.send_gcode(f"G1 Z{z0+5:.2f} F600")
        time.sleep(0.5)
        # move horizontally to the registered BL corner
        self.send_gcode(f"G1 X{x0:.2f} Y{y0:.2f} F3000")
        time.sleep(0.5)
        # lower onto the mold top
        self.send_gcode(f"G1 Z{z0:.2f} F300")
        time.sleep(0.5)

        self.log("Starting print loop…")
        self.dispensing = False

        # 3) Loop: center → rotate → translate
        for i, (rx, ry, rz, fv) in enumerate(self.raw_gcode):
            if self.stop_flag:
                self.log("Printing stopped by user.")
                self.is_printing = False
                return

            # center about spiral center
            v  = np.array([rx - x_start, ry - y_start])  # shape (2,)
            # rotate into mold frame
            xy = R @ v + np.array([x0, y0])   # shape (2,)
            # translate so BL → (x0,y0)
            xf, yf = xy
            # compute nozzle Z
            zf      = rz + z0

            # dispenser on/off (unchanged)
            if i == 0 and not self.dispensing:
                self.dispenser.dispenser_callback('start')
                self.dispensing = True
            if abs(zf - self.layer_height) < 1e-3:
                if self.dispensing:
                    self.dispenser.dispenser_callback('stop')
                    self.dispensing = False
            else:
                if not self.dispensing:
                    self.dispenser.dispenser_callback('start')
                    self.dispensing = True

            # send the transformed G-code
            self.send_gcode(f"G1 X{xf:.3f} Y{yf:.3f} Z{zf:.3f} F{fv:.1f}")

            # adaptive delay (rotate next point)
            if i < len(self.raw_gcode) - 1:
                nrx, nry, nrz, nfv = self.raw_gcode[i+1]
                nv = np.array([nrx - x_start, nry - y_start])
                nxy = R @ nv
                xf2, yf2 = nxy + np.array([x0, y0])
                zf2      = nrz + z0
                delay    = self.calculate_travel_parameters(
                            (xf, yf, zf),
                            (xf2, yf2, zf2),
                            fv
                        )
            else:
                delay = self.travel_delay

            if self.is_corner(i):
                self.log(f"Corner detected at index {i}, skipping delay")
            else:
                time.sleep(delay)

        # 4) Finish
        if self.dispensing:
            self.dispenser.dispenser_callback('stop')
        self.send_gcode("G90")
        self.log("Printing complete.")
        self.is_printing = False

    # -------------------------------------------------------------------------
    # Movement, Homing, etc.
    # -------------------------------------------------------------------------
    def moveXY(self, dx, dy):
        if dx != 0 or dy != 0:
            self.send_gcode("G91")
            feedrate = 3000
            if dx != 0:
                self.send_gcode(f"G1 X{dx:.2f} F{feedrate}")
            if dy != 0:
                self.send_gcode(f"G1 Y{dy:.2f} F{feedrate}")
            self.send_gcode("G90")

    def moveZ(self, dz):
        if dz != 0:
            self.send_gcode("G91")
            feedrate = 600
            self.send_gcode(f"G1 Z{dz:.2f} F{feedrate}")
            self.send_gcode("G90")

    def home_axis(self, axis):
        if axis == "x":
            self.send_gcode("G28 X")
        elif axis == "y":
            self.send_gcode("G28 Y")
        elif axis == "z":
            self.send_gcode("G28 Z")
        elif axis == "all":
            self.send_gcode("G28")
        else:
            self.log(f"Invalid axis: {axis}")

    def local_origin(self):
        # Uses G92 to set current position as local origin
        self.send_gcode("G92 X0 Y0 Z0")
        self.log("Setting new local origin...")
        # time.sleep(4)

    def goto_origin(self):
        """
        Moves relative to the homed X and Y.
        1) Home X/Y so that X=0, Y=0 at the end of the homing.
        2) Move to a chosen offset (e.g., bed center).
        """
        # --- 1) Home only X and Y so they are at 0,0 ---
        self.send_gcode("G28 X Y")  # or G28 X and G28 Y separately if desired
        self.log("Homed X and Y. (X=0, Y=0)")

        # --- 2) Move to your chosen 'origin' offset from home ---
        bed_x = 225   # Adjust to your actual bed width
        bed_y = 260  # Adjust to your actual bed height
        feedrate = 3000   # mm/min

        # For example, if you want the bed center to be your “origin”:
        origin_x = bed_x  / 2.0  # ~112.5
        origin_y = bed_y / 2.0  # ~130

        # Ensure absolute positioning
        self.send_gcode("G90")
        self.send_gcode(f"G1 X{origin_x:.2f} Y{origin_y:.2f} Z{100:.2f} F{feedrate}")
        self.log(f"Moved to origin offset from home: X={origin_x:.2f}, Y={origin_y:.2f} Z{100:.2f}")

    def level_z_axis(self):
        """
        Levels the Z-axis by homing it and setting an appropriate Z-offset.
        """
        self.log("Starting Z-axis leveling...")

        # 1. Home Z-axis
        self.log("Homing Z-axis...")
        self.send_gcode("G28 Z")
        time.sleep(2)  # Wait for homing to complete

        # 2. Move Z-axis slightly up to prevent nozzle from crashing
        self.log("Moving Z-axis up by 5mm to clear the bed...")
        self.send_gcode("G91")  # Switch to relative positioning
        self.send_gcode("G1 Z5 F300")  # Move Z up by 5mm
        self.send_gcode("G90")  # Switch back to absolute positioning
        time.sleep(1)

        # 3. Set Z-offset (adjust as needed)
        # You might need to adjust this value based on your printer's calibration
        desired_offset = 0.2  # Example: 0.2mm above the bed
        self.z_offset = desired_offset
        self.send_gcode(f"G92 Z{self.z_offset}")  # Set current Z position as the new Z=0
        self.log(f"Z-axis leveled. Z-offset set to {self.z_offset}mm.")


    def emergency_stop(self):
        self.log("EMERGENCY STOP!")
        self.stop_flag = True
        self.dispenser.emergency_stop()

    # -------------------------------------------------------------------------
    # Plotting
    # -------------------------------------------------------------------------
    
    def on_set_layers(self, event):
        """Update the number of layers from user input and replot."""
        try:
            new_layers = int(self.txt_layer_input.GetValue())
            if new_layers > 0:
                self.num_layers = new_layers
                self.log(f"Number of layers set to {self.num_layers}")
                self.plot_initial()
            else:
                self.log("Layer count must be greater than 0")
        except ValueError:
            self.log("Invalid layer count. Enter a number.")
    
    def plot_initial(self):
        self.ax.clear()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("G-code Preview")

        # 1) Always plot the raw global shape in red
        if hasattr(self, 'raw_gcode') and self.raw_gcode:
            raw_x = [pt[0] for pt in self.raw_gcode]
            raw_y = [pt[1] for pt in self.raw_gcode]
            raw_z = [pt[2] for pt in self.raw_gcode]

            for layer in range(self.num_layers):
                z_layer = [z + layer * self.layer_height for z in raw_z]
                self.ax.scatter(raw_x, raw_y, z_layer, marker='.', c='red', alpha=0.2, label='Raw G-code')

            # Also mark the starting point with a blue star
            self.ax.scatter([raw_x[0]], [raw_y[0]], [raw_z[0]], c='blue', marker='*', s=100, label='Raw Start')

        # 2) If we have a transform AND an initial position, plot the transformed version
        if hasattr(self, 'transform_R') and self.transform_R is not None and \
        hasattr(self, 'initial_position') and self.initial_position is not None:

            R = self.transform_R
            x0, y0 = self.initial_position
            z0 = self.initial_position_z
            x_start, y_start = self.raw_start

            tx, ty, tz = [], [], []
            for rx, ry, rz, _ in self.raw_gcode:
                v = np.array([rx - x_start, ry - y_start])
                xy = R @ v + np.array([x0, y0])
                tx.append(xy[0])
                ty.append(xy[1])
                tz.append(rz + z0)

            for layer in range(self.num_layers):
                z_layer = [z + layer * self.layer_height for z in tz]
                self.ax.scatter(tx, ty, z_layer, marker='.', c='limegreen', alpha=0.2, label='Transformed')

            # Mark the transformed start point with a purple star
            self.ax.scatter([tx[0]], [ty[0]], [tz[0]], c='magenta', marker='*', s=120, label='Transformed Start')

        # 3) Draw mold corners
        self._update_corner_plot()

        # 4) Remove duplicate labels from the legend
        handles, labels = self.ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        self.ax.legend(unique.values(), unique.keys())

        self.canvas.draw()

    def update_layer_color(self, layer_idx):
        global x_coords, y_coords, z_coords
        self.ax.clear()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Multi-Layer Plot with completed layers")

        for j in range(self.num_layers):
            color = self.colors[j % len(self.colors)]
            z_layer = [z + j*self.layer_height for z in z_coords]
            self.ax.plot(
                x_coords, y_coords, z_layer,
                marker='o',
                color=color,
                alpha=0.2,
                markersize=2  # Smaller markers
            )
        self.canvas.draw()


# -------------------------------------------------------------------------
class ArrowControlPanel(wx.Panel):
    """
    Arrow/homing panel. The key difference now: "arrow_mode" opens a new
    AxisControlFrame. No more toggling arrow_mode_active in the main frame.
    """
    def __init__(self, parent, mainframe):
        super().__init__(parent, size=(500, 510))
        self.mainframe = mainframe
        self.assets = self.load_assets()

        self.current_disp_button = None
        self.disp_value = 10.0
        self.disp_buttons = {}

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.create_controls()

    def on_paint(self, event):
        dc = wx.PaintDC(self)
        if "background" in self.assets:
            dc.DrawBitmap(self.assets["background"], 0, 0, True)
        event.Skip()

    def load_assets(self):
        assets = {}
        if not os.path.isdir(ASSETS_PATH):
            print(f"[WARN] Assets path {ASSETS_PATH} doesn't exist.")
            return assets
        for root, _, files in os.walk(ASSETS_PATH):
            for file in files:
                if file.lower().endswith(('.png','.jpg','.jpeg')):
                    key = os.path.splitext(file)[0]
                    path = os.path.join(root, file)
                    try:
                        pil_img = Image.open(path).convert("RGBA")
                        w,h = pil_img.size
                        wx_img = wx.Image(w,h)
                        wx_img.SetData(pil_img.convert("RGB").tobytes())
                        wx_img.SetAlpha(pil_img.getchannel("A").tobytes())
                        assets[key] = wx.Bitmap(wx_img)
                    except:
                        pass
        return assets
    
    def start_scringe(self):
        """Starts dispensing when the button is held down."""
        self.mainframe.log("Scringe dispensing started.")
        # self.mainframe.dispenser.dispenser_callback('start')

    def stop_scringe(self):
        """Stops dispensing when the button is released."""
        self.mainframe.log("Scringe dispensing stopped.")
        self.mainframe.dispenser.dispenser_callback('stop')

    def create_controls(self):
        self.create_button("plus_x_arrow",  311.5,164.38, "plus_x")
        self.create_button("minus_x_arrow", 44.38,164.38, "minus_x")
        self.create_button("plus_y_arrow",  176.8,31.96,  "plus_y")
        self.create_button("minus_y_arrow", 176.8,299.09, "minus_y")

        self.create_button("plus_z_arrow",  413,27,  "plus_z")
        self.create_button("minus_z_arrow", 413,315, "minus_z")

        self.create_disp_button("move_by_0.1_default", 423,265, 0.1)
        self.create_disp_button("move_by_1_default",   423,210, 1.0)
        self.create_disp_button("move_by_10_default",  423,153, 10.0)
        self.create_disp_button("move_by_100_default", 423,96,  100.0)

        self.create_button("Home_x_default",   30.68,26.26,   "home_x")
        self.create_button("Home_y_default",   308.08,289.95, "home_y")
        self.create_button("Home_z_default",   308.08,25.11,  "home_z")
        self.create_button("Home_all_default", 30.68,289.95,  "home_all")

        self.create_button("arrow_mode_default",   37.53,404.11,"arrow_mode")
        self.create_button("play_pause_default",   101.46,408.68,"play_pause")
        self.create_button("stop_default",         149.4,392.69, "emergency_stop")
        self.create_button("gcode",                217.89,401.83,"send_gcode")
        self.create_button("settings",             329.77,404.11,"settings")
        self.create_button("level_z_axis_default", 269,404,"level_z")
        self.create_button("scringe_default", 415,399,"scringe")

        self.create_button("origin button", 180.22,168.95, "local_origin")

    def create_disp_button(self, asset_name, x, y, disp_value):
        default_bmp = self.assets.get(asset_name)
        if not default_bmp:
            return

        clicked_key = asset_name.replace("_default", "_clicked")
        clicked_bmp = self.assets.get(clicked_key, default_bmp)

        btn = wx.StaticBitmap(self, bitmap=default_bmp, pos=(int(x), int(y)))
        self.disp_buttons[disp_value] = (default_bmp, clicked_bmp, btn)

        def on_press(evt):
            pass

        def on_release(evt):
            self.set_active_displacement(disp_value)

        btn.Bind(wx.EVT_LEFT_DOWN, on_press)
        btn.Bind(wx.EVT_LEFT_UP, on_release)

    def set_active_displacement(self, disp_value):
        """
        Update the active displacement value and visually update the corresponding button.
        """
        if disp_value not in self.disp_buttons:
            return
        self.disp_value = disp_value

        # restore old button
        if self.current_disp_button is not None:
            old_data = self.disp_buttons.get(self.current_disp_button)
            if old_data:
                def_bmp, clicked_bmp, widget = old_data
                widget.SetBitmap(def_bmp)

        # set new button
        new_data = self.disp_buttons[disp_value]
        def_bmp, clicked_bmp, widget = new_data
        widget.SetBitmap(clicked_bmp)
        self.current_disp_button = disp_value

        self.mainframe.log(f"Displacement set to {disp_value}")

    def create_button(self, asset_name, x, y, command_id):
        default_bmp = self.assets.get(asset_name)
        if not default_bmp:
            return
        if asset_name.endswith("_default"):
            clicked_key = asset_name.replace("_default","_clicked")
        else:
            clicked_key = asset_name + "_clicked"
        clicked_bmp = self.assets.get(clicked_key, default_bmp)

        btn = wx.StaticBitmap(self, bitmap=default_bmp, pos=(int(x),int(y)))

        if command_id == "scringe":
            btn.Bind(wx.EVT_LEFT_DOWN, lambda evt: self.start_scringe())
            btn.Bind(wx.EVT_LEFT_UP, lambda evt: self.stop_scringe())
        else:
            def on_press(evt):
                btn.SetBitmap(clicked_bmp)
            def on_release(evt):
                btn.SetBitmap(default_bmp)
                self.handle_command(command_id)

            btn.Bind(wx.EVT_LEFT_DOWN, on_press)
            btn.Bind(wx.EVT_LEFT_UP, on_release)

    def handle_command(self, command_id):
        self.mainframe.log(f"Command triggered: {command_id}")
        mf = self.mainframe
        if command_id == "plus_x":
            mf.moveXY(+self.disp_value, 0)
            return
        if command_id == "minus_x":
            mf.moveXY(-self.disp_value, 0)
            return
        if command_id == "plus_y":
            mf.moveXY(0, +self.disp_value)
            return
        if command_id == "minus_y":
            mf.moveXY(0, -self.disp_value)
            return
        if command_id == "plus_z":
            mf.moveZ(+self.disp_value)
            return
        if command_id == "minus_z":
            mf.moveZ(-self.disp_value)
            return
        if command_id == "emergency_stop":
            mf.emergency_stop()
            return
        if command_id == "home_x":
            mf.home_axis("x")
            return
        if command_id == "home_y":
            mf.home_axis("y")
            return
        if command_id == "home_z":
            mf.home_axis("z")
            return
        if command_id == "home_all":
            mf.home_axis("all")
            return
        if command_id == "goto_origin":
            mf.goto_origin()
            return
        if command_id == "local_origin":
            mf.local_origin()
            return
        if command_id == "scringe":
            mf.start_scringe()
            return

        if command_id == "arrow_mode":
            # Instead of toggling arrow_mode in main frame,
            # we open a separate AxisControlFrame.
            AxisControlFrame(parent=self.mainframe, mainframe=self.mainframe, disp_value=self.disp_value)
            return

        if command_id == "play_pause":
            if mf.is_printing:
                mf.is_paused = not mf.is_paused
                st = "Paused" if mf.is_paused else "Resumed"
                mf.log(f"Print {st}.")
            else:
                mf.log("No active print to pause/resume.")
            return
        if command_id == "send_gcode":
            self.open_gcode_dialog()
            return
        if command_id == "settings":
            dlg = SettingsDialog(parent=mf, mainframe=mf)
            dlg.ShowModal()
            dlg.Destroy()
            return
        if command_id == "level_z":
            mf.level_z_axis()
            return

        mf.log(f"Unknown command: {command_id}")

    def open_gcode_dialog(self):
        dlg = wx.TextEntryDialog(
            parent=self.mainframe,
            message="Enter a G-code command.\n\n" + "\n".join(USEFUL_GCODE_COMMANDS),
            caption="Send G-code"
        )
        if dlg.ShowModal() == wx.ID_OK:
            cmd = dlg.GetValue().strip()
            if cmd:
                self.mainframe.send_gcode(cmd)
        dlg.Destroy()

# -------------------------------------------------------------------------
class SettingsDialog(wx.Dialog):
    """
    A wx.Dialog with fields for:
      - Pressure
      - Vacuum
      - Initial Delay
      - Travel Delay
      - Dispense Delay
    """
    def __init__(self, parent, mainframe):
        super().__init__(parent, title="Settings", size=(600, 500),
                         style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.mainframe = mainframe

        panel = wx.Panel(self)
        dlg_sizer = wx.BoxSizer(wx.VERTICAL)
        panel.SetSizer(dlg_sizer)

        # — Pressure row —
        pressure_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lblP = wx.StaticText(panel, label="Pressure (BAR):")
        pressure_sizer.Add(lblP, 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)

        # single-mode field
        self.txt_pressure = wx.TextCtrl(panel, value=str(mainframe.pressure))
        pressure_sizer.Add(self.txt_pressure, 1, wx.EXPAND|wx.RIGHT, 10)

        # dual-mode fields (start hidden)
        self.lbl_pressure_new = wx.StaticText(panel, label="1:")
        self.txt_pressure_new = wx.TextCtrl(panel, value=str(mainframe.pressure))
        self.lbl_pressure_old = wx.StaticText(panel, label="2:")
        self.txt_pressure_old = wx.TextCtrl(panel, value=str(mainframe.pressure))

        for w in (self.lbl_pressure_new, self.txt_pressure_new,
                self.lbl_pressure_old, self.txt_pressure_old):
            pressure_sizer.Add(w, 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)
            w.Hide()   # initially hidden

        self.btn_set_pressure = wx.Button(panel, label="Set Pressure")
        self.btn_set_pressure.Bind(wx.EVT_BUTTON, self.on_set_pressure)
        pressure_sizer.Add(self.btn_set_pressure, 0, wx.ALIGN_CENTER_VERTICAL)

        dlg_sizer.Add(pressure_sizer, 0, wx.EXPAND|wx.ALL, 5)

        # — Vacuum row —
        vacuum_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lblV = wx.StaticText(panel, label="Vacuum (H₂O):")
        vacuum_sizer.Add(lblV, 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)

        self.txt_vacuum = wx.TextCtrl(panel, value=str(mainframe.vacuum))
        vacuum_sizer.Add(self.txt_vacuum, 1, wx.EXPAND|wx.RIGHT, 10)

        self.lbl_vacuum_new = wx.StaticText(panel, label="1:")
        self.txt_vacuum_new = wx.TextCtrl(panel, value=str(mainframe.vacuum))
        self.lbl_vacuum_old = wx.StaticText(panel, label="2:")
        self.txt_vacuum_old = wx.TextCtrl(panel, value=str(mainframe.vacuum))

        for w in (self.lbl_vacuum_new, self.txt_vacuum_new,
                self.lbl_vacuum_old, self.txt_vacuum_old):
            vacuum_sizer.Add(w, 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)
            w.Hide()

        self.btn_set_vacuum = wx.Button(panel, label="Set Vacuum")
        self.btn_set_vacuum.Bind(wx.EVT_BUTTON, self.on_set_vacuum)
        vacuum_sizer.Add(self.btn_set_vacuum, 0, wx.ALIGN_CENTER_VERTICAL)

        dlg_sizer.Add(vacuum_sizer, 0, wx.EXPAND|wx.ALL, 5)

        # Single/ Dual Mode Dispenser
        mode_choices = ["single", "dual"]
        self.radio_mode = wx.RadioBox(
            panel,
            label="Dispenser Mode",
            choices=[c.capitalize() for c in mode_choices],
            majorDimension=1,
            style=wx.RA_SPECIFY_ROWS
        )
        self.radio_mode.Bind(wx.EVT_RADIOBOX, self.on_mode_changed)
        dlg_sizer.Add(self.radio_mode, 0, wx.ALL|wx.EXPAND, 5)

        # pick the right button:
        self.radio_mode.SetSelection(mode_choices.index(self.mainframe.dispenser_mode))

        # force a first layout so the correct widgets show/hide immediately:
        self.on_mode_changed(None)

        # Initial Delay
        init_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lbl_init = wx.StaticText(panel, label="Initial Delay (s):")
        init_sizer.Add(lbl_init, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.txt_init = wx.TextCtrl(panel, value=str(mainframe.initial_delay))
        init_sizer.Add(self.txt_init, 1, wx.EXPAND)
        dlg_sizer.Add(init_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Travel Delay
        travel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lbl_travel = wx.StaticText(panel, label="Travel Delay (s):")
        travel_sizer.Add(lbl_travel, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.txt_travel = wx.TextCtrl(panel, value=str(mainframe.travel_delay))
        travel_sizer.Add(self.txt_travel, 1, wx.EXPAND)
        dlg_sizer.Add(travel_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Dispense Delay
        dispense_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lbl_dispense = wx.StaticText(panel, label="Dispense Delay (s):")
        dispense_sizer.Add(lbl_dispense, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.txt_dispense = wx.TextCtrl(panel, value=str(mainframe.dispense_delay))
        dispense_sizer.Add(self.txt_dispense, 1, wx.EXPAND)
        dlg_sizer.Add(dispense_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # — Timed Dispense row —
        timed_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.chk_timed = wx.CheckBox(panel, label="Enable timed dispense")
        self.txt_interval = wx.TextCtrl(panel, value=str(mainframe.timed_dispense_interval), size=(50,-1))
        timed_sizer.Add(self.chk_timed, 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 10)
        timed_sizer.Add(wx.StaticText(panel, label="Interval (s):"), 0, wx.ALIGN_CENTER_VERTICAL|wx.RIGHT, 5)
        timed_sizer.Add(self.txt_interval, 0, wx.ALIGN_CENTER_VERTICAL)
        dlg_sizer.Add(timed_sizer, 0, wx.EXPAND|wx.ALL, 5)

        # set initial states
        self.chk_timed.SetValue(mainframe.timed_dispense_enabled)
        self.txt_interval.Enable(mainframe.timed_dispense_enabled)
        # toggle interval field when checkbox changes
        self.chk_timed.Bind(wx.EVT_CHECKBOX, self.on_toggle_timed)

        # Set Delays Button
        delay_btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        btn_set_delays = wx.Button(panel, label="Set Delays")
        btn_set_delays.Bind(wx.EVT_BUTTON, self.on_set_delays)
        delay_btn_sizer.Add(btn_set_delays, 0, wx.RIGHT, 10)
        dlg_sizer.Add(delay_btn_sizer, 0, wx.ALIGN_RIGHT | wx.ALL, 5)

        main_sizer = wx.BoxSizer(wx.VERTICAL)
        main_sizer.Add(panel, 1, wx.EXPAND | wx.ALL, 10)
        self.SetSizer(main_sizer)
        self.Layout()

    def on_toggle_timed(self, event):
        enabled = self.chk_timed.GetValue()
        self.txt_interval.Enable(enabled)

    def on_mode_changed(self, event):
        mode = self.radio_mode.GetStringSelection().lower()
        # if it actually changed:
        if mode != self.mainframe.dispenser_mode:
            self.mainframe.dispenser_mode = mode
            self.mainframe.dispenser = MultiDispenserController(mode=mode)
            self.mainframe.log(f"Switched to {mode.title()} mode.")
        # now show/hide the widgets...
        dual = (mode == 'dual')
        self.txt_pressure.Show(not dual)
        for w in (self.lbl_pressure_new,
                self.txt_pressure_new,
                self.lbl_pressure_old,
                self.txt_pressure_old):
            w.Show(dual)
        self.txt_vacuum.Show(not dual)
        for w in (self.lbl_vacuum_new,
                self.txt_vacuum_new,
                self.lbl_vacuum_old,
                self.txt_vacuum_old):
            w.Show(dual)
        self.Layout()

    def on_set_pressure(self, event):
        mode = self.radio_mode.GetStringSelection().lower()
        try:
            if mode == 'dual':
                p_new = float(self.txt_pressure_new.GetValue())
                p_old = float(self.txt_pressure_old.GetValue())
                run_both(
                    lambda: self.mainframe.dispenser.new.set_pressure(p_new),
                    lambda: self.mainframe.dispenser.old.set_pressure(p_old)
                )
                self.mainframe.log(f"NEW={p_new}, OLD={p_old}")
            else:
                p = float(self.txt_pressure.GetValue())
                self.mainframe.dispenser.dispenser_callback(f"pressure {p}")
                self.mainframe.log(f"Pressure set to {p}")
        except ValueError:
            self.mainframe.log("Invalid pressure value.")

    def on_set_vacuum(self, event):
        mode = self.radio_mode.GetStringSelection().lower()
        try:
            if mode == 'dual':
                v_new = float(self.txt_vacuum_new.GetValue())
                v_old = float(self.txt_vacuum_old.GetValue())
                run_both(
                    lambda: self.mainframe.dispenser.new.set_vacuum(v_new),
                    lambda: self.mainframe.dispenser.old.set_vacuum(v_old)
                )
                self.mainframe.log(f"NEW={v_new}, OLD={v_old}")
            else:
                v = float(self.txt_vacuum.GetValue())
                self.mainframe.dispenser.dispenser_callback(f"vacuum {v}")
                self.mainframe.log(f"Vacuum set to {v}")
        except ValueError:
            self.mainframe.log("Invalid vacuum value.")

    def on_set_delays(self, event):
        try:
            init_val = float(self.txt_init.GetValue())
            trav_val = float(self.txt_travel.GetValue())
            disp_val = float(self.txt_dispense.GetValue())
            interval = float(self.txt_interval.GetValue())

            self.mainframe.initial_delay = init_val
            self.mainframe.travel_delay = trav_val
            self.mainframe.dispense_delay = disp_val
            self.mainframe.timed_dispense_enabled  = self.chk_timed.GetValue()
            self.mainframe.timed_dispense_interval = interval

            self.mainframe.log(
                f"Delays updated: initial={init_val}, travel={trav_val}, dispense={disp_val}"
            )
            self.mainframe.log(f"Timed‐dispense {'ON' if self.mainframe.timed_dispense_enabled else 'OFF'} @ {interval:.2f}s")

            # --- Check if dispenser mode changed ---
            selected_mode = self.radio_mode.GetStringSelection().lower()
            if selected_mode != self.mainframe.dispenser_mode:
                self.mainframe.dispenser_mode = selected_mode
                self.mainframe.dispenser = MultiDispenserController(
                    mode=selected_mode
                )
                self.mainframe.log(f"Switched to {selected_mode.title()} mode.")
        except ValueError:
            self.mainframe.log("Invalid numeric value in delays or interval.")

class RegistrationDialog(wx.Dialog):
    """
    A pop-up dialog that shows:
      • Corner 1–4 text fields and Register buttons
      • Clear All
      • Compute Transformation
    It reads/writes into parent.corners (a list of length 4).
    """

    def __init__(self, parent, title="Registration"):
        super().__init__(parent, title=title, size=(350, 300))
        self.mainframe = parent

        panel = wx.Panel(self)
        sizer = wx.BoxSizer(wx.VERTICAL)
        panel.SetSizer(sizer)

        # Keep references to the text controls so we can update them later
        self.corner_txt = []
        corner_labels = ["BL", "BR", "TR", "TL"]
        for i in range(4):
            row = wx.BoxSizer(wx.HORIZONTAL)
            lbl = wx.StaticText(panel, label=f"{corner_labels[i]}:")
            txt = wx.TextCtrl(panel,
                            value="",
                            size=(120, -1),
                            style=wx.TE_READONLY)
            btn = wx.Button(panel, label="Register")
            btn.Bind(wx.EVT_BUTTON, lambda evt, idx=i: self.on_register(idx))
            row.Add(lbl, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
            row.Add(txt, 0, wx.RIGHT, 5)
            row.Add(btn, 0)
            sizer.Add(row, 0, wx.ALL, 5)
            self.corner_txt.append(txt)

        # ── New: “Register Initial Position” button ──
        init_row = wx.BoxSizer(wx.HORIZONTAL)
        btn_init = wx.Button(panel, label="Register Initial Position")
        btn_init.Bind(wx.EVT_BUTTON, self.on_register_initial)
        init_row.Add(btn_init, 1, wx.EXPAND | wx.ALL, 5)
        sizer.Add(init_row, 0, wx.EXPAND)

        # ── Final row: Clear All and Compute Transformation ──
        btn_row = wx.BoxSizer(wx.HORIZONTAL)

        btn_clear = wx.Button(panel, label="Clear All")
        btn_clear.Bind(wx.EVT_BUTTON, self.on_clear)

        btn_compute = wx.Button(panel, label="Compute Transformation")
        btn_compute.Bind(wx.EVT_BUTTON, self.on_compute)

        btn_row.Add(btn_clear, 1, wx.EXPAND | wx.RIGHT, 5)
        btn_row.Add(btn_compute, 1, wx.EXPAND)

        sizer.Add(btn_row, 0, wx.EXPAND | wx.ALL, 5)


        # Populate existing corner values (if any) from mainframe.corners
        self._load_existing()

    def on_register_initial(self, event):
        """
        Called when “Register Initial Position” is clicked.
        Delegate to mainframe to do an M114 and log/store the initial XYZ.
        """
        self.mainframe.on_register_initial()


    def _load_existing(self):
        """
        Read self.mainframe.corners and populate each text field
        so that if you re-open this dialog, you still see previous values.
        """
        for idx, corner in enumerate(self.mainframe.corners):
            if corner is not None:
                x, y, z = corner
                self.corner_txt[idx].SetValue(f"{x:.2f}, {y:.2f}, {z:.2f}")

    def on_register(self, idx):
        """
        Called when “Register” for corner idx is clicked.
        Delegates to mainframe.on_register_corner(idx), then updates the text box.
        """
        # 1) Let the mainframe query M114, parse, store into self.mainframe.corners[idx]
        self.mainframe.on_register_corner(idx)

        # 2) If mainframe.corners[idx] was set, update our text ctrl
        c = self.mainframe.corners[idx]
        if c is not None:
            x, y, z = c
            self.corner_txt[idx].SetValue(f"{x:.2f}, {y:.2f}, {z:.2f}")

    def on_clear(self, event):
        """
        “Clear All” clicked: tell mainframe to clear its list of corners,
        then wipe all text fields in this dialog.
        """
        self.mainframe.on_clear_corners(event=None)
        for txt in self.corner_txt:
            txt.SetValue("")

    def on_compute(self, event):
        """
        “Compute Transformation” clicked: delegate to mainframe to compute origin+angle.
        """
        self.mainframe.on_compute_transformation(event=None)


class BioPrinterApp(wx.App):
    def OnInit(self):
        frame = BioPrinterMainFrame(None)
        frame.Show()
        return True

def main():
    app = BioPrinterApp(False)
    app.MainLoop()

if __name__ == "__main__":
    main()
