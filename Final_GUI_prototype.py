#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import wx
import os
import serial
import serial.tools.list_ports
import time
import threading
import math
import logging

import matplotlib
matplotlib.use("WXAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas

from PIL import Image

ASSETS_PATH = "/home/rami/Downloads/assets"

USEFUL_GCODE_COMMANDS = [
    "M114   (Get current position)",
    "M503   (Report current settings)",
    "M105   (Read extruder hotend temp)",
    "M112   (Emergency stop)",
    "G28    (Home all axes)",
    "G90    (Absolute positioning)",
    "G91    (Relative positioning)",
    "G1 X10 Y5 F3000 (Move X+10, Y+5 at feed=3000)",
]

# We'll keep global x_coords, y_coords, z_coords, F_coords
x_coords = []
y_coords = []
z_coords = []
F_coords = []

# --- Dispenser Controller Code (unchanged) ---
class DispenserController:
    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger('DispenserController')
        self.ser = None
        self.connect()
        self.is_timed_mode = True
        self.logger.debug('Dispenser Controller Started')

    def connect(self):
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
                self.ser.write(b'\x05')
                time.sleep(0.1)
                ack_response = self.ser.read(1)
                if ack_response != b'\x06':
                    self.logger.warning("Did not receive ACK after ENQ.")
                    return

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

    def dispenser_callback(self, command_str):
        if command_str == 'start':
            self.send_command('DI  ', '')
        elif command_str == 'stop':
            self.send_command('DI  ', '')
        elif command_str.startswith('pressure '):
            try:
                pressure_value = float(command_str.split(' ')[1])
                if 0.0 <= pressure_value <= 650.0:
                    formatted_pressure = f"{int(pressure_value * 10):04d}"
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

        self.SetTitle("ARTS BioPrinter")
        self.SetSize((1400, 800))
        self.Centre()

        self.remote_socket = None
        self.dispenser = DispenserController()
        self.printer_serial = None
        self.is_paused = False
        self.is_printing = False
        self.stop_flag = False

        # Some parameters
        self.pressure = 2.0
        self.vacuum = 2.0
        self.initial_delay = 5.0
        self.travel_delay = 0.02
        self.dispense_delay = 0.05

        # MULTI-LAYER PLOT SETTINGS
        self.num_layers = 1
        self.layer_height = 0.3
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

        import socket
        try:
            self.remote_socket = socket.create_connection((remote_host, remote_port), timeout=5)
            self.log(f"Connected to remote printer at {remote_host}:{remote_port}")
        except Exception as e:
            self.log(f"Remote connection failed: {e}")
            self.remote_socket = None

    def init_layout(self):
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        self.SetSizer(main_sizer)

        # -- TOP ROW with serial port controls & load file --
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

        btn_load_txt = wx.Button(top_panel, label="Load File")
        btn_load_txt.Bind(wx.EVT_BUTTON, self.on_load_txt_file)
        top_sizer.Add(btn_load_txt, 0, wx.RIGHT, 5)

        btn_start_print = wx.Button(top_panel, label="Start Print")
        btn_start_print.Bind(wx.EVT_BUTTON, self.on_start_print)
        top_sizer.Add(btn_start_print, 0, wx.RIGHT, 5)

        # -- LAYER CONTROL --
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

        btn_remote_connect = wx.Button(top_panel, label="Remote Connect")
        btn_remote_connect.Bind(wx.EVT_BUTTON, self.on_remote_connect)
        top_sizer.Add(btn_remote_connect, 0, wx.RIGHT, 5)

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
        dlg = wx.FileDialog(self, "Open .txt file with X/Y/Z/F lines",
                            wildcard="Text files (*.txt)|*.txt|All files|*.*",
                            style=wx.FD_OPEN|wx.FD_FILE_MUST_EXIST)
        if dlg.ShowModal() == wx.ID_CANCEL:
            return
        path = dlg.GetPath()
        dlg.Destroy()

        try:
            x_coords.clear()
            y_coords.clear()
            z_coords.clear()
            F_coords.clear()
            with open(path, "r") as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) == 4:
                        x, y, z, fv = parts
                        x = float(x)
                        y = float(y)
                        z = float(z)
                        fv = float(fv)
                        print(f"Read: X={x}, Y={y}, Z={z}, F={fv}")  # Debug
                        x_coords.append(x)
                        y_coords.append(y)
                        z_coords.append(z)
                        F_coords.append(fv)
            self.log(f"Loaded {len(x_coords)} coords from {path}. Now plotting multi-layer.")
            self.plot_initial()
        except Exception as e:
            self.log(f"Error loading file: {e}")

    def on_start_print(self, event):
        if self.is_printing:
            self.log("Already printing!")
            return
        self.is_printing = True
        self.stop_flag = False
        self.log("Printing...")

        t = threading.Thread(target=self.run_printing)
        t.daemon = True
        t.start()

    def run_printing(self):
        self.log("Applying initial delay and moving to start...")
        if not x_coords:
            self.log("No coordinates loaded.")
            self.is_printing = False
            return

        # Move to first coordinate
        self.send_gcode(f"G1 X{x_coords[0]} Y{y_coords[0]} Z{z_coords[0]} F{F_coords[0]}")
        time.sleep(self.initial_delay) # Allow setup time

        self.log("Starting multi-layer print...")

        for i in range(len(x_coords)): # This will iterate through every coordinate in the file
            if self.stop_flag:
                self.log("Printing Stopped by user/emergency stop.")
                self.is_printing = False
                return
            
            # Define travel path range (based on the line numbers in the original file)
            #travel_start = 1663
            #travel_end = 1748

            #if not (travel_start <= i <= travel_end):
            #    if i == 0:  # Fix condition to activate at start and end
            #        self.dispenser.dispenser_callback('start')
            
            # Move to coordinate
            self.send_gcode(f"G1 X{x_coords[i]} Y{y_coords[i]} Z{z_coords[i]} F{F_coords[i]}")
            time.sleep(self.travel_delay)  # Use self.travel_delay (set this to ~0.05s or more)

            # Move to coordinate (with layer offset)
            # self.send_gcode(f"G1 X{x_coords[i]} Y{y_coords[i]} Z{z_coords[i] + j*self.layer_height} F{F_coords[i]}")

            # Embedded Dispense logic
            # time.sleep(self.dispense_delay)
            # self.dispenser.dispenser_callback('start')
            # time.sleep(self.dispense_delay)
            # self.dispenser.dispenser_callback('start')
            # Travel delay
            # time.sleep(self.travel_delay)

        # After finishing a layer, update the color for that layer
        # wx.CallAfter(self.update_layer_color, i)

        # Wrap-up
        self.dispenser.dispenser_callback('start')
        self.send_gcode("G90")
        if self.printer_serial:
            self.printer_serial.flush()
            # self.printer_serial.close()  # only if you want to close the port
        self.log("Printing done.")
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
        global x_coords, y_coords, z_coords
        self.ax.clear()
        self.ax.set_xlabel("X-axis")
        self.ax.set_ylabel("Y-axis")
        self.ax.set_zlabel("Z-axis")

        for j in range(self.num_layers):
            color = self.colors[j % len(self.colors)]
            z_layer = [z + j*self.layer_height for z in z_coords]
            self.ax.plot(
                x_coords, y_coords, z_layer,
                marker='.', color=color, alpha=0.2, markersize=2
            )
        self.ax.plot(
            x_coords[0], y_coords[0], z_layer[0],
                marker='*', color="b", alpha=1, markersize=10)
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

        # Pressure
        pressure_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lbl_pressure = wx.StaticText(panel, label="Pressure (BAR*0.01):")
        pressure_sizer.Add(lbl_pressure, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.txt_pressure = wx.TextCtrl(panel, value=str(mainframe.pressure))
        pressure_sizer.Add(self.txt_pressure, 1, wx.EXPAND)
        btn_set_pressure = wx.Button(panel, label="Set Pressure")
        btn_set_pressure.Bind(wx.EVT_BUTTON, self.on_set_pressure)
        pressure_sizer.Add(btn_set_pressure, 0, wx.LEFT, 10)
        dlg_sizer.Add(pressure_sizer, 0, wx.EXPAND | wx.ALL, 5)

        # Vacuum
        vacuum_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lbl_vacuum = wx.StaticText(panel, label="Vacuum (H20):")
        vacuum_sizer.Add(lbl_vacuum, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 5)
        self.txt_vacuum = wx.TextCtrl(panel, value=str(mainframe.vacuum))
        vacuum_sizer.Add(self.txt_vacuum, 1, wx.EXPAND)
        btn_set_vacuum = wx.Button(panel, label="Set Vacuum")
        btn_set_vacuum.Bind(wx.EVT_BUTTON, self.on_set_vacuum)
        vacuum_sizer.Add(btn_set_vacuum, 0, wx.LEFT, 10)
        dlg_sizer.Add(vacuum_sizer, 0, wx.EXPAND | wx.ALL, 5)

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

    def on_set_pressure(self, event):
        try:
            val = float(self.txt_pressure.GetValue())
            self.mainframe.pressure = val
            self.mainframe.dispenser.dispenser_callback(f"pressure {val}")
            self.mainframe.log(f"Pressure set to {val}")
        except ValueError:
            self.mainframe.log("Invalid pressure value.")

    def on_set_vacuum(self, event):
        try:
            val = float(self.txt_vacuum.GetValue())
            self.mainframe.vacuum = val
            self.mainframe.dispenser.dispenser_callback(f"vacuum {val}")
            self.mainframe.log(f"Vacuum set to {val}")
        except ValueError:
            self.mainframe.log("Invalid vacuum value.")

    def on_set_delays(self, event):
        try:
            init_val = float(self.txt_init.GetValue())
            trav_val = float(self.txt_travel.GetValue())
            disp_val = float(self.txt_dispense.GetValue())

            self.mainframe.initial_delay = init_val
            self.mainframe.travel_delay = trav_val
            self.mainframe.dispense_delay = disp_val
            self.mainframe.log(
                f"Delays updated: initial={init_val}, travel={trav_val}, dispense={disp_val}"
            )
        except ValueError:
            self.mainframe.log("Invalid delay value.")


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
