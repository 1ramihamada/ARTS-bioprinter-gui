# GUI V1
# Printing on surface

import serial
import logging
import serial.tools.list_ports
import time
import matplotlib
matplotlib.use("TkAgg")  # Use TkAgg backend for embedding in Tkinter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import threading

# --- Dispenser Controller Code (unchanged) ---
class DispenserController:
    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger('DispenserController')
        self.ser = None
        self.connect()
        self.is_timed_mode = True
        self.logger.info('Dispenser Controller Started')

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
                self.logger.info(f'Connected to {port.device}')
                time.sleep(2)
                return
            except serial.SerialException as e:
                self.logger.error(f"Error connecting to {port.device}: {e}")
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
                if 0.0 <= pressure_value <= 300.0:
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

# --- Original Code Setup ---
SERIAL_PORT = '/dev/ttyACM0'  # Adjust if needed
BAUD_RATE = 115200
COORDINATES_FILE_PATH = '/home/rami/Downloads/LongHorn Edge 4.txt'  # Adjust if needed

x_coords = []
y_coords = []
z_coords = []
F_coords = []

try:
    with open(COORDINATES_FILE_PATH, 'r') as file:
        for line in file:
            coordinates = line.strip().split()
            if len(coordinates) == 4:
                try:
                    x = float(coordinates[0])
                    y = float(coordinates[1])
                    z = float(coordinates[2])
                    F = int(coordinates[3])
                    x_coords.append(x)
                    y_coords.append(y)
                    z_coords.append(z)
                    F_coords.append(F)
                except ValueError:
                    print(f"Invalid line: {line.strip()}")
            else:
                print(f"Invalid line: {line.strip()}")
except FileNotFoundError:
    print(f"File not found: {COORDINATES_FILE_PATH}")
except Exception as e:
    print(f"An error occurred: {e}")

# Transform coordinates as before
x_coords = [2*x-200 for x in x_coords]
y_coords = [2*y-100 for y in y_coords]
xx = []
yy = []
zz = []
for i in range(len(x_coords)):
    if i == 0 or (x_coords[i] - xx[-1])**2 + (y_coords[i] - yy[-1])**2 > 3**2:
        xx.append(x_coords[i])
        yy.append(y_coords[i])
        zz.append(z_coords[i])
x_coords, y_coords, z_coords = xx, yy, zz

# Connect to the printer
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
time.sleep(10)  # Wait for the printer to initialize
ser.write(b"G90\n")  # absolute positioning
ser.flush()

def send_gcode(cmd):
    ser.write((cmd + "\n").encode('utf-8'))
    ser.flush()

def read_response():
    response = ser.readlines()
    for line in response:
        print(line.decode('utf-8').strip())

send_gcode("G92 X0 Y0 Z0")

dispenser = DispenserController()
dispenser.dispenser_callback('pressure 75') # initial pressure

# --- GUI ---
class PrintGUI:
    def __init__(self, master):
        self.master = master
        master.title("3D Print GUI")

        # Initialize the stop_flag
        self.stop_flag = False

        # Variables for delays and pressure/vacuum
        self.initial_delay = tk.DoubleVar(value=10.0)  
        self.travel_delay = tk.DoubleVar(value=1.0)
        self.dispense_delay = tk.DoubleVar(value=0.05)
        self.pressure = tk.DoubleVar(value=75.0)
        self.vacuum = tk.DoubleVar(value=5.0)
        self.status_text = tk.StringVar(value="Status: Idle")

        # Create frames
        control_frame = ttk.Frame(master)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        plot_frame = ttk.Frame(master)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Emergency Stop Button with custom style
        stop_button = tk.Button(
            control_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            bg="red",         # Button background color
            fg="white",       # Button text color
            font=("Helvetica", 16, "bold"),  # Font for visibility
            relief=tk.RAISED,  # Button style (raised for emphasis)
            padx=10,           # Padding for larger button
            pady=5
        )
        stop_button.pack(anchor=tk.W, fill=tk.X, pady=20)  # Padding around the button

        # Controls
        ttk.Label(control_frame, text="Pressure (BAR * 0.01):").pack(anchor=tk.W)
        tk.Entry(control_frame, textvariable=self.pressure).pack(anchor=tk.W, fill=tk.X)

        ttk.Label(control_frame, text="Vacuum (H2O):").pack(anchor=tk.W)
        tk.Entry(control_frame, textvariable=self.vacuum).pack(anchor=tk.W, fill=tk.X)

        ttk.Label(control_frame, text="Initial Delay (s):").pack(anchor=tk.W)
        tk.Entry(control_frame, textvariable=self.initial_delay).pack(anchor=tk.W, fill=tk.X)

        ttk.Label(control_frame, text="Travel Delay (s):").pack(anchor=tk.W)
        tk.Entry(control_frame, textvariable=self.travel_delay).pack(anchor=tk.W, fill=tk.X)

        ttk.Label(control_frame, text="Dispense Delay (s):").pack(anchor=tk.W)
        tk.Entry(control_frame, textvariable=self.dispense_delay).pack(anchor=tk.W, fill=tk.X)

        ttk.Label(control_frame, textvariable=self.status_text).pack(anchor=tk.W, pady=10)

        start_button = ttk.Button(control_frame, text="Start Print", command=self.start_print)
        start_button.pack(anchor=tk.W, fill=tk.X, pady=5)

        set_pressure_button = ttk.Button(control_frame, text="Set Pressure", command=self.set_pressure)
        set_pressure_button.pack(anchor=tk.W, fill=tk.X, pady=5)

        set_vacuum_button = ttk.Button(control_frame, text="Set Vacuum", command=self.set_vacuum)
        set_vacuum_button.pack(anchor=tk.W, fill=tk.X, pady=5)

        home_button = ttk.Button(control_frame, text="Home", command=self.send_home)
        home_button.pack(anchor=tk.W, fill=tk.X, pady=5)

        # Plot Setup
        self.fig = plt.Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Plot initial layers in lighter colors
        self.num_layers = 3
        self.layer_height = 0.3
        self.colors = ["red", "orange", "yellow", "lime", "cyan", "blue", "blueviolet", "violet", "magenta"]
        self.plot_initial()

    def plot_initial(self):
        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('3D Plot of Coordinates')

        for j in range(self.num_layers):
            color = self.colors[j % len(self.colors)]
            z_layer = [z + j * self.layer_height for z in z_coords]
            # Use the alpha parameter for transparency
            self.ax.plot(x_coords, y_coords, z_layer, marker='o', color=color, alpha=0.2)
        self.canvas.draw()

    def lighten_color(self, color, amount=0.5):
        import matplotlib.colors as mcolors
        c = mcolors.to_rgb(color)
        c_light = [1 - (1 - x)*amount for x in c]
        return c_light

    def set_pressure(self):
        p = self.pressure.get()
        dispenser.dispenser_callback(f'pressure {p}')

    def set_vacuum(self):
        v = self.vacuum.get()
        dispenser.dispenser_callback(f'vacuum {v}')

    def send_home(self):
        send_gcode("G1 X0 Y0 Z0")

    def emergency_stop(self):
        """
        Trigger the emergency stop functionality.
        """
        self.stop_flag = True  # Stop any ongoing printing process
        dispenser.emergency_stop()  # Call the dispenser's emergency stop
        self.status_text.set("Status: Emergency Stop Triggered!")


    def start_print(self):
        self.status_text.set("Status: Printing...")
        t = threading.Thread(target=self.run_printing)
        t.start()

    def run_printing(self):
        # Set status to printing
        self.status_text.set("Status: Applying initial delay...")

        # Apply the initial delay
        send_gcode(f"G1 X{x_coords[0]} Y{y_coords[0]} Z{z_coords[0]} F{F_coords[0]}")
        time.sleep(self.initial_delay.get())

        # Update status to indicate printing has started
        self.status_text.set("Status: Printing...")

        for j in range(self.num_layers):
            if self.stop_flag:  # Check for emergency stop
                self.status_text.set("Status: Printing Stopped.")
                return
            for i in range(len(x_coords)):
                if self.stop_flag:  # Check for emergency stop
                    self.status_text.set("Status: Printing Stopped.")
                    return
                # Move to the next coordinate
                send_gcode(f"G1 X{x_coords[i]} Y{y_coords[i]} Z{z_coords[i] + 0.3 * j} F{F_coords[i]}")
                time.sleep(self.dispense_delay.get())  # Dispense delay
                dispenser.dispenser_callback('start')  # Start dispensing
                time.sleep(self.dispense_delay.get())  # Additional dispense delay
                dispenser.dispenser_callback('start')  # Start dispensing again
                time.sleep(self.travel_delay.get())  # Travel delay

            # After finishing a layer, update the layer color
            self.master.after(0, self.update_layer_color, j)

        # Once all layers are done
        ser.write(b"G90\n")
        ser.flush()
        ser.close()

        # Update status to indicate completion
        self.master.after(0, self.status_text.set, "Status: Complete")


    def update_layer_color(self, layer_idx):
        # Redraw the plot showing completed layers in full color and future layers as transparent
        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('3D Plot of Coordinates')

        for j in range(self.num_layers):
            z_layer = [z + j * self.layer_height for z in z_coords]
            if j <= layer_idx:
                # This layer is done, use full color
                color = self.colors[j % len(self.colors)]
                self.ax.plot(x_coords, y_coords, z_layer, marker='o', color=color, alpha=1.0)
            else:
                # Future layers as transparent
                color = self.colors[j % len(self.colors)]
                self.ax.plot(x_coords, y_coords, z_layer, marker='o', color=color, alpha=0.2)

        self.canvas.draw()


root = tk.Tk()
app = PrintGUI(root)
root.mainloop()
