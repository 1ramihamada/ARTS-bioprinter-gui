# ARTS BioPrinter Controller

## Overview
The **ARTS BioPrinter Controller** is a Python-based application that provides a GUI interface for controlling a bioprinter. The software facilitates communication with a Prusa 3D printer and Nordson fluid dispenser via serial ports, allowing users to move axes, send G-code commands, and execute precise dispensing operations.

## Features
- **Graphical Interface**: Built using `wxPython` for interactive control.
- **Serial Communication**: Uses `pyserial` to interface with dispensers and 3D printers.
- **Live 3D Visualization**: Displays printing paths with `matplotlib`.
- **Custom G-code Execution**: Send manual G-code commands to the printer.
- **Remote Connection Support**: Enables remote control over a network.
- **Multi-Layer Printing**: Handles multiple layers for 3D bio-printing.
- **Emergency Stop**: Provides a safety stop mechanism for both dispenser and printer.

## Requirements
- Python 3.x
- Nordson Ultimus V
- Prusa i3 MK3S+

## Installation
1. Clone the repository:
   ```sh
   git clone https://github.com/your-repo/bioprinter-controller.git
   cd bioprinter-controller
   ```
2. Install dependencies:
  # Ubuntu:
  ```sh
  pip install wxPython pyserial matplotlib pillow
  ```
  # Windows:
  ```sh
  py -m pip install --upgrade pip
  py -m pip install wxPython pyserial matplotlib pillow
  ```
3. Ensure your printer and dispenser are connected via USB.

## Usage
Run the application with:
```sh
python bioprinter_controller.py
```

### Key Functionalities
#### 1. **Dispenser Control**
- Connects to available serial ports
- Sends pressure, vacuum, and dispense commands
- Supports emergency stop functionality

#### 2. **Printer Control**
- Loads and sends G-code commands
- Allows movement using arrow keys or GUI buttons
- Supports homing (`G28`), positioning (`G1`), and other G-code commands

#### 3. **Graphical Features**
- 3D visualization of printing paths
- Adjustable displacement settings for manual control
- Remote control support via TailScale

### Example G-code Commands
```
G28    ; Home all axes
G1 X10 Y5 F3000  ; Move X+10, Y+5 at feedrate 3000 mm/min
M112   ; Emergency stop
```

### Emergency Stop
To stop the bioprinter and dispenser immediately:
1. Press the **Emergency Stop** button in the GUI
2. Or send the command:
   ```sh
   M112  # Printer emergency stop
   ````

## License
This project is licensed under the MIT License.
