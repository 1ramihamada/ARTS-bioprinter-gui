import subprocess

CURA_ENGINE_SEARCH_PATH="/home/rami/Downloads"
CURA_ENGINE_BINARY = "/home/rami/CuraEngine/build/Release/CuraEngine"
BASE_PROFILE_JSON = "/home/rami/Downloads/printer_profile/prusa_i3_mk3.def.json"
STL_FILE = "/home/rami/Downloads/shiba.stl"
OUTPUT_GCODE = "/home/rami/Downloads/shiba.gcode"

command = [
    CURA_ENGINE_BINARY,
    "slice",
    "-d", CURA_ENGINE_SEARCH_PATH,
    "-j", BASE_PROFILE_JSON,
    "-l", STL_FILE,
    "-o", OUTPUT_GCODE,
    "-s", "layer_height=0.3",
    "-s", "wall_thickness=1.2",
    "-s", "fill_density=20",
    "-s", "infill_pattern=linear",
    "-s", "print_speed=60",
    "-s", "gcode_flavor=reprap",
    "-s", "gcode_comments=false"
]


try:
    subprocess.run(command, check=True)
    print(f"G-code successfully generated and saved to {OUTPUT_GCODE}")
except subprocess.CalledProcessError as e:
    print(f"Error while slicing: {e}")
