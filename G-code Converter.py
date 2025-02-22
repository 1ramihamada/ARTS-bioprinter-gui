import re
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Example usage:
file_path = "/home/rami/Downloads/MultidrictionSensorChannel_0.4n_0.2mm_PETG_MK4IS_1m.txt"
output_path_txt = "/home/rami/Downloads/New_MultidrictionSensorChannel_0.4n_0.2mm_PETG_MK4IS_1m.txt"

threshold_distance = 1

def distance(p1, p2):
    """Euclidean distance in 2D (for the XY-plane)."""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def insert_segments(start, end, threshold):
    """
    Inserts intermediate points if distance > threshold.
    Returns a list of (x, y) points: [start, ..., end].
    """
    points = [start]
    dist = distance(start, end)
    if dist > threshold:
        num_segments = int(math.ceil(dist / threshold))
        for i in range(1, num_segments):
            t = i / num_segments
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            points.append((x, y))
    points.append(end)
    return points

def extract_coordinates(file_path, threshold):
    """
    Reads G-code, extracts (X, Y, Z, F, Height).
    Inserts extra XY points if distance > threshold.
    Returns a list of dicts: [{X, Y, Z, F, Height}, ...].
    """
    with open(file_path, 'r') as infile:
        lines = infile.readlines()

    coordinates = []
    last_x, last_y, last_z, last_height, last_F = 0, 0, 0, 0, 0

    for line in lines:
        line = line.strip()

        # 1) Capture Height from e.g. ";Z:0.2"
        if line.startswith(';Z'):
            height_match = re.search(r':([-+]?\d*\.\d+|\d+)', line)
            if height_match:
                last_height = float(height_match.group(1))

        # 2) Capture G0/G1 moves
        elif line.startswith('G1') or line.startswith('G0'):
            x_match = re.search(r'X([-+]?\d*\.\d+|\d+)', line)
            y_match = re.search(r'Y([-+]?\d*\.\d+|\d+)', line)
            z_match = re.search(r'Z([-+]?\d*\.\d+|\d+)', line)
            F_match = re.search(r'F([-+]?\d*\.\d+|\d+)', line)

            if x_match:
                new_x = float(x_match.group(1))
            else:
                new_x = last_x

            if y_match:
                new_y = float(y_match.group(1))
            else:
                new_y = last_y

            if z_match:
                new_z = float(z_match.group(1))
            else:
                new_z = last_z

            if F_match:
                new_F = float(F_match.group(1))
            else:
                new_F = last_F

            # 3) Insert segments if needed
            if last_z == last_height and (new_x != last_x or new_y != last_y):
                segments = insert_segments((last_x, last_y), (new_x, new_y), threshold)
                for seg_x, seg_y in segments:
                    coordinates.append({
                        'X': seg_x,
                        'Y': seg_y,
                        'Z': last_z,
                        'F': new_F,
                        'Height': last_height
                    })
            else:
                coordinates.append({
                    'X': new_x,
                    'Y': new_y,
                    'Z': new_z,
                    'F': new_F,
                    'Height': last_height
                })

            # Update "last" values
            last_x, last_y, last_z, last_F = new_x, new_y, new_z, new_F

    return coordinates

def export_to_txt(coordinates, output_path):
    """
    Writes the coordinates to a text file WITHOUT a header line,
    and WITHOUT the Height column.

    So the output columns are:
      X, Y, Z, F
    """
    with open(output_path, 'w') as f:
        # No header line
        for c in coordinates:
            x = c['X']
            y = c['Y']
            z = c['Z']
            f_val = c['F']
            # omit c['Height']
            f.write(f"{x:.3f} {y:.3f} {z:.3f} {f_val:.2f}\n")

###############################################################################
# MAIN
###############################################################################
coords = extract_coordinates(file_path, threshold_distance)

# Write to a text file (no header, only X/Y/Z/F columns)
export_to_txt(coords, output_path_txt)

print(f"Coordinates have been exported to {output_path_txt}")
print(f"Number of points: {len(coords)}")

# Debug-print in terminal
for i, coord in enumerate(coords[:10], start=1):
    print(f"Line {i}: X={coord['X']}, Y={coord['Y']}, Z={coord['Z']}, F={coord['F']}, Height={coord['Height']}")

# Separate X, Y, Z into lists for plotting (ignoring F, Height)
X = [c['X'] for c in coords]
Y = [c['Y'] for c in coords]
Z = [c['Z'] for c in coords]

# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(X, Y, Z, marker='o', linestyle='-', color='b')

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('3D Plot of X, Y, Z Coordinates')

plt.show()
