import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_xyz_from_file(filename):
    xs, ys, zs = [], [], []

    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    xs.append(x)
                    ys.append(y)
                    zs.append(z)
                except ValueError:
                    continue  # skip malformed lines

    # Create 3D plot
    print(f"Loaded {len(xs)} points")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs, ys, zs, s=5)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Scatter Plot of XYZ Points')
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    plot_xyz_from_file("/home/rami/Downloads/Converted_Mo_version.txt")  # Replace with your actual filename
