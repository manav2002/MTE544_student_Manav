import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader


def plot_paths():
    # Load robotPose.csv
    headers, values = FileReader("robotPose.csv").read_file()

    # Get x and y values
    kf_x_index = headers.index("kf_x")
    kf_y_index = headers.index("kf_y")

    executed_x = [line[kf_x_index] for line in values]
    executed_y = [line[kf_y_index] for line in values]

    # Load the generated path (exported from planner.py as Path)
    try:
        generated_path = np.loadtxt("generated_path.csv", delimiter=",")
        generated_x = generated_path[:, 0]
        generated_y = generated_path[:, 1]
    except Exception as e:
        print(f"Error loading generated path: {e}")
        return

    # Plot both paths
    plt.figure(figsize=(10, 8))

    # Plot executed path
    plt.plot(executed_x, executed_y, label="Executed Path", linestyle="--", marker="o", color="blue")

    # Plot generated path
    plt.plot(generated_x, generated_y, label="Generated Path", linestyle="-", marker="x", color="red")

    # Add title, label, legend, grid
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title("Comparison of Generated and Executed Paths")
    plt.legend()
    plt.grid(True)

    # Show the plot
    plt.show()


if __name__ == "__main__":
    plot_paths()
