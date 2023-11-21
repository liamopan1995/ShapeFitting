import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def read_file(filename, color):
    # Lists to store circles
    circles = []

    # Open the file and read the data
    with open(filename, 'r') as file:
        for line in file:
            # Split each line by whitespace to get the individual values
            parts = line.split()
            if len(parts) >= 4:  # Ensure the line has timestamp, x, y, and r
                x = float(parts[1])
                y = float(parts[2])
                r = float(parts[3])
                circles.append(Circle((x, y), r, fill=False, color=color))

    return circles

def plot_files(filenames, colors):
    # Create a figure and axis for plotting
    fig, ax = plt.subplots()

    all_circles = []
    for filename, color in zip(filenames, colors):
        circles = read_file(filename, color)
        all_circles.extend(circles)
        for circle in circles:
            ax.add_artist(circle)

    # Set limits, labels, and aspect ratio
    ax.set_xlim(-20,
                20)
    ax.set_ylim(-25,
                25)
    # Set x and y ticks
    ax.set_xticks(range(-20, 21,2))  # This sets ticks from -20 to 20 with increments of 1
    ax.set_yticks(range(-25, 26,2))  # This sets ticks from -25 to 25 with increments of 1

    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Circles from Multiple Files')
    ax.grid(True)
    
    plt.show()

# Provide the names of your txt files and their corresponding colors below
#files = ['single_scan_0.txt', 'single_scan_5.txt', 'single_scan_10.txt']

files = [ 'single_scan_139.txt', 'single_scan_140.txt', 'single_scan_141.txt']
colors = ['red', 'green', 'yellow']

#files = ['single_scan_127.txt', 'single_scan_128.txt']
#colors = ['red', 'green']

plot_files(files, colors)

#plot_files(files, colors)
