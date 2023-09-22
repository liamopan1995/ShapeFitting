import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def plot_txt_file(filename):  
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
                circles.append(Circle((x, y), r, fill=False))

    # Create a figure and axis for plotting
    fig, ax = plt.subplots()

    # Add circles to the plot
    for circle in circles:
        ax.add_artist(circle)

    # Set limits and labels
    ax.set_xlim(-20,
                20)
    ax.set_ylim(-25,
                25)
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Circles from ' + filename)
    ax.grid(True)
    
    plt.show(block=False)

# Provide the names of your txt files below
plot_txt_file('single_scan_0.txt')
plot_txt_file('single_scan_10.txt')
plot_txt_file('single_scan_25.txt')
plot_txt_file('single_scan_50.txt')

plt.show()

