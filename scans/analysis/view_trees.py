# Read the data from the actual file
file_path = 'translation_consective_poses.txt'
data = pd.read_csv(file_path, sep=" ", header=None)

# Extract the relevant columns for plotting
x = data[0]
y = data[1]
radius = data[2]
color_values = data[5]

# Create a color map
unique_colors = np.unique(color_values)
colors = plt.cm.get_cmap('viridis', len(unique_colors))
color_map = dict(zip(unique_colors, colors(range(len(unique_colors)))))

# Create the plot
plt.figure(figsize=(10, 6))
for i in range(len(data)):
    circle = plt.Circle((x[i], y[i]), radius[i], color=color_map[color_values[i]], alpha=0.5)
    plt.gca().add_patch(circle)

plt.xlim(min(x) - 1, max(x) + 1)
plt.ylim(min(y) - 1, max(y) + 1)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Plot of Circles with Colored Categories')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

