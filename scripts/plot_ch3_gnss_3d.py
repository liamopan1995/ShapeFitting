import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input a valid file path as a command-line argument.')
        sys.exit(1)
    else:
        path = sys.argv[1]
        try:
            path_data = np.loadtxt(path)
        except Exception as e:
            print('Error loading data:', e)
            sys.exit(1)
        
        # Create a new figure and 3D axis
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the path using x, y, and z coordinates
        ax.scatter(path_data[0,1],path_data[0,2],path_data[0,3])
        ax.plot(path_data[:, 1], path_data[:, 2], path_data[:, 3])
        
        # Set labels for the axes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.title('3D Path')
        
        # Display the plot
        plt.show()

