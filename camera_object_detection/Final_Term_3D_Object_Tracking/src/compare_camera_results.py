# libraries
import matplotlib.axes as axes
import matplotlib.pyplot as plt

# Data
x  = [1, 2, 3, 4, 5, 6 ,7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]

# SHITOMASI + BRIEF
y1_lidar  = [13.82, 11.39, 18.59, 12.29, 13.73, 6.95,  50.45, 21.52, 12.45, 14.84, 10.35, 10.11, 9.23,  11.15, 7.92,  9.05,  9.58,  9.22]
y1_camera = [15.39, 40.09, 15.42, 13.63, 13.85, 13.52, 16.82, 13.25, 13.05, 18.26, 11.26, 13.39, 11.69, 12.69, 13.79, 11.48, 11.25, 11.44]

# HARRIS + BRIEF
y2_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y2_camera = [12.3518, 11.5838, -5000, 12.9793, 26.3393, 50.2722, 13.5316, 16.0341, 11.4731, 15.4418, 12.9125, 13.3472, 11.1824, 12.3028, 8.15086, 10.347, 10.4321, 11.9483]

# FAST + BRIEF
y3_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y3_camera = [13.3604, 12.84, 33.6995, 17.8482, 173.98, 12.8903, 32.0554, 25.3921, 13.6347, 13.4789, 13.7952, 11.0145, 12.273, 12.3292, 12.0672, 12.2613, 9.80845, 11.6857]

# -5000 means -inf
y2 = [1.5, 2.3, 3.1, 4.2, 5.6, 6.1 ,7.5, 8.2, 9, 10.8]
y3 = [-0.1, 3, -12.2, 3, 4, 6, 3.3, 2.2, 9, 10.2]

# plotting the line 1 points 
plt.plot(x, y1_lidar, '-x', label = "SHITOMASI + BRIEF")
# plotting the line 2 points
# plt.plot(x, y1_camera, '-o', label = "SHITOMASI + BRIEF")

# plotting the line 1 points 
plt.plot(x, y2_lidar, '-x', label = "HARRIS + BRIEF")
# plotting the line 2 points
# plt.plot(x, y2_camera, '-o', label = "HARRIS + BRIEF")

# plotting the line 1 points 
plt.plot(x, y2_lidar, '-x', label = "FAST + BRIEF")
# plotting the line 2 points
# plt.plot(x, y3_camera, '-o', label = "FAST + BRIEF")


plt.xlabel('x - axis')
# Set the y axis label of the current axis.
plt.ylabel('y - axis')
# Set a title of the current axes.
plt.title('Camera-based TTC via Various Detector/Descriptor Combinations')
# show a legend on the plot
plt.legend()
# set ylim
plt.ylim(-100, 200)
# Display a figure.
plt.show()