# libraries
import matplotlib.axes as axes
import matplotlib.pyplot as plt
import numpy as np

# Data
inf = np.nan # use nan to represent inf/-inf
x  = [1, 2, 3, 4, 5, 6 ,7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]

# SHITOMASI + BRIEF
y1_lidar  = [13.82, 11.39, 18.59, 12.29, 13.73, 6.95,  50.45, 21.52, 12.45, 14.84, 10.35, 10.11, 9.23,  11.15, 7.92,  9.05,  9.58,  9.22]
y1_camera = [15.39, 40.09, 15.42, 13.63, 13.85, 13.52, 16.82, 13.25, 13.05, 18.26, 11.26, 13.39, 11.69, 12.69, 13.79, 11.48, 11.25, 11.44]

# HARRIS + BRIEF
y2_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y2_camera = [12.3518, 11.5838, np.nan, 12.9793, 26.3393, 50.2722, 13.5316, 16.0341, 11.4731, 15.4418, 12.9125, 13.3472, 11.1824, 12.3028, 8.15086, 10.347, 10.4321, 11.9483]

# FAST + BRIEF
y3_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y3_camera = [13.3604, 12.84, 33.6995, 17.8482, 173.98, 12.8903, 32.0554, 25.3921, 13.6347, 13.4789, 13.7952, 11.0145, 12.273, 12.3292, 12.0672, 12.2613, 9.80845, 11.6857]

# FAST + BRISK
y4_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y4_camera = [27.3664, -114.855, 39.6158, 145.807, -20.4517, np.nan, -0.321148, 13.3076, -0.705423, np.nan, np.nan, 13.5788, 15.5997, 58.9522, 112.594, 16.5855, 11.921, 86.3783]

# HARRIS + BRISK
y5_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y5_camera = [-inf, 14.3958, -inf, 17.8877, -2.65703, -inf, -1.60711, -57.8825, 13.3666, 37.4449, 14.3129, 11.9445, 12.2202, 61.0256, 13.0645, 15.969, 11.4, 14.9272]

# HARRIS + ORB
y6_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y6_camera = [12.3736, 13.0249, 43.416, 13.8558, -inf, 26.0003, 13.8686, 16.5174, 11.9434, 12.682, 13.8455, 12.1102, 11.6118, 13.0669, 8.82507, 13.312, 10.8452, 12.5741]

# BRISK + ORB
y7_lidar  = [13.8965, 11.3797, 18.5951, 12.2969, 13.7339, 6.95595, 50.4471, 21.5201, 12.4533, 14.8439, 10.3535, 10.1097, 9.2321, 11.1515, 7.91908, 9.05329, 9.58194, 9.22293]
y7_camera = [-226.854, 104.239, 34.3356, 194.109, -26.6819, 100.836, 50.6976, 26.2895, 18.9378, 24.9965, 24.694, 20.813, 16.0697, 18.28, 29.2748, 15.9734, 14.7284, 19.5188]

# plotting the line 1 points 
# plt.plot(x, y1_lidar, '-x', label = "SHITOMASI + BRIEF")
# plotting the line 1 points
plt.plot(x, y1_camera, '-o', label = "SHITOMASI + BRIEF (0% inf)")

# plotting the line 2 points 
# plt.plot(x, y2_lidar, '-x', label = "HARRIS + BRIEF")
# plotting the line 2 points
plt.plot(x, y2_camera, '-o', label = "HARRIS + BRIEF (5.6% inf)")

# plotting the line 3 points 
# plt.plot(x, y3_lidar, '-x', label = "FAST + BRIEF")
# plotting the line 3 points
plt.plot(x, y3_camera, '-o', label = "FAST + BRIEF (0% inf)")

# plotting the line 4 points 
# plt.plot(x, y4_lidar, '-x', label = "FAST + BRISK")
# plotting the line 4 points
plt.plot(x, y4_camera, '-o', label = "FAST + BRISK (16.7% inf)")

# plotting the line 5 points 
# plt.plot(x, y5_lidar, '-x', label = "HARRIS + BRISK")
# plotting the line 5 points
plt.plot(x, y5_camera, '-o', label = "HARRIS + BRISK (16.7% inf)")

# plotting the line 6 points 
# plt.plot(x, y6_lidar, '-x', label = "HARRIS + ORB")
# plotting the line 6 points
plt.plot(x, y6_camera, '-o', label = "HARRIS + ORB (5.6% inf)")

# plotting the line 7 points 
# plt.plot(x, y7_lidar, '-x', label = "BRISK + ORB")
# plotting the line 7 points
plt.plot(x, y7_camera, '-o', label = "BRISK + ORB (0% inf)")

plt.xlabel('X Axis - Image Series')
# Set the y axis label of the current axis.
plt.ylabel('Y Axis - TTC [sec]')
# Set a title of the current axes.
# plt.title('Lidar-based TTC via Various Detector/Descriptor Combinations')
plt.title('Camera-based TTC via Various Detector/Descriptor Combinations')
# show a legend on the plot
plt.legend()
# set ylim
# plt.ylim(0, 65)
plt.ylim(-250, 210)
# Display a figure.
plt.show()