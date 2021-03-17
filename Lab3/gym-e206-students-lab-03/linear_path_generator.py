# Linear Path Generator

# Replacement for Dubins path module that connects points via a straight 
# line, disregarding robot heading

# Nathan Sunbury

import math
import numpy as np
import matplotlib.pyplot as plt

def construct_linear_path(point_0, point_1, step_size):

    step_number = int(math.sqrt(((point_1[0] - point_0[0])**2) + ((point_1[1] - point_0[1])**2))/step_size)
    
    x_points = np.linspace(point_0[0],point_1[0], step_number)
    x_points = np.ndarray.tolist(x_points)

    y_points = np.linspace(point_0[1],point_1[1], step_number)
    y_points = np.ndarray.tolist(y_points)

    theta_points = np.linspace(point_0[2],point_1[2], step_number)
    theta_points = np.ndarray.tolist(theta_points)

    configurations = []
    for i in range(len(x_points)):
        configurations.append([x_points[i],y_points[i],theta_points[i]])

    return configurations

if __name__ == '__main__':
    tp0 = [-3,4,0]
    tp1 = [10,3,0]
    step_size = .5
    path = construct_linear_path(tp0, tp1, step_size)
    # print("a ", path)
    # path = [np.ndarray.tolist(path[0]), np.ndarray.tolist(path[1]), np.ndarray.tolist(path[2])]
    # print("b ", path)
    # fig = plt.plot(path[0], path[1])
    # plt.show()
    
