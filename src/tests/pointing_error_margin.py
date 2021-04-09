#!/usr/bin/env python

""" 
REFERENCES:
https://stackoverflow.com/questions/42512346/python-average-distance-between-a-bunch-of-points-in-the-x-y-plane

"""

import math
import numpy as np
from itertools import combinations
import matplotlib.pyplot as plt
import seaborn as sns



def distance_x_y(p1, p2):
    (x1, y1), (x2, y2) = p1, p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def distance_x_y_z(p1, p2):
    (x1, y1, z1), (x2, y2, z2) = p1, p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def main():
    x = [-2.32290,-2.15592,-2.07648,-2.31255,-2.06637,-2.23019,-2.03105,-2.01570,-2.15701,-2.16206]
    y = [-9.15861,-9.14866,-9.12283,-9.12515,-9.10469,-9.12836,-9.06095,-9.11084,-9.08172,-9.13098]
    z = [1.06970,1.07227,1.06468,1.03473,1.03800,1.04088,1.04317,1.03529,1.03178,1.04414]

    points_xy = list(zip(x,y))
    points_xyz = list(zip(x,y,z))
    distances_x_y = [distance_x_y(p1, p2) for p1, p2 in combinations(points_xy, 2)]
    distances_x_y_z = [distance_x_y_z(p1, p2) for p1, p2 in combinations(points_xyz, 2)]

    # print len(distances_x_y)
    # print len(distances_x_y_z)

    average_x_y = sum(distances_x_y) / len(distances_x_y)
    average_x_y_z = sum(distances_x_y_z) / len(distances_x_y_z)

    max_x_y = np.amax(np.array(distances_x_y))
    max_x_y_z = np.amax(np.array(distances_x_y_z))

    min_x_y = np.amin(np.array(distances_x_y))
    min_x_y_z = np.amin(np.array(distances_x_y_z))

    std_x_y = np.std(distances_x_y)
    std_x_y_z = np.std(distances_x_y_z)

    print ("\n=========================================================================================\n")

    print("The average distance between all points in x-y is: " + str(average_x_y))
    print("The maximum distance between all points in x-y is: " + str(max_x_y))
    print("The minimum distance between all points in x-y is: " + str(min_x_y))
    print("The standard deviation in x-y is: " + str(std_x_y))

    print ("\n=========================================================================================\n")

    print("The average distance between all points in x-y-z is: " + str(average_x_y_z))
    print("The maximum distance between all points in x-y-z is: " + str(max_x_y_z))
    print("The minimum distance between all points in x-y-z is: " + str(min_x_y_z))
    print("The standard deviation in x-y-z is: " + str(std_x_y_z))

    print ("\n=========================================================================================\n")


        

    # Draw the plot
    plt.hist(distances_x_y, color = 'blue', edgecolor = 'black')
    
    # Title and labels
    plt.title('Histogram with distances in (x,y)')
    plt.xlabel('distances_x_y')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()


    # Draw the plot
    plt.hist(distances_x_y_z, color = 'blue', edgecolor = 'black')
    
    # Title and labels
    plt.title('Histogram with distances in (x,y,z)')
    plt.xlabel('distances_x_y_z')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()