#!/usr/bin/env python

""" 
REFERENCES:
https://stackoverflow.com/questions/42512346/python-average-distance-between-a-bunch-of-points-in-the-x-y-plane
https://towardsdatascience.com/histograms-and-density-plots-in-python-f6bda88f5ac0

"""

import math
import numpy as np
from itertools import combinations
import matplotlib.pyplot as plt
import seaborn as sns
import scipy



def distance_x_y(p1, p2):
    (x1, y1), (x2, y2) = p1, p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def distance_x_y_z(p1, p2):
    (x1, y1, z1), (x2, y2, z2) = p1, p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def main():
    pointing_locations = [[-2.3228980690177763, -9.158608686018814, 1.0697022870690867],
                        [-2.155922755231462, -9.148659477671654, 1.0722689651610946],
                        [-2.0764846761287026, -9.122833199425695, 1.064680117667932],
                        [-2.3125476042519337, -9.125153569602453, 1.0347300487150841],
                        [-2.0663700714870035, -9.10469442604205, 1.0379960052693367],
                        [-2.2301920504647983, -9.12835994375536, 1.0408819125567874],
                        [-2.031052405262054, -9.06095447215161, 1.0431676426474836],
                        [-2.0156985381871713, -9.110844191941329, 1.0352854279225965],
                        [-2.157012439441496, -9.081716649215778, 1.0317787758315053],
                        [-2.162063709948788, -9.130983395109995, 1.0441414156100453],
                        [-2.1355684681662703, -9.078018473628228, 1.0691449782118088],
                        [-2.2331857429590944, -9.088984312391744, 1.0612285644039081],
                        [-2.0342894619742253, -9.09750990989405, 1.0723688350947478],
                        [-2.012752762324877, -9.100652119724398, 1.0381782103936037],
                        [-1.948920037633393, -9.096733765286471, 1.0257795178231612],
                        [-2.292827822225496, -9.128820688337271, 1.0393278991407888],
                        [-2.056995777081922, -9.060466430426814, 1.068145302250472],
                        [-2.0070143924264814, -9.147401182760552, 1.0374064549889404],
                        [-1.9771102738388882, -9.118050895944734, 1.0663259865702628],
                        [-2.066505336820594, -9.148391051468094, 1.0457923371828364],
                        [-1.97339265988, -9.06895183975, 1.03517302158],
                        [-2.29112640253, -9.10623737105, 1.07327439936],
                        [-2.10235936939, -9.0907713993, 1.04368418885],
                        [-2.09209282835, -9.07683017543, 1.07425308052],
                        [-2.14287114918, -9.0759884257, 1.0372928254],
                        [-2.14200516601, -9.07208156316, 1.03703911615],
                        [-2.12650094165, -9.07013841391, 1.03838341201],
                        [-2.14680623735, -9.0876746155, 1.03548108184],
                        [-2.10678319148, -9.15281085261, 1.04036324412],
                        [-2.11432112522, -9.14995791599, 1.03727034255],
                        [-2.14040461154, -9.09967010811, 1.03404819338],
                        [-2.19089850132, -9.12886960628, 1.0386819417]]

    x = [item[0] for item in pointing_locations]
    y = [item[1] for item in pointing_locations]
    z = [item[2] for item in pointing_locations]

    points_xy = list(zip(x,y))
    points_xyz = list(zip(x,y,z))
    distances_x_y = [distance_x_y(p1, p2) for p1, p2 in combinations(points_xy, 2)]
    distances_x_y_z = [distance_x_y_z(p1, p2) for p1, p2 in combinations(points_xyz, 2)]

    log_distances_x_y = np.log(distances_x_y)
    log_distances_x_y_z = np.log(distances_x_y_z)

    # print len(distances_x_y)
    # print len(distances_x_y_z)

    average_x_y = sum(distances_x_y) / len(distances_x_y)
    average_x_y_z = sum(distances_x_y_z) / len(distances_x_y_z)
    log_average_x_y = sum(log_distances_x_y) / len(log_distances_x_y)
    log_average_x_y_z = sum(log_distances_x_y_z) / len(log_distances_x_y_z)


    max_x_y = np.amax(np.array(distances_x_y))
    max_x_y_z = np.amax(np.array(distances_x_y_z))
    log_max_x_y = np.amax(np.array(log_distances_x_y))
    log_max_x_y_z = np.amax(np.array(log_distances_x_y_z))

    min_x_y = np.amin(np.array(distances_x_y))
    min_x_y_z = np.amin(np.array(distances_x_y_z))
    log_min_x_y = np.amin(np.array(log_distances_x_y))
    log_min_x_y_z = np.amin(np.array(log_distances_x_y_z))

    std_x_y = np.std(distances_x_y)
    std_x_y_z = np.std(distances_x_y_z)
    log_std_x_y = np.std(log_distances_x_y)
    log_std_x_y_z = np.std(log_distances_x_y_z)

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

    print("The logarithmically transformed average distance between all points in x-y is: " + str(log_average_x_y))
    print("The logarithmically transformed maximum distance between all points in x-y is: " + str(log_max_x_y))
    print("The logarithmically transformed minimum distance between all points in x-y is: " + str(log_min_x_y))
    print("The logarithmically transformed standard deviation in x-y is: " + str(log_std_x_y))

    print ("\n=========================================================================================\n")

    print("The logarithmically transformed average distance between all points in x-y-z is: " + str(log_average_x_y_z))
    print("The logarithmically transformed maximum distance between all points in x-y-z is: " + str(log_max_x_y_z))
    print("The logarithmically transformed minimum distance between all points in x-y-z is: " + str(log_min_x_y_z))
    print("The logarithmically transformed standard deviation in x-y-z is: " + str(log_std_x_y_z))
    upper = log_average_x_y_z + (2*log_std_x_y_z)
    lower = log_average_x_y_z - (2*log_std_x_y_z)
    print(upper)
    print(lower)

    print ("\n=========================================================================================\n")

        

    # Draw the plot

    _, bins, _  = plt.hist(distances_x_y, color = 'blue', edgecolor = 'black', bins = 25, normed=True)
    mu, sigma = scipy.stats.norm.fit(distances_x_y)
    best_fit_line = scipy.stats.norm.pdf(bins, mu, sigma)
    plt.plot(bins, best_fit_line, color='red', linewidth=3)
    
    # Title and labels
    plt.title('Histogram with distances in (x,y)')
    plt.xlabel('distances_x_y')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()


    # Draw the plot
    
    _, bins, _  = plt.hist(distances_x_y_z, color = 'blue', edgecolor = 'black', bins = 25, normed=True)
    mu, sigma = scipy.stats.norm.fit(distances_x_y_z)
    best_fit_line = scipy.stats.norm.pdf(bins, mu, sigma)
    plt.plot(bins, best_fit_line, color='red', linewidth=3)
    
    # Title and labels
    plt.title('Histogram with distances in (x,y,z)')
    plt.xlabel('distances_x_y_z')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()

    # Draw the plot
    
    _, bins, _  = plt.hist(log_distances_x_y, color = 'blue', edgecolor = 'black', bins = 25, normed=True)
    mu, sigma = scipy.stats.norm.fit(log_distances_x_y)
    best_fit_line = scipy.stats.norm.pdf(bins, mu, sigma)
    plt.plot(bins, best_fit_line, color='red', linewidth=3)
    
    # Title and labels
    plt.title('Histogram with logarithmic distances in (x,y)')
    plt.xlabel('log_distances_x_y_z')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()

    # Draw the plot
    
    _, bins, _  = plt.hist(log_distances_x_y_z, color = 'blue', edgecolor = 'black', bins = 25, normed=True)
    mu, sigma = scipy.stats.norm.fit(log_distances_x_y_z)
    best_fit_line = scipy.stats.norm.pdf(bins, mu, sigma)
    plt.plot(bins, best_fit_line, color='red', linewidth=3)
    
    # Title and labels
    plt.title('Histogram with logarithmic distances in (x,y,z)')
    plt.xlabel('log_distances_x_y_z')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()