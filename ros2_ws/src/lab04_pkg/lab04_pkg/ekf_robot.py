import math
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from math import degrees, sin, cos
import sympy

arrow = u'$\u2191$'

from lab04_pkg.utils import residual
from lab04_pkg.plot_utils import plot_covariance
from lab04_pkg.ekf import RobotEKF
from lab04_pkg.probabilistic_models import (
    evaluate_sampling_dist,
    sample_velocity_motion_model,
    sample_odometry_motion_model,
    get_odometry_command,
    landmark_range_bearing_model,
    landmark_range_bearing_sensor,
)
from lab04_pkg.probabilistic_models import velocity_mm_simpy, odometry_mm_simpy, landmark_sm_simpy



def main():

    #TASK 0
    ####Probabilistic velocity-based motion model (sampling):####
    dt = 0.5
    n_samples = 500
    x = np.array([2, 4, 0])  # initial robot pose
    u = np.array([0.8, 0.6])  # [v, w]
    a = np.array([0.001, 0.01, 0.1, 0.2, 0.05, 0.05])  # noise parameters for velocity motion model
    x_prime = np.zeros((n_samples, 3))

    for i in range(n_samples):
        x_prime[i, :] = sample_velocity_motion_model(x, u, a, dt)

    #media -> mu[0] : media posizione x' finali, mu[1] : media posizione y' finali, mu[2] : media angoli theta' finali 
    mu = np.mean(x_prime, axis=0)
    #deviazione standard
    sigma = np.std(x_prime, axis=0)
    evaluate_sampling_dist(mu[0], sigma[0], n_samples, np.random.normal)

    #plotting samples
    
    #plot della posizione iniziale x[0] e x[1], con una freccia come marker 
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    #da la giusta direzione alla freccia
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x[2])-90)
    plt.scatter(x[0], x[1], marker=rotated_marker, s=100, facecolors='none', edgecolors='b')
    #generazione della nuvola, itera solamente i primi 200 punti
    for x_ in x_prime[:200]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling")
    plt.savefig("velocity_samples.pdf")
    plt.show()

    #Gt jacobian and Vt jacobian with sympy
    Gt, Vt = velocity_mm_simpy()[1:3]

    ####Probabilistic measurement model - landmark model (sampling):####
    n_samples2 = 1000
    

    #initialize EKF for robot with velocity motion model
    """Initialize the EKF for a robot with velocity motion model
        ekf_robot = RobotEKF(
        dim_x = 3,
        dim_u = 2,
        eval_gux = sample_velocity_motion_model,
        eval_Gt = velocity_mm_simpy()[1],
        eval_Vt = velocity_mm_simpy()[2],
    )
    """



if __name__ == "__main__":
    main()