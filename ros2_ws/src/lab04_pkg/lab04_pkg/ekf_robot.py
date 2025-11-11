import math
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from math import degrees, sin, cos
import sympy

from lab04_pkg.utils import landmark_model_sample_pose

arrow = u'$\u2191$'



from lab04_pkg.ekf import RobotEKF
from lab04_pkg.probabilistic_models import (
    evaluate_sampling_dist,
    landmark_sm_simpy,
    sample_velocity_motion_model,
    velocity_mm_simpy

)



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
    _, eval_Gt, eval_Vt = velocity_mm_simpy()

    Gt_numerical =eval_Gt(*x, *u, dt)
    print("G_t:\n", sympy.latex(sympy.Matrix(Gt_numerical)))
    Vt_numerical = eval_Vt(*x, *u, dt)
    print("V_t:\n", sympy.latex(sympy.Matrix(Vt_numerical)))

    ####Probabilistic measurement model - landmark model (sampling):####
    n_samples2 = 1000
    robot_pose = np.array([0.0, 0.0, math.pi/4])  # robot pose
    z = np.array([5.0, math.pi/6])  # measurement [range, bearing]
    landmark_pos = np.array([5.0, 2.0])  # landmark
    sigma = np.array([0.3, math.pi/24])  # noise standard deviations [range, bearing]

    xland_prime = np.zeros((n_samples2, 3))
    for i in range(n_samples2):
        xland_prime[i, :] = landmark_model_sample_pose(z, landmark_pos, sigma)
         # plot robot pose
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(xland_prime[i, 2])-90)
        plt.scatter(xland_prime[i, 0], xland_prime[i, 1], marker=rotated_marker, s=80, facecolors='none', edgecolors='b')

    # plot real pose
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(robot_pose[2])-90)
    plt.scatter(robot_pose[0], robot_pose[1], marker=rotated_marker, s=140, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("Landmark Model Pose Sampling")
    plt.show()

    #Ht jacobian with sympy
    _, Ht = landmark_sm_simpy()
    Ht_numerical = Ht(*robot_pose, *landmark_pos)
    print("H_t:\n", sympy.latex(sympy.Matrix(Ht_numerical)))






if __name__ == "__main__":
    main()