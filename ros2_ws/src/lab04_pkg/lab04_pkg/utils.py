from matplotlib import pyplot as plt
import numpy as np
from numpy import linalg as la
import math
from math import cos, sin, sqrt, atan2
import numpy as np
import sympy
from sympy import symbols, Matrix
from scipy.stats import norm




def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = theta % (2 * np.pi)  # force in range [0, 2 pi)
    if theta > np.pi:  # move to [-pi, pi)
        theta -= 2 * np.pi
    
    return theta

# Gaussian Function
def gaussian(x, mu, sigma):
    return (1.0 / (np.sqrt(2*np.pi) * sigma)) * np.exp(-0.5 * ((x - mu) / sigma)**2)
    
def compute_p_hit_dist(dist, max_dist, sigma):
    '''
    Compute the hit probability p_hit for a given distance measurement.
    Args:
        dist: observed distance measurement
        max_dist: maximum measurable distance
        sigma: standard deviation of the Gaussian noise
    Returns:
        p_hit: normalized hit probability
    '''
    # Normalize the Gaussian over [0, max_dist]
    normalize_hit = 1e-9
    for j in range(round(max_dist)):
        normalize_hit += gaussian(j, 0., sigma)
    normalize_hit = 1. / normalize_hit

    p_hit = gaussian(dist, 0., sigma)*normalize_hit

    return p_hit

# Plot the distribution of z samples
def plot_sampling_dist(samples, title="Distribution of z samples", fig_name="z_star_hist.pdf"):
    '''
    Plot the distribution of z samples.
    Args:
        samples: array of z samples
        title: title of the plot
        fig_name: name of the file to save the plot
    '''
    
    n_bins = 100
    plt.hist(samples, n_bins)
    plt.title(title)
    plt.grid()
    plt.savefig(fig_name)
    plt.show()
    plt.close('all')

def landmark_model_sample_pose(z, landmark, sigma):
    """""
    Sample a robot pose from the landmark model
    Inputs:
        - z: the measurements features (range and bearing of the landmark from the sensor) [r, phi]
        - landmark: the landmark position in the map [m_x, m_y]
        - sigma: the standard deviation of the measurement noise [sigma_r, sigma_phi]
    Outputs:
        - x': the sampled robot pose [x', y', theta']
    """""
    m_x, m_y = landmark[:]
    sigma_r, sigma_phi = sigma[:]

    gamma_hat = np.random.uniform(0, 2*math.pi)
    r_hat = z[0] + np.random.normal(0, sigma_r)
    phi_hat = z[1] + np.random.normal(0, sigma_phi)

    x_ = m_x + r_hat * math.cos(gamma_hat)
    y_ = m_y + r_hat * math.sin(gamma_hat)
    theta_ = gamma_hat - math.pi - phi_hat

    return np.array([x_, y_, theta_])

def get_odometry_command(odom_pose, odom_pose_prev):
    """Transform robot poses taken from odometry to u command
    Arguments:
    odom_pose -- last odometry pose of the robot [x, y, theta] at time t
    odom_pose_prev -- previous odometry pose of the robot [x, y, theta] at time t-1

    Output:
    u_odom : np.array [rot1, trasl, rot2]
    """

    x_odom, y_odom, theta_odom = odom_pose[:]
    x_odom_prev, y_odom_prev, theta_odom_prev = odom_pose_prev[:]

    rot1 = math.atan2(y_odom - y_odom_prev, x_odom - x_odom_prev) - theta_odom_prev
    trasl = sqrt((x_odom - x_odom_prev) ** 2 + (y_odom - y_odom_prev) ** 2)
    rot2 = theta_odom - theta_odom_prev - rot1

    return np.array([rot1, trasl, rot2])




def landmark_range_bearing_model(robot_pose, landmark, sigma):
    """""
    Sampling z from landmark model for range and bearing
    """ ""
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]

    r_ = math.dist([x, y], [m_x, m_y]) + np.random.normal(0.0, sigma[0])
    phi_ = math.atan2(m_y - y, m_x - x) - theta + np.random.normal(0.0, sigma[1])
    return np.array([r_, phi_])


def landmark_range_bearing_sensor(robot_pose, landmark, sigma, max_range=6.0, fov=math.pi / 2):
    """""
    Simulate the detection of a landmark with a virtual sensor able to estimate range and bearing
    """ ""
    z = landmark_range_bearing_model(robot_pose, landmark, sigma)

    # filter z for a more realistic sensor model (add a max range distance and a FOV)
    if z[0] > max_range or abs(z[1]) > fov / 2:
        return None

    return z


def velocity_mm_Gt(x, u, dt):
    """
    Evaluate Jacobian Gt w.r.t state x=[x, y, theta]
    """
    theta = x[2]
    v, w = u[0], u[1]
    r = v / w
    Gt = np.array(
        [
            [1, 0, -r * cos(theta) + r * cos(theta + w * dt)],
            [0, 1, -r * sin(theta) + r * sin(theta + w * dt)],
            [0, 0, 1],
        ]
    )

    return Gt


def velocity_mm_Vt(x, u, dt):
    """
    Evaluate Jacobian Vt w.r.t command u=[v,w]
    """
    theta = x[2]
    v, w = u[0], u[1]
    r = v / w
    Vt = np.array(
        [
            [
                -sin(theta) / w + sin(theta + w * dt) / w,
                dt * v * cos(theta + w * dt) / w + v * sin(theta) / w**2 - v * sin(theta + w * dt) / w**2,
            ],
            [
                -cos(theta) / w - cos(theta + w * dt) / w,
                dt * v * sin(theta + w * dt) / w - v * cos(theta) / w**2 + v * cos(theta + w * dt) / w**2,
            ],
            [0, dt],
        ]
    )

    return Vt


def landmark_sm_Ht(x, mx, my):
    x, y = x[0], x[1]
    div = sqrt((mx - x) ** 2 + (my - y) ** 2)
    Ht = np.array([[(-mx + x) / div, (-my + y) / div, 0], [-(-my + y) / div, -(mx - x) / div, -1]])

    return Ht


# decorator
def squeeze_sympy_out(func):
    # inner function
    def squeeze_out(*args):
        out = func(*args).squeeze()
        return out

    return squeeze_out

#it calculates automatically the Jacobians using sympy
def velocity_mm_simpy():
    """
    Define Jacobian Gt w.r.t state x=[x, y, theta] and Vt w.r.t command u=[v, w]
    """
    x, y, theta, v, w, dt = symbols("x y theta v w dt")
    R = v / w
    beta = theta + w * dt
    gux = Matrix(
        [
            [x - R * sympy.sin(theta) + R * sympy.sin(beta)],
            [y + R * sympy.cos(theta) - R * sympy.cos(beta)],
            [beta],
        ]
    )

    eval_gux = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), gux, "numpy"))
    #compute the Jacobian (partial derivatives) w.r.t. the state variables
    Gt = gux.jacobian(Matrix([x, y, theta]))
    eval_Gt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Gt, "numpy"))
    print("Gt:", Gt)

    #compute the Jacobian (partial derivatives) w.r.t. the command variables
    Vt = gux.jacobian(Matrix([v, w]))
    eval_Vt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Vt, "numpy"))
    print("Vt:", Vt)

    return eval_gux, eval_Gt, eval_Vt


def odometry_mm_simpy():
    """
    Define Jacobian Gt and Vt for the odometry motion model
    """
    rot1, trasl, rot2 = symbols(r"\delta_{rot1} \delta_{trasl} \delta_{rot2}")
    x, y, theta = symbols(r"x y \theta")
    gux_odom = Matrix(
        [
            [x + trasl * sympy.cos(theta + rot1)],
            [y + trasl * sympy.sin(theta + rot1)],
            [theta + rot1 + rot2],
        ]
    )
    Gt_odom = gux_odom.jacobian(Matrix([x, y, theta]))
    Vt_odom = gux_odom.jacobian(Matrix([rot1, trasl, rot2]))

    args = (x, y, theta, rot1, trasl, rot2)
    eval_gux_odom = squeeze_sympy_out(sympy.lambdify(args, gux_odom, "numpy"))
    eval_Gt_odom = squeeze_sympy_out(sympy.lambdify(args, Gt_odom, "numpy"))
    eval_Vt_odom = squeeze_sympy_out(sympy.lambdify(args, Vt_odom, "numpy"))

    return eval_gux_odom, eval_Gt_odom, eval_Vt_odom


def landmark_sm_simpy():
    x, y, theta, mx, my = symbols("x y theta m_x m_y")

    hx = Matrix(
        [
            [sympy.sqrt((mx - x) ** 2 + (my - y) ** 2)],
            [sympy.atan2(my - y, mx - x) - theta],
        ]
    )
    eval_hx = squeeze_sympy_out(sympy.lambdify((x, y, theta, mx, my), hx, "numpy"))

    Ht = hx.jacobian(Matrix([x, y, theta]))
    eval_Ht = squeeze_sympy_out(sympy.lambdify((x, y, theta, mx, my), Ht, "numpy"))
    # print("Ht:", Ht)

    return eval_hx, eval_Ht


def landmark_slam_jacobian_sympy():
    x, y, theta, mx, my = symbols("x y theta m_x m_y")

    hx = Matrix(
        [
            [sympy.sqrt((mx - x) ** 2 + (my - y) ** 2)],
            [sympy.atan2(my - y, mx - x) - theta],
        ]
    )

    Ht = hx.jacobian(Matrix([x, y, theta, mx, my]))

    return sympy.lambdify(((x, y, theta), mx, my), Ht, "numpy")

def evaluate_sampling_dist(mu, sigma, n_samples, sample_function):
#prende in ingresso mu e sigma (LA STATISTICA CALCOLATA)
#sample_function è il metodo per campionare -> np.random.normal
    n_bins = 100
    samples = []

#generazione di n_samples NUOVI campioni casuali
    for i in range(n_samples):
        samples.append(sample_function(mu, sigma))

#"verifica" che la media e la distribuzione abbiano valori simili
    print("%30s : mean = %.3f, std_dev = %.3f" % ("Normal", np.mean(samples), np.std(samples)))

#creazione dell'istogramma (mostra quante volte un valore è apparso)
    count, bins, ignored = plt.hist(samples, n_bins)
    #sovrapposizione con la curva a campana (di colore rosso) QUELLA TEORICA, con mu e sigma fornite
    plt.plot(bins, norm(mu, sigma).pdf(bins), linewidth=2, color='r')
    plt.xlim([mu - 5*sigma, mu + 5*sigma])
    plt.title("Normal distribution of samples")
    plt.grid()
    plt.savefig("gaussian_dist.pdf")
    plt.show()