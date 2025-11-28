
import yaml
import numpy as np
import probabilistic_models as pm
from sympy import symbols, Matrix
import sympy
from math import sin, sqrt


def squeeze_sympy_out(func):
    # inner function
    def squeeze_out(*args):
        out = func(*args).squeeze()
        return out

    return squeeze_out

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
id_list = [11, 12, 13, 21, 22, 23, 31, 32, 33]


#lettura landmark nel file yaml
filename = "/home/luke_skywalker/ros2_ws/src/turtlebot3_perception/turtlebot3_perception/config/landmarks.yaml"

with open(filename, 'r') as file:
    data = yaml.safe_load(file)

landmarks_matrix = np.column_stack((  data['landmarks']['x'],data['landmarks']['y']))
print(landmarks_matrix)
landmarks_coordinate = {}
for id, landmark in zip(id_list, landmarks_matrix):
    landmarks_coordinate[id] = landmark

print(landmarks_coordinate)



