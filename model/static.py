import math
import numpy as np
from .basic_model import Jacobbin_1, Jacobbin_2, Jacobbin_3, Jacobbin_end, consider_f

# Parameters
L = np.array([0.40, 0.35, 0.20])
m = np.array([3.0, 2.5, 1.0])
m_load = 2.0
g = 9.81
# I = (1 / 12) * m * L**2

def static_torque(q):
    W = m * g
    W1 = np.array([0, W[0]])
    W2 = np.array([0, W[1]])
    W3 = np.array([0, W[2]])
    F = np.array([0, m_load * g])
# q = np.array([0, 0, 0])
    M = Jacobbin_1(q, L).T @ W1 + Jacobbin_2(q, L).T @ W2 + Jacobbin_3(q, L).T @ W3 + Jacobbin_end(q, L).T @ F
    return np.round(M, 4)

def static_torque_with_f(q, q_dot, b, Fc):
    M = static_torque(q)
    return consider_f(M, q_dot, b, Fc)

def is_self_lock(q, Fc):
    q_dot = np.zeros(3)
    M = static_torque_with_f(q, q_dot, b=0, Fc=Fc)
    return np.all(np.isclose(M, 0))
