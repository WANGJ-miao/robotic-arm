import numpy as np
from .basic_model import Jacobbin_1, Jacobbin_2, Jacobbin_3, Jacobbin_end, consider_f

# Parameters
L = np.array([0.40, 0.35, 0.20])
m = np.array([3.0, 2.5, 1.0])
m_load = 2.0
g = 9.81
# I = (1 / 12) * m * L**2

def static_torque(q, *, lengths=None, masses=None, load_mass=None, gravity=None):
    """计算重力补偿力矩；可选参数的单位依次为 m、kg、kg、m/s²。"""
    lengths = np.asarray(L if lengths is None else lengths, dtype=float)
    masses = np.asarray(m if masses is None else masses, dtype=float)
    load_mass = m_load if load_mass is None else float(load_mass)
    gravity = g if gravity is None else float(gravity)
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.all(np.isfinite(q)):
        raise ValueError("关节角必须为三个有限数值（弧度）")
    if lengths.shape != (3,) or not np.all(np.isfinite(lengths)) or np.any(lengths <= 0):
        raise ValueError("连杆长度必须为三个有限正数")
    if masses.shape != (3,) or not np.all(np.isfinite(masses)) or np.any(masses < 0):
        raise ValueError("连杆质量必须为三个有限非负数")
    if not np.isfinite(load_mass) or load_mass < 0 or not np.isfinite(gravity) or gravity < 0:
        raise ValueError("负载质量和重力加速度必须为有限非负数")
    W = masses * gravity
    W1 = np.array([0, W[0]])
    W2 = np.array([0, W[1]])
    W3 = np.array([0, W[2]])
    F = np.array([0, load_mass * gravity])
# q = np.array([0, 0, 0])
    M = Jacobbin_1(q, lengths).T @ W1 + Jacobbin_2(q, lengths).T @ W2 + Jacobbin_3(q, lengths).T @ W3 + Jacobbin_end(q, lengths).T @ F
    return M

def static_torque_with_f(q, q_dot, b, Fc, **parameters):
    M = static_torque(q, **parameters)
    return consider_f(M, q_dot, b, Fc)

def static_friction_margin(q, Fc, **parameters):
    """以未扣除摩擦的补偿力矩计算系统最小静摩擦余量（N·m）。"""
    Fc = np.broadcast_to(np.asarray(Fc, dtype=float), (3,))
    if not np.all(np.isfinite(Fc)) or np.any(Fc < 0):
        raise ValueError("最大静摩擦力矩必须为有限非负数")
    return float(np.min(Fc - np.abs(static_torque(q, **parameters))))

def is_self_lock(q, Fc, **parameters):
    return static_friction_margin(q, Fc, **parameters) >= 0
