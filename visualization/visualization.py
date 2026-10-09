import numpy as np
import matplotlib.pyplot as plt

from model import static

def draw_effort_with_joint23_fixed(theta2: float=0, theta3: float=0):
    """ 
    固定关节23, 绘制随着关节1变化, 三个关节的力矩大小
    单位rad
    """
    theta1_array = np.linspace(-np.pi, np.pi, 300)

    tau_raw_history = []
    tau_with_fric_history = []

    for theta1 in theta1_array:
        q = np.array([theta1, theta2, theta3])
        tau_raw = static.static_torque(q)
        q_dot = np.zeros(3)
        tau_with_fric = static.static_torque_with_f(q, q_dot, b=0, Fc=2)
        tau_raw_history.append(tau_raw)
        tau_with_fric_history.append(tau_with_fric)

    tau_raw_history = np.array(tau_raw_history)
    tau_with_fric_history  = np.array(tau_with_fric_history)

    fig, ax = plt.subplots(1, 2, squeeze=False)
    ax[0, 0].plot(np.rad2deg(theta1_array), tau_raw_history[:, 0], label="without friction, motor 0")
    ax[0, 0].plot(np.rad2deg(theta1_array), tau_raw_history[:, 1], label="without friction, motor 1")
    ax[0, 0].plot(np.rad2deg(theta1_array), tau_raw_history[:, 2], label="without friction, motor 2")
    ax[0, 1].plot(np.rad2deg(theta1_array), tau_with_fric_history[:, 0], label="with friction, motor 0")
    ax[0, 1].plot(np.rad2deg(theta1_array), tau_with_fric_history[:, 1], label="with friction, motor 1")
    ax[0, 1].plot(np.rad2deg(theta1_array), tau_with_fric_history[:, 2], label="with friction, motor 2")
    for i in range(2):
        ax[0, i].set_xlabel(r"$\theta_1$ (deg)")
        ax[0, i].set_ylabel(r"Torque (N*m)")
        ax[0, i].legend()
        ax[0, i].grid()
    plt.show()

def draw_effort_with_joint13_fixed(theta1: float=0, theta3: float=0):
    """ 
    固定关节13, 绘制随着关节2变化, 三个关节的力矩大小
    单位rad
    """
    theta2_array = np.linspace(-np.pi, np.pi, 300)

    tau_raw_history = []
    tau_with_fric_history = []

    for theta2 in theta2_array:
        q = np.array([theta1, theta2, theta3])
        tau_raw = static.static_torque(q)
        q_dot = np.zeros(3)
        tau_with_fric = static.static_torque_with_f(q, q_dot, b=0, Fc=2)
        tau_raw_history.append(tau_raw)
        tau_with_fric_history.append(tau_with_fric)

    tau_raw_history = np.array(tau_raw_history)
    tau_with_fric_history  = np.array(tau_with_fric_history)

    fig, ax = plt.subplots(1, 2, squeeze=False)
    ax[0, 0].plot(np.rad2deg(theta2_array), tau_raw_history[:, 0], label="without friction, motor 0")
    ax[0, 0].plot(np.rad2deg(theta2_array), tau_raw_history[:, 1], label="without friction, motor 1")
    ax[0, 0].plot(np.rad2deg(theta2_array), tau_raw_history[:, 2], label="without friction, motor 2")
    ax[0, 1].plot(np.rad2deg(theta2_array), tau_with_fric_history[:, 0], label="with friction, motor 0")
    ax[0, 1].plot(np.rad2deg(theta2_array), tau_with_fric_history[:, 1], label="with friction, motor 1")
    ax[0, 1].plot(np.rad2deg(theta2_array), tau_with_fric_history[:, 2], label="with friction, motor 2")
    for i in range(2):
        ax[0, i].set_xlabel(r"$\theta_2$ (deg)")
        ax[0, i].set_ylabel(r"Torque (N*m)") # 这里可以这样插入变量吗
        ax[0, i].legend()
        ax[0, i].grid()
    plt.show()

def draw_effort_with_joint12_fixed(theta1: float=0, theta2: float=0):
    """ 
    固定关节12, 绘制随着关节3变化, 三个关节的力矩大小
    单位rad
    """
    theta3_array = np.linspace(-np.pi, np.pi, 300)

    tau_raw_history = []
    tau_with_fric_history = []

    for theta3 in theta3_array:
        q = np.array([theta1, theta2, theta3])
        tau_raw = static.static_torque(q)
        q_dot = np.zeros(3)
        tau_with_fric = static.static_torque_with_f(q, q_dot, b=0, Fc=2)
        tau_raw_history.append(tau_raw)
        tau_with_fric_history.append(tau_with_fric)

    tau_raw_history = np.array(tau_raw_history)
    tau_with_fric_history  = np.array(tau_with_fric_history)

    fig, ax = plt.subplots(1, 2, squeeze=False)
    ax[0, 0].plot(np.rad2deg(theta3_array), tau_raw_history[:, 0], label="without friction, motor 0")
    ax[0, 0].plot(np.rad2deg(theta3_array), tau_raw_history[:, 1], label="without friction, motor 1")
    ax[0, 0].plot(np.rad2deg(theta3_array), tau_raw_history[:, 2], label="without friction, motor 2")
    ax[0, 1].plot(np.rad2deg(theta3_array), tau_with_fric_history[:, 0], label="with friction, motor 0")
    ax[0, 1].plot(np.rad2deg(theta3_array), tau_with_fric_history[:, 1], label="with friction, motor 1")
    ax[0, 1].plot(np.rad2deg(theta3_array), tau_with_fric_history[:, 2], label="with friction, motor 2")
    for i in range(2):
        ax[0, i].set_xlabel(r"$\theta_3$ (deg)")
        ax[0, i].set_ylabel(r"Torque (N*m)")
        ax[0, i].legend()
        ax[0, i].grid()
    plt.show()

def draw_self_lock_position(Fc:float=7):
    N = 40
    theta1_array = np.linspace(-np.pi, np.pi, N)
    theta2_array = np.linspace(-np.pi, np.pi, N)
    theta3_array = np.linspace(-np.pi, np.pi, N)
    lock_theta1 = []
    lock_theta2 = []
    lock_theta3 = []
    for theta1 in theta1_array:
        for theta2 in theta2_array:
            for theta3 in theta3_array:
                q = np.array([theta1, theta2, theta3])
                if static.is_self_lock(q, Fc):
                    lock_theta1.append(theta1)
                    lock_theta2.append(theta2)
                    lock_theta3.append(theta3)
    fig, ax = plt.subplots(squeeze=False, subplot_kw={"projection": "3d"})
    ax[0, 0].scatter(
        np.rad2deg(lock_theta1),
        np.rad2deg(lock_theta2),
        np.rad2deg(lock_theta3),
    )
    ax[0, 0].set_xlabel(r"$\theta_1$ (deg)")
    ax[0, 0].set_xlabel(r"$\theta_2$ (deg)")
    ax[0, 0].set_xlabel(r"$\theta_3$ (deg)")
    ax[0, 0].set_title("Self-locking space")
    plt.show()

def draw_self_lock_position_with_margin(Fc:float=7):
    N = 40
    theta1_array = np.linspace(-np.pi, np.pi, N)
    theta2_array = np.linspace(-np.pi, np.pi, N)
    theta3_array = np.linspace(-np.pi, np.pi, N)
    lock_theta1 = []
    lock_theta2 = []
    lock_theta3 = []
    lock_margin = []
    for theta1 in theta1_array:
        for theta2 in theta2_array:
            for theta3 in theta3_array:
                q = np.array([theta1, theta2, theta3])
                M = static.static_torque_with_f(q, np.zeros(3), 0, Fc)
                margin = np.min(Fc - np.abs(M))
                if margin >= 0:
                    lock_theta1.append(theta1)
                    lock_theta2.append(theta2)
                    lock_theta3.append(theta3)
                    lock_margin.append(margin)
    fig, ax = plt.subplots(squeeze=False, subplot_kw={"projection": "3d"})
    ax[0, 0].scatter(
        np.rad2deg(lock_theta1),
        np.rad2deg(lock_theta2),
        np.rad2deg(lock_theta3),
        c = lock_margin,
        s = 5,
        alpha = 0.2
    )
    ax[0, 0].set_xlabel(r"$\theta_1$ (deg)")
    ax[0, 0].set_xlabel(r"$\theta_2$ (deg)")
    ax[0, 0].set_xlabel(r"$\theta_3$ (deg)")
    ax[0, 0].set_title("Self-locking space")
    plt.show()

if __name__ == "__main__":
    # test
    # draw_effort_with_joint23_fixed(theta2=np.pi / 6, theta3=np.pi / 6)
    draw_self_lock_position_with_margin(5)
