import numpy as np
def forward_kinematics(q, L):
    """输入角度和臂长, 返回后两个关节和末端位置, 三根杆的质心位置"""
    L1, L2, L3 = L
    theta1, theta2, theta3 = q
    a1 = theta1
    a2 = theta1 + theta2
    a3 = theta1 + theta2 + theta3

    A = L1 * np.array([
        np.cos(a1),
        np.sin(a1)
    ])

    B = A + L2 * np.array([
        np.cos(a2),
        np.sin(a2)
    ])

    C = B + L3 * np.array([
        np.cos(a3),
        np.sin(a3)
    ])

    C1 = 0.5 * A
    C2 = A + 0.5 * L2 * np.array([
        np.cos(a2),
        np.sin(a2)
    ])
    C3 = B + 0.5 * L3 * np.array([
        np.cos(a3),
        np.sin(a3)
    ])
    
    return (A, B, C, C1, C2, C3)

def Jacobbin_end(q, L):
    l1, l2, l3 = L
    theta1, theta2, theta3 = q
    a1 = theta1
    a2 = theta1 + theta2
    a3 = theta1 + theta2 + theta3
    s1 = np.sin(a1)
    s2 = np.sin(a2)
    s3 = np.sin(a3)
    c1 = np.cos(a1)
    c2 = np.cos(a2)
    c3 = np.cos(a3)
    dr_da = np.array([[l1 * (-s1), l2 * (-s2), l3 * (-s3)], 
                      [l1 * c1, l2 * c2, l3 * c3]])
    da_dtheta = np.array([[1, 0, 0],
                          [1, 1, 0],
                          [1, 1, 1]])
    dr_dtheta = dr_da @ da_dtheta
    return dr_dtheta

def Jacobbin_1(q, L):
    l1, _, _ = L
    theta1, _, _ = q
    a1 = theta1
    s1 = np.sin(a1)
    c1 = np.cos(a1)
    dr_da = np.array([
        [0.5 * l1 * (-s1),0, 0],
        [0.5 * l1 * c1, 0, 0]
        ])
    da_dtheta = np.array([[1, 0, 0],
                          [1, 1, 0],
                          [1, 1, 1]])
    dr_dtheta = dr_da @ da_dtheta
    return dr_dtheta

def Jacobbin_2(q, L):
    l1, l2, _ = L
    theta1, theta2, _ = q
    a1 = theta1
    a2 = theta1 + theta2
    s1 = np.sin(a1)
    c1 = np.cos(a1)
    s2 = np.sin(a2)
    c2 = np.cos(a2)
    dr_da = np.array([
        [l1 * (-s1),    0.5 * l2 * (-s2),   0],
        [l1 * c1,       0.5 * l2 * c2,      0]
        ])
    da_dtheta = np.array([[1, 0, 0],
                          [1, 1, 0],
                          [1, 1, 1]])
    dr_dtheta = dr_da @ da_dtheta
    return dr_dtheta

def Jacobbin_3(q, L):
    l1, l2, l3 = L
    theta1, theta2, theta3 = q
    a1 = theta1
    a2 = theta1 + theta2
    a3 = theta1 + theta2 + theta3
    s1 = np.sin(a1)
    s2 = np.sin(a2)
    s3 = np.sin(a3)
    c1 = np.cos(a1)
    c2 = np.cos(a2)
    c3 = np.cos(a3)
    dr_da = np.array([
              [l1 * (-s1),  l2 * (-s2), 0.5 * l3 * (-s3)], 
              [l1 * c1,     l2 * c2,    0.5 * l3 * c3]])
    da_dtheta = np.array([[1, 0, 0],
                          [1, 1, 0],
                          [1, 1, 1]])
    dr_dtheta = dr_da @ da_dtheta
    return dr_dtheta

def fric_v(q_dot, b):
    return -(b * q_dot).astype(float)

def fric_c(q_dot, Fc):
    return -(np.sign(q_dot) * Fc).astype(float)

def fric_cv(q_dot, b, Fc):
    return (fric_v(q_dot, b) + fric_c(q_dot, Fc)).astype(float)

def consider_f(M, q_dot, b, Fc):
    # align the shape of Fc to M.shape
    Fc = np.broadcast_to(Fc, M.shape)
    fs = fric_cv(q_dot, b, Fc)
    for i, _ in enumerate(fs):
        if np.isclose(q_dot[i], 0): # 如果角速度是0, 静态分析, f会尽量让M变小
            f = -np.sign(M[i]) * np.minimum(Fc[i], np.abs(M[i]))
            fs[i] = f
    return M + fs

# if __name__ == "__main__":
    # test
    # a = np.array([1, -1, 0, -3])
    # print(fric_cv(a, 0.02, 2))
    # print(np.minimum(2, np.abs(-1.2)))
