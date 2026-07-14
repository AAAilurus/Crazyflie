import numpy as np
from scipy.linalg import expm, solve_discrete_are

# ------------------------------------------------------------
# Crazyflie parameters
# ------------------------------------------------------------

m = 0.025 + 4.0 * 0.0008

Ixx = 16.571710e-6
Iyy = 16.655602e-6
Izz = 29.261652e-6

g = 9.81

kf = 1.28192e-8
moment_constant = 0.005964552
km = kf * moment_constant

arm = 0.031

omega_hover = np.sqrt(m * g / (4.0 * kf))
c_thrust = 2.0 * kf * omega_hover

# ------------------------------------------------------------
# Continuous-time model
#
# state:
# [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
#
# input:
# [collective, roll, pitch, yaw] motor-speed corrections
# ------------------------------------------------------------

Ac = np.zeros((12, 12))
Bc = np.zeros((12, 4))

Ac[0, 3] = 1.0
Ac[1, 4] = 1.0
Ac[2, 5] = 1.0

Ac[3, 7] = g
Ac[4, 6] = -g

Ac[6, 9] = 1.0
Ac[7, 10] = 1.0
Ac[8, 11] = 1.0

Bc[5, 0] = 4.0 * c_thrust / m
Bc[9, 1] = 4.0 * arm * c_thrust / Ixx
Bc[10, 2] = 4.0 * arm * c_thrust / Iyy
Bc[11, 3] = 8.0 * km * omega_hover / Izz

# ------------------------------------------------------------
# Sampling time
# Must match the controller update period.
# Example: 1 kHz => Ts = 0.001 s
# ------------------------------------------------------------

Ts = 0.001

# ------------------------------------------------------------
# Exact zero-order-hold discretization
#
# exp([[Ac, Bc],
#      [ 0,  0 ]] Ts)
# =
# [[Ad, Bd],
#  [ 0,  I ]]
# ------------------------------------------------------------

n = Ac.shape[0]
m_in = Bc.shape[1]

M = np.zeros((n + m_in, n + m_in))
M[:n, :n] = Ac
M[:n, n:] = Bc

Md = expm(M * Ts)

Ad = Md[:n, :n]
Bd = Md[:n, n:]

# ------------------------------------------------------------
# Discrete LQR weights
# ------------------------------------------------------------

Q = np.diag([
    400.0,   # x
    400.0,   # y
    180.0,   # z

    6.0,     # vx
    6.0,     # vy
    35.0,    # vz

    100.0,   # roll
    100.0,   # pitch
    50.0,    # yaw

    12.0,    # p
    12.0,    # q
    10.0     # r
])

R = np.diag([
    0.25,    # collective
    0.12,    # roll
    0.12,    # pitch
    1.50     # yaw
])

# ------------------------------------------------------------
# Solve DARE and compute discrete LQR gain
# ------------------------------------------------------------

P = solve_discrete_are(Ad, Bd, Q, R)

Kd = np.linalg.solve(
    R + Bd.T @ P @ Bd,
    Bd.T @ P @ Ad
)

eig_cl = np.linalg.eigvals(Ad - Bd @ Kd)

np.set_printoptions(
    precision=10,
    suppress=True,
    linewidth=240
)

print("Ts =", Ts)
print("mass =", m)
print("omega_hover =", omega_hover)

print("\nAc =")
print(Ac)

print("\nBc =")
print(Bc)

print("\nAd =")
print(Ad)

print("\nBd =")
print(Bd)

print("\nQ =")
print(Q)

print("\nR =")
print(R)

print("\nP =")
print(P)

print("\nKd =")
print(Kd)

print("\nclosed-loop discrete eigenvalues =")
print(eig_cl)

print("\nmaximum eigenvalue magnitude =")
print(np.max(np.abs(eig_cl)))

print("\nC++ initializer:")
for row in Kd:
    print(
        "{" +
        ", ".join(f"{value:.12f}" for value in row) +
        "},"
    )
