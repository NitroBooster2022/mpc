import numpy as np
from scipy.linalg import block_diag
from pykalman import KalmanFilter

# Define the time step
dt = 0.1

# Define the wheelbase of the vehicle
L = 2.0

# Define the system dynamics
def transition_function(state, control):
    x, y, psi, v = state
    v_control, delta = control
    x_new = x + v * np.cos(psi) * dt
    y_new = y + v * np.sin(psi) * dt
    psi_new = psi + (v / L) * np.tan(delta) * dt
    v_new = v + v_control * dt
    return np.array([x_new, y_new, psi_new, v_new])

# Define the Jacobian of the system dynamics
def transition_function_jacobian(state, control):
    x, y, psi, v = state
    _, delta = control
    F = np.array([
        [1, 0, -v * np.sin(psi) * dt, np.cos(psi) * dt],
        [0, 1, v * np.cos(psi) * dt, np.sin(psi) * dt],
        [0, 0, 1, (1 / L) * np.tan(delta) * dt],
        [0, 0, 0, 1]
    ])
    return F

# Define the measurement model
def observation_function(state):
    x, y, psi, v = state
    ax = v * np.cos(psi)
    ay = v * np.sin(psi)
    return np.array([v, psi, ax, ay])

# Define the Jacobian of the measurement model
def observation_function_jacobian(state):
    x, y, psi, v = state
    H = np.array([
        [0, 0, 0, 1],
        [0, 0, 1, 0],
        [-v * np.sin(psi), 0, v * np.cos(psi), np.cos(psi)],
        [v * np.cos(psi), 0, v * np.sin(psi), np.sin(psi)]
    ])
    return H

# Initialize the Extended Kalman Filter
ekf = KalmanFilter(initial_state_mean=np.zeros(4),
                            transition_functions=transition_function,
                            observation_functions=observation_function,
                            transition_covariance=np.eye(4) * 0.1,
                            observation_covariance=np.eye(4) * 0.1,
                            transition_matrices=transition_function_jacobian,
                            observation_matrices=observation_function_jacobian)
