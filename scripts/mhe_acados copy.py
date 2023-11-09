import numpy as np
from casadi import SX, vertcat, Function
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# control inputs
v = ca.SX.sym('v')
delta = ca.SX.sym('delta')
controls = ca.vertcat(v, delta)
# model states
x = ca.SX.sym('x')
y = ca.SX.sym('y')
psi = ca.SX.sym('psi')
states = ca.vertcat(x, y, psi)
L = 0.27
rhs = [v*ca.cos(psi), v*ca.sin(psi), v/L*ca.tan(delta)]

f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])
# acados model
x_dot = ca.SX.sym('x_dot', len(rhs))
f_impl = x_dot - f(states, controls)

model = AcadosModel()
model.f_expl_expr = f(states, controls)
model.f_impl_expr = f_impl
model.x = states
model.xdot = x_dot
model.u = controls
model.p = []
model.name = 'bicycle_model'

# Create an acados OCP object for the MHE problem
ocp = AcadosOcp()
ocp.model = model

# MHE horizon length
N = 20
ocp.solver_options.tf = N * 0.1  # Time horizon

# Set up the cost function - typically, the cost for an MHE problem would include terms
# for the error between the measured and estimated states and possibly for the input efforts.
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.cost.cost_type_e = 'NONLINEAR_LS'  # Cost type for terminal cost

nx = model.x.size()[0]
# u状态数
nu = model.u.size()[0]
# ny数为x与u之和，原因详见系统CasADi例子以及ACADOS构建优化问题PDF介绍
ny = nx + nu

# Your weights matrix should be square with dimensions ny x ny
Q_mhe = np.diag([1.0, 1.0, 0.1])

# Make sure your reference is also the correct size
yref = np.zeros((ny,))

# Now set the dimensions and weights in your OCP
ocp.dims.N = N
ocp.dims.ny = ny  # Number of measurements
ocp.dims.ny_e = ny  # Number of measurements at the terminal stage
ocp.cost.W = Q_mhe  # Set the weight matrix
ocp.cost.W_e = Q_mhe  # Set the weight matrix for terminal cost
ocp.cost.yref = yref  # Set the reference for the output
ocp.cost.yref_e = yref  # Set the reference for the output at the terminal stage
ocp.model.cost_y_expr = states
ocp.model.cost_y_expr_e = ocp.model.cost_y_expr
print("cost_y_expr shape:", ocp.model.cost_y_expr.size())
print("cost_y_expr_e shape:", ocp.model.cost_y_expr_e.size())

# Set up constraints (if any)
# For example, you can add bounds on the inputs if needed
ocp.constraints.lbu = np.array([-1.0, -np.deg2rad(30)])  # Lower bounds on v and delta
ocp.constraints.ubu = np.array([1.0, np.deg2rad(30)])    # Upper bounds on v and delta
ocp.constraints.idxbu = np.array([0, 1])  # Index of controls subject to bounds

# Configure the solver options
ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.tf = 0.1  # Sampling time

# Generate the acados solver for the MHE problem
mhe_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

# Simulation parameters
T = 10  # Total simulation time in seconds
dt = 0.1  # Sampling time
N_sim = int(T/dt)  # Number of simulation steps
v_true = 2.0  # True speed in m/s
L = 2.0  # Wheelbase in meters
R = 10.0  # Radius of the circle in meters
delta_true = np.arctan(L/R)  # Steering angle for a circle of radius R

# Initialize state trajectory
x_true = np.zeros(N_sim)
y_true = np.zeros(N_sim)
psi_true = np.zeros(N_sim)

for i in range(1, N_sim):
    psi_true[i] = psi_true[i-1] + (v_true / L) * np.tan(delta_true) * dt
    x_true[i] = x_true[i-1] + v_true * np.cos(psi_true[i-1]) * dt
    y_true[i] = y_true[i-1] + v_true * np.sin(psi_true[i-1]) * dt

# True states
true_states = np.vstack((x_true, y_true, psi_true)).T
# Noise standard deviations
sigma_x = 0.1  # Standard deviation for x position
sigma_y = 0.1  # Standard deviation for y position
sigma_psi = np.deg2rad(1)  # Standard deviation for psi in radians

# Generate noisy measurements
measurements = true_states + np.random.normal(0, [sigma_x, sigma_y, sigma_psi], true_states.shape)

# Assume the initial guess is all zeros (can be set to a better guess if available)
initial_guess = np.zeros((N, n_states))

# Run MHE with simulated measurements
estimated_states = []
for i in range(N_sim - N):
    # Set the measurements for the current window
    for j in range(N):
        mhe_solver.set(j, "yref", measurements[i + j])

    # Solve the MHE problem
    status = mhe_solver.solve()
    
    if status != 0:
        print(f"MHE solver returned status {status}")

    # Get the estimated state at the current time
    x_est = mhe_solver.get(0, "x")
    estimated_states.append(x_est.tolist())

    # Set initial guess for the next window to speed up convergence
    for j in range(N - 1):
        mhe_solver.set(j, "x", mhe_solver.get(j + 1, "x"))
    mhe_solver.set(N - 1, "x", x_est)

estimated_states = np.array(estimated_states)
