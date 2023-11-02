#!/usr/bin/env python
# coding=UTF-8

from path import Path
import os
import sys
import shutil
import errno
import timeit

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver

# import casadi as ca
import numpy as np
import scipy.linalg

from draw_acados import Draw_MPC_tracking
# from draw import Draw_MPC_tracking
import casadi as ca
from acados_template import AcadosModel
import time
import matplotlib.pyplot as plt
def plot_waypoints(waypoints, color='blue', label_suffix=''):
    """
    Parameters:
    - waypoints : np.array
        Array of shape (n x 3) where columns are x, y, and psi respectively.
    - color : str
        Color for the waypoints and arrows.
    - label_suffix : str
        Suffix for the label of the waypoints.
    """
    # Extract x, y, and psi from waypoints
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    psi = waypoints[:, 2]
    plt.figure(figsize=(10, 8))
    plt.title('Waypoints with Yaw')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.grid(True)
    plt.axis('equal')  # Equal aspect ratio ensures that the plotted waypoints are represented accurately
    plt.xlim(-20, 20)
    plt.ylim(-20, 20)
    # Plot the waypoints
    plt.plot(x, y, '-o', color=color, label=f'Waypoints {label_suffix}', markersize=4)
    # Plot arrows indicating yaw
    for xi, yi, psii in zip(x, y, psi):
        plt.arrow(xi, yi, 0.5 * np.cos(psii), 0.5 * np.sin(psii), head_width=0.2, head_length=0.3, fc=color, ec=color)
    plt.legend()
    plt.show()
def rotate_waypoints(waypoints, theta):
    """
    Rotates a set of waypoints around the origin by an angle theta.

    Parameters:
    - waypoints : np.array
        Array of shape (n x 3) where columns are x, y, and psi respectively.
    - theta : float
        The angle by which to rotate the waypoints, in radians.

    Returns:
    - rotated_waypoints : np.array
        Array of shape (n x 3) with the rotated waypoints.
    """

    # Create the 2D rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # Separate x, y, and psi from the waypoints
    xy = waypoints[:, :2]
    psi = waypoints[:, 2]

    # Apply the rotation to x, y coordinates
    rotated_xy = np.dot(xy, rotation_matrix.T)  # The rotation matrix is transposed due to the way np.dot works

    # Apply the rotation to psi (yaw) angles and normalize them
    rotated_psi = (psi + theta) % (2 * np.pi)

    rotated_psi = np.arctan2(np.sin(rotated_psi), np.cos(rotated_psi))

    # Recombine into the complete array and return
    rotated_waypoints = np.hstack((rotated_xy, rotated_psi[:, None]))  # None is used to reshape psi from (n,) to (n,1)

    return rotated_waypoints
def transform_waypoints(waypoints, theta, x, y):
    """
    Transforms a set of waypoints by rotating them around the origin by an angle theta and then translating them so that the first waypoint is at (x, y).

    Parameters:
    - waypoints : np.array
        Array of shape (n x 3) where columns are x, y, and psi respectively.
    - theta : float
        The angle by which to rotate the waypoints, in radians.
    - x : float
        The x-coordinate of the desired location for the first waypoint.
    - y : float
        The y-coordinate of the desired location for the first waypoint.

    Returns:
    - transformed_waypoints : np.array
        Array of shape (n x 3) with the transformed waypoints.
    """

    # Create the 2D rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # Separate x, y, and psi from the waypoints
    xy = waypoints[:, :2]
    psi = waypoints[:, 2]

    # Translate the waypoints such that the first waypoint is at the origin
    xy -= xy[0]

    # Apply the rotation to x, y coordinates
    rotated_xy = np.dot(xy, rotation_matrix.T)

    # Apply the rotation to psi (yaw) angles and normalize them
    rotated_psi = (psi + theta) % (2 * np.pi)
    rotated_psi = np.arctan2(np.sin(rotated_psi), np.cos(rotated_psi))

    # Translate the waypoints to the desired (x, y) location
    rotated_xy += np.array([x, y])

    # Recombine into the complete array and return
    transformed_waypoints = np.hstack((rotated_xy, rotated_psi[:, None]))

    return transformed_waypoints
class BicycleModel(object):
    def __init__(self, L=0.27):
        model = AcadosModel() #  ca.types.SimpleNamespace()
        constraint = ca.types.SimpleNamespace()
        # control inputs
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta')
        controls = ca.vertcat(v, delta)

        # n_controls = controls.size()[0]
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        states = ca.vertcat(x, y, psi)

        rhs = [v*ca.cos(psi), v*ca.sin(psi), v/L*ca.tan(delta)]
        # rhs = [v*ca.cos(psi), v*ca.sin(psi), delta]

        f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])
        # f_expl = ca.vcat(rhs)
        # acados model
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - f(states, controls)

        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []
        model.name = 'mobile_robot'

        # constraint
        constraint.v_max = 1
        constraint.v_min = -0.6
        constraint.delta_max = 0.4
        constraint.delta_min = -0.4
        constraint.x_min = 0.
        constraint.x_max = 15.
        constraint.y_min = 0.
        constraint.y_max = 15.
        constraint.expr = ca.vcat([v, delta])

        self.model = model
        self.constraint = constraint

def safe_mkdir_recursive(directory, overwrite=False):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except:
                print('Error while removing directory {}'.format(directory))


class MobileRobotOptimizer(object):
    def __init__(self, m_model, m_constraint, T, n_nodes, gazebo = False):
        model = m_model
        self.T = T
        self.t_horizon = T*n_nodes
        self.N = n_nodes

        # 设置环境变量
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        ## 获得系统中ACADOS的安装路径，请参照安装要求设置好
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)

        # x状态数
        nx = model.x.size()[0]
        self.nx = nx
        # u状态数
        nu = model.u.size()[0]
        self.nu = nu
        # ny数为x与u之和，原因详见系统CasADi例子以及ACADOS构建优化问题PDF介绍
        ny = nx + nu + 2
        # 额外参数，本例中没有
        n_params = len(model.p)

        # 构建OCP
        ocp = AcadosOcp()
        ## 设置ACADOS系统引用以及库的路径（因为ACADOS最后将以C的形式运行，所以必须设置正确）
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ## 设置模型
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.t_horizon
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        self.xy_cost = 1#*2
        self.yaw_cost = 0.5
        self.v_cost = 1 
        self.steer_cost = 0.05#.25
        self.delta_v_cost = 0.25*10
        self.delta_steer_cost = 0.5*10
        self.costs = np.array([self.xy_cost, self.yaw_cost, self.v_cost, self.steer_cost, self.delta_v_cost, self.delta_steer_cost])
        # 代价函数
        # Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, .0]])
        # R = np.array([[0.5, 0.0], [0.0, 0.05]])
        Q = np.array([[self.xy_cost, 0.0, 0.0],[0.0, self.xy_cost, 0.0],[0.0, 0.0, self.yaw_cost]])*1
        R = np.array([[self.v_cost, 0.0], [0.0, self.steer_cost]])*1
        R_delta = np.array([[self.delta_v_cost, 0.0], [0.0, self.delta_steer_cost]])
        ## cost类型为线性
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        # ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W = scipy.linalg.block_diag(Q, R, R_delta)  # Including the control rate cost
        ocp.cost.W_e = Q
        ## 这里V类矩阵的定义需要参考ACADOS构建里面的解释，实际上就是定义一些映射关系
        print("ny: ", ny, ", nx: ", nx, ", nu: ", nu)
        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)

        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[nx:nx+nu, :] = np.eye(nu) # Map the actual controls

        ocp.cost.Vz = np.zeros((ny, nu))
        ocp.cost.Vz[nx+nu:, :] = np.eye(nu) # Map the control differences
        ocp.cost.Vx_e = np.eye(nx)

        # 约束条件设置
        ocp.constraints.lbu = np.array([m_constraint.v_min, m_constraint.delta_min])
        ocp.constraints.ubu = np.array([m_constraint.v_max, m_constraint.delta_max])
        ## 这里是为了定义之前约束条件影响的index，它不需要像CasADi那样定义np.inf这种没有实际意义的约束。
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.lbx = np.array([m_constraint.x_min, m_constraint.y_min])
        ocp.constraints.ubx = np.array([m_constraint.x_max, m_constraint.y_max])
        ocp.constraints.idxbx = np.array([0, 1])

        # 一些状态的值，在实际仿真中可以重新给定，所里这里就定义一些空值
        x_ref = np.zeros(nx)
        u_ref = np.zeros(nu)
        ## 将给定值设定，注意到这里不需要像之前那样给所有N-1设定ref值，ACADOS会默认进行设置
        ocp.constraints.x0 = x_ref
        ### 0--N-1
        # ocp.cost.yref = np.concatenate((x_ref, u_ref))
        desired_delta_u = np.zeros(nu)
        ocp.cost.yref = np.concatenate((x_ref, u_ref, desired_delta_u))
        ### N
        ocp.cost.yref_e = x_ref # yref_e means the reference for the last stage

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        json_file = os.path.join('./'+model.name+'_acados_ocp.json')
        initialize_solver_time = timeit.default_timer()
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        print("initialize_solver_time: ", timeit.default_timer() - initialize_solver_time)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)
        
        self.v_ref = 0.537*1
        self.path = Path(self.v_ref, self.N, self.T)
        self.waypoints_x = self.path.waypoints_x
        self.waypoints_y = self.path.waypoints_y
        self.num_waypoints = self.path.num_waypoints
        self.wp_normals = self.path.wp_normals
        self.kappa = self.path.kappa
        self.density = self.path.density
        self.state_refs = self.path.state_refs
        self.input_refs = self.path.input_refs
        self.waypoints_x = self.state_refs[:,0]
        self.waypoints_y = self.state_refs[:,1]
        self.init_state = self.state_refs[0]
        self.local_states = np.zeros((self.N+1, 3))
        self.prev_u = np.zeros(self.nu)

        self.t0 = 0
        self.current_state = self.init_state.copy()
        self.u0 = np.zeros((self.N, 2))
        self.next_trajectories = np.tile(self.init_state, self.N+1).reshape(self.N+1, -1) # set the initial state as the first trajectories for the robot
        self.next_controls = np.zeros((self.N, 2))
        self.next_states = np.zeros((self.N+1, 3))
        self.x_c = [] # contains for the history of the state
        self.u_c = []
        self.u_cc = []
        self.t_c = [self.t0] # for the time
        self.xx = []
        self.x_refs = []
        self.x_errors = []
        self.y_errors = []
        self.yaw_errors = []
        ## start MPC
        self.mpciter = 0
        self.start_time = time.time()
        self.index_t = []
        gaz_bool = "" if not gazebo else "_gazebo_"
        filepath = os.path.dirname(os.path.abspath(__file__))
        name = 'hahaha'
        self.export_fig = os.path.join(filepath+'/gifs_acados',name + gaz_bool + '_N'+str(self.N) + '_vref'+str(self.v_ref) 
                                       + '_T'+str(self.T))
        self.obstacle = []
        self.x_min = m_constraint.x_min
        self.x_max = m_constraint.x_max
        self.y_min = m_constraint.y_min
        self.y_max = m_constraint.y_max
        self.target_waypoint_index = 0
        self.region_of_acceptance = 0.05
        self.last_waypoint_index = 0
        self.L = 0.27
        self.f_np = lambda x_, u_: np.array([u_[0]*np.cos(x_[2]), u_[0]*np.sin(x_[2]), (u_[0]/self.L)*np.tan(u_[1])])
    def shift_movement(self, x, u): 
        f_value = self.f_np(x,u)
        return x + self.T*f_value
    def update_and_solve(self, useLocal=False):
        self.target_waypoint_index = self.find_next_waypoint(self.current_state[0], self.current_state[1])
        self.next_trajectories, self.next_controls = self.path.desired_command_and_trajectory(self.target_waypoint_index)
        xs = self.state_refs[self.target_waypoint_index].copy()
        self.local_states = self.next_trajectories.copy()
        # plt.figure(figsize=(10, 8))
        # plt.title('Waypoints with Yaw')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.grid(True)
        # plt.axis('equal')  # Equal aspect ratio ensures that the plotted waypoints are represented accurately
        # plt.xlim(-20, 20)
        # plt.ylim(-20, 20)
        if useLocal:
            # print("local: ", np.around(self.local_states[0],3), ", xs: ", np.around(xs,3), "cur: ", np.around(self.current_state,3))
            self.local_states = transform_waypoints(self.local_states, np.pi, 5, 5)
            xs = transform_waypoints(xs.reshape(1,-1), np.pi).reshape(-1)
            # print(self.local_states.shape, xs.shape)
            # plot_waypoints(self.local_states, color='red', label_suffix='2')
            # plot_waypoints(self.next_trajectories, color='blue', label_suffix='1')
            # plt.legend()
            # plt.show()
            # exit()
        self.solver.set(self.N, 'yref', xs)
        # print x cur and ref state
        # print("x cur: ", np.around(self.current_state,3), ", x ref: ", np.around(xs,3))
        for j in range(self.N):
            if self.target_waypoint_index+j >= self.state_refs.shape[0]:
                self.solver.set(j, 'yref', np.concatenate((self.local_states[-1], np.zeros(2), np.zeros(2))))
            else:
                # if self.target_waypoint_index > 375 and self.target_waypoint_index < 500:
                #     print("idx: ", self.target_waypoint_index, "j: ", j,", ref yaw: ", round(self.state_refs[self.target_waypoint_index+j,2],3), "cur yaw: ", round(self.current_state[2],2))
                ref_u = self.input_refs[self.target_waypoint_index+j]
                ref_delta_u = ref_u - (self.solver.get(j-1, 'u') if j > 0 else np.zeros_like(ref_u))
                self.solver.set(j, 'yref', np.concatenate((self.local_states[j], self.next_controls[j], ref_delta_u)))# 设置当前循环x0 (stage 0)
        # update initial condition
        self.solver.set(0, 'lbx', self.current_state)
        self.solver.set(0, 'ubx', self.current_state)
        # 求解
        status = self.solver.solve()
        if status != 0 :
            print('ERROR!!! acados acados_ocp_solver returned status {}. Exiting.'.format(status))
            return None
        # 得到下个时刻最优控制
        next_u = self.solver.get(0, 'u')
        self.prev_u = next_u.copy()
        return next_u
    def integrate_next_states(self, u_res=None):
        # 以下纯粹为了仿真
        # 仿真器获得当前位置和控制指令
        self.integrator.set('x', self.current_state)
        self.integrator.set('u', u_res)
        # 仿真器计算结果
        status_s = self.integrator.solve()
        if status_s != 0:
            raise Exception('acados integrator returned status {}. Exiting.'.format(status_s))
        # 将仿真器计算的小车位置作为下一个时刻的初始值
        self.current_state = self.integrator.get('x')
        self.t_c.append(self.t0)
        self.u_c.append(u_res)
        self.t0 += self.T

    def update_current_state(self, x, y, yaw):
        # Ensure target_waypoint_index is within bounds
        if self.target_waypoint_index < len(self.state_refs):
            # Extract the reference yaw at the current target waypoint
            ref_yaw = self.state_refs[self.target_waypoint_index, 2]
            # Adjust yaw to minimize its difference with ref yaw
            while yaw - ref_yaw > np.pi:
                yaw -= 2 * np.pi
            while yaw - ref_yaw < -np.pi:
                yaw += 2 * np.pi
        self.current_state = np.array([x, y, yaw])
    def update_real_state(self, x, y, yaw):
        if self.target_waypoint_index < len(self.state_refs):
            ref_yaw = self.state_refs[self.target_waypoint_index, 2]
            while yaw - ref_yaw > np.pi:
                yaw -= 2 * np.pi
            while yaw - ref_yaw < -np.pi:
                yaw += 2 * np.pi
        self.real_state = np.array([x, y, yaw])
    def find_closest_waypoint(self, x, y):
        distances = np.linalg.norm(np.vstack((self.waypoints_x, self.waypoints_y)).T - np.array([x, y]), axis=1)
        index = np.argmin(distances)
        return index, distances[index]
    def find_next_waypoint(self, x, y):
        closest_idx, dist_to_waypoint = self.find_closest_waypoint(x, y)
        # ensure we're moving forward in the waypoint list, but not jumping too far ahead
        if dist_to_waypoint < self.region_of_acceptance:
            if closest_idx - self.last_waypoint_index < 15:
                self.last_waypoint_index = max(self.last_waypoint_index, closest_idx+1)
            else:
                print("here: ", closest_idx, self.last_waypoint_index, closest_idx - self.last_waypoint_index)
                closest_idx = self.last_waypoint_index
                # self.last_waypoint_index += 1
        else:
            if closest_idx - self.last_waypoint_index > 15:
                print("here2: ", closest_idx, self.last_waypoint_index, closest_idx - self.last_waypoint_index)
                closest_idx = self.last_waypoint_index + 1
            # If not within the region of acceptance, take smaller steps forward in the waypoint list
            self.last_waypoint_index += 1
        # print("dist_to_waypoint: ", dist_to_waypoint, ", closest_idx: ", closest_idx, ", last_waypoint_index: ", self.last_waypoint_index)
        target_idx = max(self.last_waypoint_index, closest_idx)
        return min(target_idx, len(self.waypoints_x) - 1)
   # 开始仿真
    def simulation(self, x0, path):
        # self.last_waypoint_index = 357
        # self.target_waypoint_index = 357
        # 初始位置
        self.current_state = self.state_refs[self.last_waypoint_index]
        self.init_state = self.current_state 
        print("x start: ", self.current_state)
        # 存储仿真结果
        simX = [self.current_state]
        simU = []
        simRef = [self.current_state]
        # 只是为了计算运行时间
        time_record = []

		# 闭环仿真
        i = -1
        for _ in range(500):
            if self.target_waypoint_index >= self.state_refs.shape[0]-1: 
                break
        # print("shape1: ", self.state_refs.shape[0], ", shape2: ", self.input_refs.shape[0])
        while self.target_waypoint_index < self.num_waypoints-1:
            i += 1
            start = timeit.default_timer()
            idx1 = self.last_waypoint_index
            # u_current = self.update_and_solve()
            # print("u1: ", u_current)
            # self.last_waypoint_index = idx1
            # u2 = self.update_and_solve()
            # print("u1-u2: ", u_current-u2)
            u_current = self.update_and_solve()
            if u_current is None:
                break
            # print("u2: ", u2)
            # if i>2:
            #     exit()
            simU.append(u_current)
            # 计时结束
            time_record.append(timeit.default_timer() - start)
            self.integrate_next_states(u_current)
            simX.append(self.current_state)
            simRef.append(self.state_refs[self.target_waypoint_index])
            # print(i,") cur idx: ", self.target_waypoint_index, ", x: ", round(self.current_state[0],2), ", y: ", round(self.current_state[1],2), ", psi: ", round(self.current_state[2],2), ", v: ", round(u_current[0],2), ", w: ", round(u_current[1],2), ", inputrefs: ", self.input_refs[self.target_waypoint_index], ", staterefs: ", self.state_refs[self.target_waypoint_index])
        simX = np.array(simX)
        simU = np.array(simU)
        simRef = np.array(simRef)
        time_record = np.array(time_record)
        print("average estimation time is {}".format(time_record.mean()))
        print("max estimation time is {}".format(time_record.max()))
        print("min estimation time is {}".format(time_record.min()))
        # print("simU: ", simU)
        print("simX: ", simX.shape, "ref: ", simRef.shape)
        # Draw_MPC_point_stabilization_v1(rob_diam=0.3, init_state=x0, target_state=xs, robot_states=simX, )
        draw_result = Draw_MPC_tracking(simU, simX, simRef, x0,
                                       self.state_refs[:,0], 
                                      self.state_refs[:,1])
    def draw_result(self, stats):
        Draw_MPC_tracking(self.u_c, obstacle = self.obstacle, rob_diam=0.3, init_state=self.init_state, 
                        robot_states=self.xx, ref_states = self.x_refs, export_fig=self.export_fig, waypoints_x=self.waypoints_x, 
                        waypoints_y=self.waypoints_y, stats = stats, costs = self.costs, xmin=self.x_min, xmax=self.x_max, ymin=self.y_min, ymax=self.y_max, times = self.t_c)
    def compute_stats(self):
        ## after loop
        print("iter: ", self.mpciter)
        t_v = np.array(self.index_t)
        print("mean solve time: ",t_v.mean(), "max: ", t_v.max(), "min: ", t_v.min(), "std: ", t_v.std(), "median: ", np.median(t_v))
        print((time.time() - self.start_time)/(self.mpciter))
        # print the control input, keep 2 digits after the decimal point
        self.u_c = np.array(self.u_c)
        self.x_c = np.array(self.x_c)

        print("average kappa: ", np.mean(self.kappa))
        average_speed = np.mean(self.u_c[:, 0])
        average_steer = np.mean(self.u_c[:, 1])
        
        delta_u_c = np.diff(self.u_c, axis=0)
        average_delta_speed = np.mean(np.abs(delta_u_c[:, 0]))
        average_delta_steer = np.mean(np.abs(delta_u_c[:, 1]))
        
        print(f"Average speed: {average_speed:.4f} m/s")
        print(f"Average steer angle: {average_steer:.4f} rad")
        print(f"Average change in speed: {average_delta_speed:.4f} m/s²")
        print(f"Average change in steer angle: {average_delta_steer:.4f} rad/s")

        average_x_error = np.mean(np.abs(self.x_errors))
        average_y_error = np.mean(np.abs(self.y_errors))
        self.yaw_errors = np.array(self.yaw_errors)
        self.yaw_errors = np.arctan2(np.sin(self.yaw_errors), np.cos(self.yaw_errors))
        average_yaw_error = np.mean(np.abs(self.yaw_errors))

        print(f"Average x error: {average_x_error:.4f} m")
        print(f"Average y error: {average_y_error:.4f} m")
        print(f"Average yaw error: {average_yaw_error:.4f} rad")

        stats = [average_speed, average_steer, average_delta_speed, average_delta_steer, average_x_error, average_y_error, average_yaw_error]
        return stats
    
if __name__ == '__main__':
    mobile_robot_model = BicycleModel()
    opt = MobileRobotOptimizer(m_model=mobile_robot_model.model,
                               m_constraint=mobile_robot_model.constraint, T=0.1, n_nodes=100)
    # opt.simulation(x0=np.array([0, 0, 0]), xs=np.array([2., 1.5, 0.]))
    path = Path()
    x0 = path.state_refs[0]
    opt.simulation(x0=x0, path = path)
