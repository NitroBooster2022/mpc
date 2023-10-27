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

# from draw_acados import Draw_MPC_tracking
from draw import Draw_MPC_tracking
import casadi as ca
from acados_template import AcadosModel
import time

class BicycleModel(object):
    def __init__(self, L=0.27):
        model = AcadosModel() #  ca.types.SimpleNamespace()
        constraint = ca.types.SimpleNamespace()
        # control inputs
        v = ca.SX.sym('v')
        delta = ca.SX.sym('delta')
        controls = ca.vertcat(v, delta)

        delta_v = ca.SX.sym('delta_v')
        delta_delta = ca.SX.sym('delta_delta')
        delta_controls = ca.vertcat(delta_v, delta_delta)

        # n_controls = controls.size()[0]
        # model states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        psi = ca.SX.sym('psi')
        states = ca.vertcat(x, y, psi)

        rhs = [v*ca.cos(psi), v*ca.sin(psi), v/L*ca.tan(delta)]

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
        constraint.x_max = 20.
        constraint.y_min = 0.
        constraint.y_max = 20.
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
    def __init__(self, m_model, m_constraint, t_horizon, n_nodes, gazebo = False):
        model = m_model
        self.t_horizon = t_horizon
        self.T = t_horizon / n_nodes
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
        self.yaw_cost = 0#.5
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
        ocp.constraints.lbx = np.array([m_constraint.x_min, m_constraint.y_min, -40000*np.pi])
        ocp.constraints.ubx = np.array([m_constraint.x_max, m_constraint.y_max, 40000*np.pi])
        ocp.constraints.idxbx = np.array([0, 1,2])

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
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.integrator = AcadosSimSolver(ocp, json_file=json_file)
        
        self.v_ref = 0.537*1
        self.path = Path(self.v_ref, self.N)
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
    def update_and_solve(self):
        self.target_waypoint_index = self.find_next_waypoint(self.current_state[0], self.current_state[1])
        self.next_trajectories, self.next_controls = self.path.desired_command_and_trajectory(self.target_waypoint_index)
        xs = self.state_refs[self.target_waypoint_index]
        # make sure yaw between -pi and pi
        # self.current_state[2] = np.arctan2(np.sin(self.current_state[2]), np.cos(self.current_state[2]))
        self.solver.set(self.N, 'yref', xs)
        for j in range(self.N):
            if self.target_waypoint_index+j >= self.state_refs.shape[0]:
                self.solver.set(j, 'yref', np.concatenate((self.state_refs[-1], np.zeros(2), np.zeros(2))))
            else:
                if j == 0:
                    delta_u_ref = self.input_refs[self.target_waypoint_index] - self.prev_u
                    print("delta_u_ref: ", delta_u_ref)
                else:
                    delta_u_ref = self.input_refs[self.target_waypoint_index+j] - self.input_refs[self.target_waypoint_index+j-1]
                self.solver.set(j, 'yref', np.concatenate((self.state_refs[self.target_waypoint_index+j], self.input_refs[self.target_waypoint_index+j], delta_u_ref)))
                # self.solver.set(j, 'yref', np.concatenate((self.state_refs[self.target_waypoint_index+j], self.input_refs[self.target_waypoint_index+j]))) 
        # 设置当前循环x0 (stage 0)
        self.solver.set(0, 'lbx', self.current_state)
        self.solver.set(0, 'ubx', self.current_state)
        # 求解
        status = self.solver.solve()
        if status != 0 :
            raise Exception('acados acados_ocp_solver returned status {}. Exiting.'.format(status))
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
        # make sure orientation is within [-pi, pi]
        if self.current_state[2] > np.pi:
            self.current_state[2] -= 2*np.pi
        elif self.current_state[2] < -np.pi:
            self.current_state[2] += 2*np.pi
        self.t_c.append(self.t0)
        self.u_c.append(u_res)
        self.t0 += self.T

    def update_current_state(self, x, y, yaw):
        self.current_state = np.array([x, y, yaw])
    def update_real_state(self, x, y, yaw):
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
        self.last_waypoint_index = 0
        # 初始位置
        self.current_state = self.state_refs[self.last_waypoint_index]
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
            u_current = self.update_and_solve()
            simU.append(u_current)
            # 计时结束
            time_record.append(timeit.default_timer() - start)
            self.integrate_next_states(u_current)
            simX.append(self.current_state)
            simRef.append(self.state_refs[self.target_waypoint_index])
            # print(i,") cur idx: ", self.target_waypoint_index, ", x: ", round(self.current_state[0],2), ", y: ", round(self.current_state[1],2), ", psi: ", round(self.current_state[2],2), ", v: ", round(u_current[0],2), ", w: ", round(u_current[1],2), ", inputrefs: ", self.input_refs[self.target_waypoint_index])
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
                               m_constraint=mobile_robot_model.constraint, t_horizon=2.45, n_nodes=10)
    # opt.simulation(x0=np.array([0, 0, 0]), xs=np.array([2., 1.5, 0.]))
    path = Path()
    x0 = path.state_refs[0]
    opt.simulation(x0=x0, path = path)
