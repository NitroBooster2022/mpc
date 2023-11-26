#pragma once
#include <Eigen/Dense>
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_mobile_robot.h"
#include "acados_solver_mobile_robot.h"

class Optimizer {
public:
    Optimizer(double x_init = 0.83, double y_init = 0.33, double yaw_init = 1.65756);
    int run(); 
    int update_and_solve();
    void integrate_next_states();
    int find_next_waypoint();
    void update_real_states(double x, double y, double yaw);
    void update_current_states(double x, double y, double yaw);
    Eigen::VectorXd computeStats();
    std::string getSourceDirectory();
    template <typename EigenType>
    void saveToFile(const EigenType &data, const std::string &filename);
    Eigen::MatrixXd loadTxt(const std::string &filename);

// private:
    int status; // acados operation state
    int idxbx0[3];
    double min_time;
    double elapsed_time;
    double x_state[5];
    double x_current[3];
    double u_current[2];
    int N, nx, nu;
    int target_waypoint_index, last_waypoint_index, num_waypoints;
    double region_of_acceptance, t0;
    double T = 0.125;
    Eigen::Vector3d current_state;
    Eigen::MatrixXd state_refs, input_refs;
    Eigen::VectorXd distances;

    mobile_robot_solver_capsule *acados_ocp_capsule;
    mobile_robot_sim_solver_capsule *sim_capsule;
    sim_config *mobile_robot_sim_config;
    void *mobile_robot_sim_dims;
    sim_in *mobile_robot_sim_in;
    sim_out *mobile_robot_sim_out;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;

    Eigen::MatrixXd simX, simU;
    Eigen::VectorXd time_record, x_errors, y_errors, yaw_errors;

};