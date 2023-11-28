#include "optimizer.hpp"
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <limits.h>
#include <cmath>
// #include <math.h>
// #include <interpolation.h> // Alglib header for spline interpolation

#include <Eigen/Dense>
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_mobile_robot.h"
#include "acados_solver_mobile_robot.h"

Optimizer::Optimizer(double T, int N, double v_ref, double x_init, double y_init, double yaw_init):
    T(T), N(N), v_ref(v_ref)
 {
    // Initialize member variables
    min_time = 1e12;
    status = 0; // Assuming 0 is a default 'no error' state

    // Create a capsule according to the pre-defined model
    acados_ocp_capsule = mobile_robot_acados_create_capsule();

    // Initialize the optimizer
    status = mobile_robot_acados_create(acados_ocp_capsule);
    if (status) {
        printf("mobile_robot_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    // Create and initialize simulator capsule
    sim_capsule = mobile_robot_acados_sim_solver_create_capsule();
    status = mobile_robot_acados_sim_create(sim_capsule);

    mobile_robot_sim_config = mobile_robot_acados_get_sim_config(sim_capsule);
    mobile_robot_sim_dims = mobile_robot_acados_get_sim_dims(sim_capsule);
    mobile_robot_sim_in = mobile_robot_acados_get_sim_in(sim_capsule);
    mobile_robot_sim_out = mobile_robot_acados_get_sim_out(sim_capsule);

    if (status) {
        printf("acados_create() simulator returned status %d. Exiting.\n", status);
        exit(1);
    }

    // Initialize some important structure of ocp
    nlp_config = mobile_robot_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims = mobile_robot_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in = mobile_robot_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out = mobile_robot_acados_get_nlp_out(acados_ocp_capsule);

    // Setting problem dimensions
    N = nlp_dims->N;
    nx = *nlp_dims->nx;
    nu = *nlp_dims->nu;
    printf("N = %d, nx = %d, nu = %d\n", N, nx, nu);

    // Initialize target, current state and state variables
    x_current[0] = x_init;
    x_current[1] = y_init;
    x_current[2] = yaw_init;

    // x_current[0] = 14.17;
    // x_current[1] = 0.33;
    // x_current[2] = 1.48402932;

    x_state[0] = 0.0;
    x_state[1] = 0.0;
    x_state[2] = 0.0;
    x_state[3] = 0.0;
    x_state[4] = 0.0;

    current_state << x_current[0], x_current[1], x_current[2];
    target_waypoint_index = 0;
    last_waypoint_index = 0;
    region_of_acceptance = 0.03076923;
    state_refs = loadTxt("/home/simonli/bfmc_pkgs/mpc/scripts/paths/state_refs.txt");
    input_refs = loadTxt("/home/simonli/bfmc_pkgs/mpc/scripts/paths/input_refs.txt");
    num_waypoints = state_refs.rows();
    std::cout << "state_refs shape: " << state_refs.rows() << ", " << state_refs.cols() << std::endl;

    int len = static_cast<int>(num_waypoints * 1.5);
    simX = Eigen::MatrixXd::Zero(len, nx); 
    simU = Eigen::MatrixXd::Zero(len, nu);
    x_errors = Eigen::VectorXd::Zero(len, 1);
    y_errors = Eigen::VectorXd::Zero(len, 1);
    yaw_errors = Eigen::VectorXd::Zero(len, 1);
    time_record = Eigen::VectorXd::Zero(len, nu);
}

int Optimizer::run() {
    std::cout.precision(3);
    int hsy = 0;
    Eigen::VectorXd x_final = state_refs.row(state_refs.rows() - 1);
    while(1) {
        double error_norm = (x_final - current_state).norm();
        if(target_waypoint_index > num_waypoints || hsy > 500 || error_norm < 0.1) {
            break;
        }
        std::cout << "target_waypoint_index: " << target_waypoint_index << ", hsy: " << hsy << ", norm: " << error_norm << std::endl;
        x_errors(hsy) = state_refs(target_waypoint_index, 0) - x_current[0];
        y_errors(hsy) = state_refs(target_waypoint_index, 1) - x_current[1];
        yaw_errors(hsy) = state_refs(target_waypoint_index, 2) - x_current[2];
        auto t_start = std::chrono::high_resolution_clock::now();
        update_and_solve();
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        time_record(hsy) = elapsed_time_ms;
        integrate_next_states();
        for(int i=0; i<nu; i++) {
            simU(hsy, i) = u_current[i];
        }
        for(int ii=0; ii<nx; ii++) {
            simX(hsy, ii) = x_current[ii];
        }
        hsy++;
    }
    Eigen::VectorXd stats = computeStats(hsy);
    return status;
}

int Optimizer::update_and_solve() {
    auto t_start = std::chrono::high_resolution_clock::now();
    if(debug) {
        x_errors(iter) = state_refs(target_waypoint_index, 0) - x_current[0];
        y_errors(iter) = state_refs(target_waypoint_index, 1) - x_current[1];
        yaw_errors(iter) = state_refs(target_waypoint_index, 2) - x_current[2];
        // auto t_start = std::chrono::high_resolution_clock::now();
    }
    target_waypoint_index = find_next_waypoint();
    int idx = target_waypoint_index;

    for(int i=0; i<3; i++) {
        x_state[i] = state_refs(target_waypoint_index, i);
    }
    x_state[3] = 0.0;
    x_state[4] = 0.0;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_state);

    for (int j = 0; j < N; ++j) {
        if (target_waypoint_index + j >= state_refs.rows()) {
            for(int i=0; i<3; i++) {
                x_state[i] = state_refs(state_refs.rows() - 1, i);
            }
            x_state[3] = 0.0;
            x_state[4] = 0.0;
        } else {
            for (int i = 0; i < 3; ++i) {
                x_state[i] = state_refs(idx + j, i);
            }
            for (int i = 0; i < 2; ++i) {
                x_state[i + 3] = input_refs(idx + j, i);
            }
        }
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", x_state);
    }

    // Set the constraints for the current state
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);

    // Solve the optimization problem
    status = mobile_robot_acados_solve(acados_ocp_capsule);
    if (status != 0) {
        std::cout << "ERROR!!! acados acados_ocp_solver returned status " << status << ". Exiting." << std::endl;
        return 1; 
    }

    // Get the optimal control for the next step
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);

    //debug
    double error = (x_current[0] - state_refs(idx, 0)) * (x_current[0] - state_refs(idx, 0)) + (x_current[1] - state_refs(idx, 1)) * (x_current[1] - state_refs(idx, 1));
    if (debug) {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        // std::cout << "elapsed time: " << elapsed_time_ms << " ms" << std::endl;
        time_record(iter) = elapsed_time_ms;
        for(int i=0; i<nu; i++) {
            simU(iter, i) = u_current[i];
        }
        for(int ii=0; ii<nx; ii++) {
            simX(iter, ii) = x_current[ii];
        }
        // std::cout << iter<< ") x_cur: " << x_current[0] << ", " << x_current[1] << ", " << x_current[2] << ", ref: " << state_refs(idx, 0) << ", " << state_refs(idx, 1) << ", " << state_refs(idx, 2) << ", u: " << u_current[0] << ", " << u_current[1] << ", error: " << error << std::endl;
        iter++;
    }
    return 0;
}

void Optimizer::integrate_next_states() {
    // Set the current state and control input for the simulation
    sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "x", x_current);
    sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "u", u_current);

    // Run the simulation
    int status_s = mobile_robot_acados_sim_solve(sim_capsule);
    if (status_s != ACADOS_SUCCESS) {
        throw std::runtime_error("acados integrator returned status " + std::to_string(status_s) + ". Exiting.");
    }

    // Get the result and update the current state
    sim_out_get(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_out, "x", x_current);

    t0 += T;
}

int Optimizer::find_next_waypoint() {
    // auto start = std::chrono::high_resolution_clock::now();

    // Calculate distances to each waypoint
    // current_state << x_current[0], x_current[1], x_current[2];
    // distances = Eigen::VectorXd::Zero(state_refs.rows());
    // for (int i = 0; i < state_refs.rows(); ++i) {
    //     distances(i) = (Eigen::Vector2d(state_refs(i,0), state_refs(i,1)) - current_state.head(2)).norm();
    // }
    // // Find the index of the closest waypoint
    // Eigen::VectorXd::Index closest_idx;
    // double min_distance_sq = pow(distances.minCoeff(&closest_idx), 2);
    
    // Update current state only once
    current_state << x_current[0], x_current[1] , x_current[2];

    // Precompute squared norm for efficiency
    double current_norm = current_state.head(2).squaredNorm();

    // Initialize variables for finding the minimum distance
    Eigen::VectorXd::Index closest_idx;
    double min_distance_sq = std::numeric_limits<double>::max();

    static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]

    int min_index = std::max(last_waypoint_index - limit, 0); //0;
    int max_index = std::min(last_waypoint_index + limit, static_cast<int>(state_refs.rows()) - 1); //state_refs.rows() - 1;
    // int min_index = 0;
    // int max_index = state_refs.rows() - 1;
    for (int i = min_index; i < max_index; ++i) {
        double distance_sq = (state_refs.row(i).head(2).squaredNorm() 
                           - 2 * state_refs.row(i).head(2).dot(current_state.head(2))
                           + current_norm); 

        if (distance_sq < min_distance_sq) {
            min_distance_sq = distance_sq;
            closest_idx = i;
        }
    }

    //1
    // last_waypoint_index = target_waypoint_index;
    // target_waypoint_index = std::max(last_waypoint_index, static_cast<int>(closest_idx));
    // return std::min(target_waypoint_index, static_cast<int>(state_refs.rows()) - 1);

    //2
    last_waypoint_index = target_waypoint_index;
    if (min_distance_sq < region_of_acceptance*region_of_acceptance) {
        std::cout << "min_distance_sq: " << min_distance_sq << std::endl;
        target_waypoint_index ++;
    }
    if (closest_idx > target_waypoint_index) {
        target_waypoint_index = closest_idx;
    }
    double dist = sqrt(min_distance_sq);
    // std::cout << "cur:" << current_state[0] << "," << current_state[1] << ", closest_idx:" << closest_idx << ", closest:" << state_refs(closest_idx, 0) << "," << state_refs(closest_idx, 1) << ", dist: " << dist <<  ", last:" << last_waypoint_index << ", target:" << target_waypoint_index << ", u:" << u_current[0] << ", " << u_current[1] << std::endl;
    std::cout << "closest_idx:" << closest_idx <<  ", last:" << last_waypoint_index << ", target:" << target_waypoint_index << ", u:" << u_current[0] << ", " << u_current[1] << std::endl;
    return std::min(target_waypoint_index, static_cast<int>(state_refs.rows()) - 1);
    // Determine the next waypoint
    // if (min_distance_sq < region_of_acceptance*region_of_acceptance) {
    //     if (closest_idx - last_waypoint_index < 15) {
    //         last_waypoint_index = std::max(last_waypoint_index, static_cast<int>(closest_idx) + 1);
    //     } else {
    //         closest_idx = last_waypoint_index;
    //     }
    // } else {
    //     if (closest_idx > last_waypoint_index + 15) {
    //         // closest_idx = last_waypoint_index + 1;
    //         closest_idx = last_waypoint_index;
    //     }
    //     last_waypoint_index++;
    // }

    // // print current state, closest point, closest index, target index
    // int target_idx = std::max(last_waypoint_index, static_cast<int>(closest_idx));
    // double dist = sqrt(min_distance_sq);
    // // std::cout << "dist: " << dist<< ", cur:" << current_state[0] << "," << current_state[1] << "," << current_state[2] << ", closest:" << state_refs(closest_idx, 0) << "," << state_refs(closest_idx, 1) << "," << state_refs(closest_idx, 2) << ", closest_idx:" << closest_idx << ", target_idx:" << target_waypoint_index << std::endl;
    // std::cout << "cur:" << current_state[0] << "," << current_state[1] << "," << current_state[2] << ", closest_idx:" << closest_idx << ", closest:" << state_refs(closest_idx, 0) << "," << state_refs(closest_idx, 1) << "," << state_refs(closest_idx, 2) << ", dist: " << dist<<  ", last_waypoint_index:" << last_waypoint_index << ", target_idx:" << target_waypoint_index << std::endl;
    // // auto finish = std::chrono::high_resolution_clock::now();
    // // std::chrono::duration<double> elapsed = finish - start;
    // // std::cout << "find_next_waypoint() took " << elapsed.count() << " seconds" << std::endl;
    // return std::min(target_idx, static_cast<int>(state_refs.rows()) - 1);
}
void Optimizer::update_current_states(double x, double y, double yaw) {
    if(target_waypoint_index < state_refs.rows()) {
        double ref_yaw = state_refs(target_waypoint_index, 2);
        while (ref_yaw - yaw > M_PI) {
            yaw += 2 * M_PI;
        }
        while (ref_yaw - yaw < -M_PI) {
            yaw -= 2 * M_PI;
        }
    }
    x_current[0] = x;
    x_current[1] = y;
    x_current[2] = yaw;
}
void Optimizer::update_real_states(double x, double y, double yaw) {
    if(target_waypoint_index < state_refs.rows()) {
        double ref_yaw = state_refs(target_waypoint_index, 2);
        while (ref_yaw - yaw > M_PI) {
            yaw += 2 * M_PI;
        }
        while (ref_yaw - yaw < -M_PI) {
            yaw -= 2 * M_PI;
        }
    }
    x_current[0] = x;
    x_current[1] = y;
    x_current[2] = yaw;
}
Eigen::VectorXd Optimizer::computeStats(int hsy) {
    simX.conservativeResize(iter, Eigen::NoChange);
    simU.conservativeResize(iter, Eigen::NoChange);
    time_record.conservativeResize(iter, Eigen::NoChange);
    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());

    // Calculate averages for speed and steer
    double average_speed = simU.col(0).mean();
    double average_steer = simU.col(1).mean();

    // Calculate differences for speed and steer
    Eigen::MatrixXd deltaU = simU.bottomRows(simU.rows() - 1) - simU.topRows(simU.rows() - 1);
    double average_delta_speed = deltaU.col(0).cwiseAbs().mean();
    double average_delta_steer = deltaU.col(1).cwiseAbs().mean();

    // Output results
    std::cout << "Average speed: " << average_speed << " m/s\n";
    std::cout << "Average steer angle: " << average_steer << " rad\n";
    std::cout << "Average change in speed: " << average_delta_speed << " m/s²\n";
    std::cout << "Average change in steer angle: " << average_delta_steer << " rad/s\n";

    // Calculate average errors
    double average_x_error = x_errors.cwiseAbs().mean();
    double average_y_error = y_errors.cwiseAbs().mean();
    yaw_errors = (yaw_errors.array().sin().binaryExpr(yaw_errors.array().cos(), std::ptr_fun(atan2))).matrix();
    double average_yaw_error = yaw_errors.cwiseAbs().mean();

    // Output error results
    std::cout << "Average x error: " << average_x_error << " m\n";
    std::cout << "Average y error: " << average_y_error << " m\n";
    std::cout << "Average yaw error: " << average_yaw_error << " rad\n";

    // Return the statistics as a vector
    Eigen::VectorXd stats(7);
    stats << average_speed, average_steer, average_delta_speed, average_delta_steer,
                average_x_error, average_y_error, average_yaw_error;
    
    // Eigen::VectorXd stats = computeStats();
    saveToFile(simX, "simX.txt");
    saveToFile(simU, "simU.txt");
    // saveToFile(time_record, "time_record.txt"); 
    saveToFile(stats, "stats.txt");
    return stats;
}

// Helper functions
std::string Optimizer::getSourceDirectory() {
    std::string file_path(__FILE__);  // __FILE__ is the full path of the source file
    size_t last_dir_sep = file_path.rfind('/');  // For Unix/Linux path
    if (last_dir_sep == std::string::npos) {
        last_dir_sep = file_path.rfind('\\');  // For Windows path
    }
    if (last_dir_sep != std::string::npos) {
        return file_path.substr(0, last_dir_sep);  // Extract directory path
    }
    return "";  // Return empty string if path not found
}
template <typename EigenType>
void Optimizer::saveToFile(const EigenType &data, const std::string &filename) {
    std::string dir = getSourceDirectory();
    std::string file_path = dir + "/" + filename;
    std::ofstream file(file_path);
    if (file.is_open()) {
        file << data << "\n";
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    file.close();
    std::cout << "Saved to " << file_path << std::endl;
}
Eigen::MatrixXd Optimizer::loadTxt(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }

    std::string line;
    std::vector<double> matrixEntries;
    int numRows = 0;
    int numCols = -1;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double num;
        std::vector<double> lineEntries;

        while (iss >> num) {
            lineEntries.push_back(num);
        }

        if (numCols == -1) {
            numCols = lineEntries.size();
        } else if (lineEntries.size() != numCols) {
            throw std::runtime_error("Inconsistent number of columns");
        }

        matrixEntries.insert(matrixEntries.end(), lineEntries.begin(), lineEntries.end());
        numRows++;
    }

    // Use Eigen::Map with row-major layout
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), numRows, numCols);
}
// int main() {
//     Optimizer controller;
//     controller.run();
//     return 0;
// }
