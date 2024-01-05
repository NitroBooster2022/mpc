#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <limits.h>
#include <cmath>
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_mobile_robot.h"
#include "acados_solver_mobile_robot.h"
#include "acados_solver_park.h"
#include "acados_sim_solver_park.h"

class Optimizer {
public:
    Optimizer(double T, int N, double v_ref, double x_init, double y_init, double yaw_init);
    Optimizer(double T, int N, double v_ref): Optimizer(T, N, v_ref, 0.83, 0.33, 1.65756) {}
    Optimizer(): Optimizer(0.125, 40, 1.0, 0.83, 0.33, 1.65756) {}
    ~Optimizer() {
        free(acados_ocp_capsule);
        free(sim_capsule);
        free(mobile_robot_sim_config);
        free(mobile_robot_sim_dims);
        free(mobile_robot_sim_in);
        free(mobile_robot_sim_out);
        free(nlp_config);
        free(nlp_dims);
        free(nlp_in);
        free(nlp_out);

        free(acados_ocp_capsule_park);
        free(sim_capsule_park);
        free(park_sim_config);
        free(park_sim_dims);
        free(park_sim_in);
        free(park_sim_out);
        free(nlp_config_park);
        free(nlp_dims_park);
        free(nlp_in_park);
        free(nlp_out_park);
    }
    int run(); 
    int update_and_solve();
    void integrate_next_states();
    int find_next_waypoint(int min_index = -1, int max_index = -1);
    void update_current_states(double x, double y, double yaw, Eigen::Vector3d& state);
    void update_current_states(double x, double y, double yaw) {
        update_current_states(x, y, yaw, x_current);
    }
    // void update_current_states(double* state);
    Eigen::VectorXd computeStats(int hsy);
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
    // double x_current[3];
    Eigen::Vector3d x_current;
    Eigen::Vector3d x_real;
    double u_current[2];
    int N, nx, nu, iter = 0;
    int N_park, nx_park, nu_park;
    double T_park;
    int target_waypoint_index, last_waypoint_index, closest_waypoint_index, num_waypoints;
    double region_of_acceptance, region_of_acceptance_cw, region_of_acceptance_hw, v_ref, t0, T, density, rdb_circumference = 4.15;
    bool debug = true;
    Eigen::Vector3d current_state;
    Eigen::MatrixXd state_refs, input_refs, normals;
    Eigen::VectorXd state_attributes;
    enum ATTRIBUTE {
        NORMAL, CROSSWALK, INTERSECTION, ONEWAY, HIGHWAYLEFT, HIGHWAYRIGHT, ROUNDABOUT, STOPLINE, DOTTED, DOTTED_CROSSWALK
    };
    Eigen::MatrixXd *state_refs_ptr;
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

    park_solver_capsule *acados_ocp_capsule_park;
    park_sim_solver_capsule *sim_capsule_park;
    sim_config *park_sim_config;
    void *park_sim_dims;
    sim_in *park_sim_in;
    sim_out *park_sim_out;
    ocp_nlp_config *nlp_config_park;
    ocp_nlp_dims *nlp_dims_park;
    ocp_nlp_in *nlp_in_park;
    ocp_nlp_out *nlp_out_park;

    Eigen::MatrixXd simX, simU;
    Eigen::VectorXd time_record, x_errors, y_errors, yaw_errors;

    Eigen::Matrix2d rotation_matrix;
    Eigen::Vector2d rotated_xy;
    void transform_point(Eigen::Vector3d& pt, Eigen::Vector3d& pt_transformed,
                         const Eigen::Vector3d& frame1, 
                         const Eigen::Vector3d& frame2) {
        // Unpack the frames and the point
        const double &x1 = frame1[0], &y1 = frame1[1], &theta1 = frame1[2];
        const double &x2 = frame2[0], &y2 = frame2[1], &theta2 = frame2[2];
        double &x = pt[0], &y = pt[1], &psi = pt[2];
        double &x_transformed = pt_transformed[0], &y_transformed = pt_transformed[1], &psi_transformed = pt_transformed[2];

        // Step 1: Translate to the origin of frame1
        x_transformed = x - x1;
        y_transformed = y - y1;

        // Step 2: Rotate to align frame1 with frame2
        double rotation_angle = theta2 - theta1;
        
        rotation_matrix << cos(rotation_angle), -sin(rotation_angle),
                           sin(rotation_angle),  cos(rotation_angle);
        rotated_xy = rotation_matrix * pt_transformed.head(2);

        // Update psi (yaw) and normalize
        psi_transformed = std::fmod(psi + rotation_angle, 2 * M_PI);

        // Step 3: Translate to the origin of frame2
        x_transformed = rotated_xy[0] + x2;
        y_transformed = rotated_xy[1] + y2;
    }
    void set_up_park(Eigen::VectorXd& xs) {
        int reset_status = park_acados_reset(acados_ocp_capsule_park, 1);
        ocp_nlp_cost_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, N_park, "yref", xs.data());
        for (int j = 0; j < N; ++j) {
            ocp_nlp_cost_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, j, "yref", xs.data());
        }
        frame1.head(2) = x_current.head(2);
        frame1[2] = NearestDirection(x_current[2]);
        std::cout << "yaw: " << x_current[2] << ", nearest: " << frame1[2] << std::endl;
        frame2 << 0.0, 0.0, M_PI;
        frame2 << 0.0, 0.0, 0.0;
        std::cout << "frame1: " << frame1[0] << ", " << frame1[1] << ", " << frame1[2] << std::endl;
        std::cout << "frame2: " << frame2[0] << ", " << frame2[1] << ", " << frame2[2] << std::endl;
    }
    Eigen::Vector3d frame2;
    Eigen::Vector3d frame1;
    Eigen::Vector3d x_current_transformed;
    int update_and_solve_park(Eigen::VectorXd& xs, double thresh_sq = 0.0025) {
        auto t_start = std::chrono::high_resolution_clock::now();
        
        transform_point(x_current, x_current_transformed, frame1, frame2);
        double yaw_frame2 = frame2[2];
        while (x_current_transformed[2] - yaw_frame2 > M_PI) {
            x_current_transformed[2] -= 2 * M_PI;
        }
        while (x_current_transformed[2] - yaw_frame2 < -M_PI) {
            x_current_transformed[2] += 2 * M_PI;
        }
        double error_sq = (x_current_transformed - xs.head(3)).squaredNorm();
        static int count = 0;
        // std::cout << count << ")error: "<< sqrt(error_sq) <<", x_cur: " << x_current[0] << ", " << x_current[1] << ", " << x_current[2] << ", x_cur_trans: " << x_current_transformed[0] << ", " << x_current_transformed[1] << ", " << x_current_transformed[2] << ", xs: " << xs[0] << ", " << xs[1] << ", " << xs[2] << ", u: " << u_current[0] << ", " << u_current[1] << std::endl;
        count ++;
        if (error_sq < thresh_sq) {
            count = 0;
            printf("threshold reached, error: %4f\n", sqrt(error_sq));
            return 2;
        }
        ocp_nlp_constraints_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, 0, "lbx", x_current_transformed.data());
        ocp_nlp_constraints_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, 0, "ubx", x_current_transformed.data());

        // Solve the optimization problem
        status = park_acados_solve(acados_ocp_capsule_park);
        if (status != 0) {
            std::cout << "ERROR!!! acados acados_ocp_solver returned status " << status << ". Exiting." << std::endl;
            return 1; 
        }

        // Get the optimal control for the next step
        ocp_nlp_out_get(nlp_config_park, nlp_dims_park, nlp_out_park, 0, "u", &u_current);
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

    double NearestDirection(double yaw) {
        while(yaw > 2 * M_PI) {
            yaw -= 2 * M_PI;
        }
        while(yaw < 0) {
            yaw += 2 * M_PI;
        }

        static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};

        double minDifference = std::abs(yaw - directions[0]);
        double nearestDirection = directions[0];

        for (int i = 1; i < 5; ++i) {
            double difference = std::abs(yaw - directions[i]);
            if (difference < minDifference) {
                minDifference = difference;
                nearestDirection = directions[i];
            }
        }
        while (nearestDirection - yaw > M_PI) {
            nearestDirection -= 2 * M_PI;
        }
        while (nearestDirection - yaw < -M_PI) {
            nearestDirection += 2 * M_PI;
        }
        return nearestDirection;
    }

    int park() {
        std::cout.precision(3);
        //print T, N, nx, nu
        std::cout << "T: " << T_park << ", N: " << N_park << ", nx: " << nx_park << ", nu: " << nu_park << std::endl;
        Eigen::VectorXd xs(5);
        // xs << -1.31, 0, M_PI, 0, 0;
        xs << 0.63, 0.32, M_PI, 0, 0;
        xs << -0.63, -0.32, 0.0, 0, 0;
        set_up_park(xs);
        int i = 0;
        while(1) {
            status = update_and_solve_park(xs);
            if (status == 2 || i>100) {
                break;
            }
            x_current[0] += 0.05 * u_current[0] * cos(x_current[2]); // v * cos(psi) * dt
            x_current[1] += 0.05 * u_current[0] * sin(x_current[2]); // v * sin(psi) * dt
            x_current[2] += 0.05 * u_current[0]/0.27 * tan(u_current[1]); // v * tan(delta) * dt / L
            i++;
        }
        std::cout << "done1" << std::endl;
        xs << 0, 0, M_PI, 0, 0;
        xs << 0.63, 0.32, 0, 0, 0;
        set_up_park(xs);
        i = 0;
        while(1) {
            status = update_and_solve_park(xs);
            if (status == 2 || i>100) {
                break;
            }
            x_current[0] += 0.05 * u_current[0] * cos(x_current[2]); // v * cos(psi) * dt
            x_current[1] += 0.05 * u_current[0] * sin(x_current[2]); // v * sin(psi) * dt
            x_current[2] += 0.05 * u_current[0]/0.27 * tan(u_current[1]); // v * tan(delta) * dt / L
            i++;
        }
        std::cout << "done2" << std::endl;
        return 0;
    }

    void change_lane(int start_index, int end_index, bool shift_right = false, double shift_distance = 0.36-0.1) {
        if (shift_right) shift_distance *= -1;
        state_refs.block(start_index, 0, end_index-start_index, 2) += normals.block(start_index, 0, end_index-start_index, 2) * shift_distance;
        
        // Calculate the vectors for the gaps
        // Eigen::Vector2d start_gap_vector = state_refs.row(start_index) - state_refs.row(start_index - 1);
        // Eigen::Vector2d end_gap_vector = state_refs.row(end_index + 1) - state_refs.row(end_index);
        // double start_gap_yaw = std::atan2(start_gap_vector.y(), start_gap_vector.x());
        // double end_gap_yaw = std::atan2(end_gap_vector.y(), end_gap_vector.x());

        // // Calculate the number of points to interpolate based on the actual length of the gap
        // int num_start_points = static_cast<int>(density * start_gap_vector.norm() / 2);
        // int num_end_points = static_cast<int>(density * end_gap_vector.norm() / 2);

        // // Create matrices to hold the new interpolated waypoints for the start and end gaps
        // Eigen::MatrixXd start_gap_filler(num_start_points, 3);
        // Eigen::MatrixXd end_gap_filler(num_end_points, 3);

        // // Interpolate for the start gap
        // for (int i = 0; i < num_start_points; ++i) {
        //     double t = static_cast<double>(i + 1) / (num_start_points + 1);
        //     Eigen::Vector2d interpolated_position = state_refs.row(start_index - 1).head(2) * (1 - t) + 
        //                                             state_refs.row(start_index).head(2) * t;
        //     start_gap_filler.row(i).head(2) = interpolated_position;
        //     start_gap_filler(i, 2) = start_gap_yaw;
        // }
        // // Interpolate for the end gap
        // for (int i = 0; i < num_end_points; ++i) {
        //     double t = static_cast<double>(i + 1) / (num_end_points + 1);
        //     Eigen::Vector2d interpolated_position = state_refs.row(end_index).head(2) * (1 - t) + 
        //                                             state_refs.row(end_index + 1).head(2) * t;
        //     end_gap_filler.row(i).head(2) = interpolated_position;
        //     end_gap_filler(i, 2) = end_gap_yaw;
        // }
        // int original_size = state_refs.rows();
        // int new_size = original_size + start_gap_filler.rows() + end_gap_filler.rows();
        // state_refs.conservativeResize(new_size, Eigen::NoChange); 

        // // Shift the end part of the matrix to make space for the 'end_gap_filler'
        // state_refs.block(end_index + 1 + end_gap_filler.rows(), 0, original_size - end_index - 1, 3) =
        //     state_refs.block(end_index + 1, 0, original_size - end_index - 1, 3);

        // // Insert 'end_gap_filler' into 'state_refs'
        // state_refs.block(end_index + 1, 0, end_gap_filler.rows(), 3) = end_gap_filler;

        // // Shift the middle part of the matrix to make space for the 'start_gap_filler'
        // state_refs.block(start_index + start_gap_filler.rows(), 0, new_size - start_index - start_gap_filler.rows(), 3) =
        //     state_refs.block(start_index, 0, new_size - start_index - start_gap_filler.rows(), 3);

        // // Insert 'start_gap_filler' into 'state_refs'
        // state_refs.block(start_index, 0, start_gap_filler.rows(), 3) = start_gap_filler;
    }
    int get_current_attribute() {
        return state_attributes(target_waypoint_index);
    }
};

#endif // OPTIMIZER_HPP