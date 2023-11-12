#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <limits.h>

#include <Eigen/Dense>
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_mobile_robot.h"
#include "acados_solver_mobile_robot.h"

// Helper functions
std::string getCurrentWorkingDir() {
    char buff[PATH_MAX];
    getcwd(buff, PATH_MAX);
    std::string current_working_dir(buff);
    return current_working_dir;
}
std::string getSourceDirectory() {
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
void saveToFile(const Eigen::MatrixXd &matrix, const std::string &filename) {
    std::string dir = getSourceDirectory();
    std::string file_path = dir + "/" + filename;
    std::ofstream file(file_path);
    if (file.is_open()) {
        file << matrix << "\n";
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    file.close();
    std::cout << "Saved to << " << file_path << std::endl;
}
void saveToFile(const Eigen::VectorXd &vector, const std::string &filename) {
    std::string dir = getSourceDirectory();
    std::string file_path = dir + "/" + filename;
    std::ofstream file(file_path);
    if (file.is_open()) {
        file << vector << "\n";
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    file.close();
    std::cout << "Saved to << " << file_path << std::endl;
}

class MobileRobotController {
private:
    int status; // acados operation state
    int idxbx0[3];
    double min_time;
    double elapsed_time;
    double x_target[3];
    double x_state[5];
    double x_current[3];
    double u_current[3];
    int N;
    int nx;
    int nu;

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

    Eigen::MatrixXd simX;
    Eigen::MatrixXd simU;
    Eigen::VectorXd time_record;

    // Private methods for initialization and simulation
    void initialize();
    void performSimulation();

public:
    MobileRobotController();
    int run(); // Replaces the main function
};

MobileRobotController::MobileRobotController() {
    // Initialize member variables
    min_time = 1e12;
    status = 0; // Assuming 0 is a default 'no error' state
    N = 0;
    nx = 0;
    nu = 0;

    // Initialize arrays to default values
    for (int i = 0; i < 3; i++) {
        x_target[i] = 0.0;
        x_current[i] = 0.0;
        u_current[i] = 0.0;
    }

    for (int i = 0; i < 5; i++) {
        x_state[i] = 0.0;
    }

    // Setting up other necessary members
    acados_ocp_capsule = nullptr;
    sim_capsule = nullptr;
    mobile_robot_sim_config = nullptr;
    mobile_robot_sim_dims = nullptr;
    mobile_robot_sim_in = nullptr;
    mobile_robot_sim_out = nullptr;
    nlp_config = nullptr;
    nlp_dims = nullptr;
    nlp_in = nullptr;
    nlp_out = nullptr;

    // Eigen matrices and vectors initialization
    simX = Eigen::MatrixXd::Zero(1, 1); // Placeholder dimensions
    simU = Eigen::MatrixXd::Zero(1, 1); // Placeholder dimensions
    time_record = Eigen::VectorXd::Zero(1); // Placeholder dimension
}

void MobileRobotController::initialize() {
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

    // Resizing Eigen matrices and vectors based on problem dimensions
    simX.resize(N + 1, nx);
    simU.resize(N, nu);
    time_record.resize(N);

    // Initialize target, current state and state variables
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;

    x_target[0] = 2.0;
    x_target[1] = 2.0;
    x_target[2] = 1.0;

    x_state[0] = 2.0;
    x_state[1] = 2.0;
    x_state[2] = 1.0;
    x_state[3] = 0.0;
    x_state[4] = 0.0;
}

void MobileRobotController::performSimulation() {
    // Set the reference for the final step in the horizon
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_target);

    // Initial state for the simulation
    for (int i = 0; i < nx; i++) {
        simX(0, i) = x_current[i];
    }

    // Set the reference for each step in the horizon
    for (int i = 0; i < N; i++) {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", x_state);
    }

    std::cout << "N = " << N << std::endl;
    // Closed loop simulation
    for (int ii = 0; ii < N; ii++) {
        auto t_start = std::chrono::high_resolution_clock::now();

        // Set current state constraints
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);

        // Solve the optimization problem
        status = mobile_robot_acados_solve(acados_ocp_capsule);
        if (status != ACADOS_SUCCESS) {
            printf("mobile_robot_acados_solve() failed with status %d.\n", status);
            return;
        }

        // Get the optimized control input
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
        for (int i = 0; i < nu; i++) {
            simU(ii, i) = u_current[i];
        }

        // Measure elapsed time
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        time_record(ii) = elapsed_time_ms;

        // Simulation step
        sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "u", u_current);
        sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "x", x_current);

        status = mobile_robot_acados_sim_solve(sim_capsule);
        if (status != ACADOS_SUCCESS) {
            printf("mobile_robot_acados_sim_solve() failed with status %d.\n", status);
            return;
        }

        // Update current state
        sim_out_get(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_out, "x", x_current);
        for (int i = 0; i < nx; i++) {
            simX(ii + 1, i) = x_current[i];
            // std::cout << "simX(" << ii + 1 << ", " << i << ") = " << simX(ii + 1, i) << std::endl;
        }
    }

    // Print results
    for (int i = 0; i < N + 1; i++) {
        printf("Final result index %d %f, %f, %f \n", i, simX(i, 0), simX(i, 1), simX(i, 2));
    }

    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());
}

int MobileRobotController::run() {
    this->initialize();
    this->performSimulation();
    saveToFile(simX, "simX.txt");
    saveToFile(simU, "simU.txt");
    saveToFile(time_record, "time_record.txt"); 
    return status;
}

int main() {
    MobileRobotController controller;
    return controller.run();
}
