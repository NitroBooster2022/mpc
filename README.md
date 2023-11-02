# MPC

Model Predictive Control development for BFMC.

## Description

This project utilizes Gazebo model information alongside an Extended Kalman Filter (EKF) from the ROS package robot_localization. It takes a predetermined waypoint list as input, planning and simulating the input commands required to traverse the path as quickly and accurately as possible. Simulations can be conducted either within Gazebo or independently.

## Structure

- **config**: Contains EKF parameters and instructions on how to reset them during simulation.
- **include**: Houses packages and functions required by MPC.
- **launch**: Contains launch files for MPC.
- **scripts** and **src**: Programs necessary to run MPC and display simulation results.

## Dependencies

### Acados

#### Installation:

1. Clone the Acados repository and navigate into it:
    ```bash
    git clone https://github.com/acados/acados.git
    cd acados
    git submodule update --recursive --init
    ```

2. Create a build directory, navigate into it and run cmake:
    ```bash
    mkdir -p build
    cd build
    cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57
    # Note: Replace ARMV8A_ARM_CORTEX_A57 with your device's architecture or use GENERIC if unsure.
    ```

3. Build and install Acados:
    ```bash
    make -j4
    make install -j4
    ```

4. Update the Makefile:
    ```bash
    # Set the following in <acados_root_folder>/Makefile.rule
    BLASFEO_TARGET = ARMV8A_ARM_CORTEX_A57
    ACADOS_WITH_QPOASES = 1
    make shared_library
    ```

5. Install Python interface for Acados:
    ```bash
    pip3 install -e /home/{your user name}/acados/interfaces/acados_template
    ```

6. Update your `.bashrc` and source it:
    ```bash
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"home/{your user name}/acados/lib"' >> ~/.bashrc
    source ~/.bashrc
    ```

7. If you encounter issues with `t_renderer`, compile it natively:
    ```bash
    git clone https://github.com/acados/tera_renderer.git
    cd tera_renderer
    cargo build --verbose --release
    # Replace the file <acados_root_dir>/bin/t_renderer with the one compiled natively i.e. <tera_renderer_dir>/target/release/t_renderer
    ```
