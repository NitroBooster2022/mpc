# mpc

## Model Predictive Control development for BFMC

It uses Gazebo model information and an EKF from the ROS package robot_localization and a predeterminate waypoint list as input and plans and simulates the input commands to complete the path as quickly and accurately as possible. (The simulation can be done in Gazebo or without it)

## Structure

- config: EKF parameters and how to clear them in simulation

- include: Packges and function required by MPC

- launch: Launch files for MPC

- scripts and src: Programs to run MPC and display simulation results.

## Dependencies

1) Acados

Installation:

git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

mkdir -p build
cd build
cmake .. -DACADOS_WITH_QPOASES=ON -DACADOS_EXAMPLES=ON -DHPIPM_TARGET=GENERIC -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57
Note: ARMV8A_ARM_CORTEX_A57 is for jetsons. Replace with your device's architecture or use GENERIC if you're not sure.
make -j4
make install -j4

Set the BLASFEO_TARGET in <acados_root_folder>/Makefile.rule to ARMV8A_ARM_CORTEX_A57.
also, set ACADOS_WITH_QPOASES to 1.
make shared_library

pip3 install -e /home/{your user name}/acados/interfaces/acados_template
add these to .bashrc then source: 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"home/{your user name}/acados/lib"

if you encounter a problem with t_renderer, you need to compile it natively:
git clone https://github.com/acados/tera_renderer.git
cd into it.
cargo build --verbose --release
replace the file <acados_root_dir>/bin/t_renderer with the one you compiled natively, i.e. in <tera_renderer_dir>/target/release/t_renderer