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

### ROS
#### Installation:
http://www.autolabor.com.cn/book/ROSTutorials/chapter1/12-roskai-fa-gong-ju-an-zhuang/124-an-zhuang-ros.html

### robot_localization
#### Installation:

1. Install it as follows:
    ```bash
    sudo apt update
    sudo apt install ros-noetic-robot-localization
    ```

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

3. Update the Makefile:
    ```bash
    # Set the following in <acados_root_folder>/Makefile.rule
    BLASFEO_TARGET = ARMV8A_ARM_CORTEX_A57
    ACADOS_WITH_QPOASES = 1
    ```

4. Build and install Acados:
    ```bash
    make -j4
    sudo make install -j4
    cd ..
    make shared_library
    ```

5. Install Python interface for Acados:
    ```bash
    pip3 install -e /home/{your user name}/acados/interfaces/acados_template
    ```

6. Update your `.bashrc` and source it:
    ```bash
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/{your user name}/acados/lib"' >> ~/.bashrc
    echo 'export ACADOS_SOURCE_DIR="/home/{your user name}/acados"' >> ~/.bashrc
    source ~/.bashrc
    ```

7. If you encounter issues with `t_renderer`, compile it natively:
    ```bash
    git clone https://github.com/acados/tera_renderer.git
    cd tera_renderer
    cargo build --verbose --release
    # Replace the file <acados_root_dir>/bin/t_renderer with the one compiled natively i.e. <tera_renderer_dir>/target/release/t_renderer
    ```

### Other Stuff
    ```bash
    sudo apt install nlohmann-json3-dev
    ```

## Running the Example

### Acados Simulation

1. Run the following command to execute the MPC simulation:
    ```bash
    python3 scripts/mpc_acados.py
    ```

2. Upon successful execution, a GIF illustrating the simulation will be saved in `scripts/gifs_acados`.

### Gazebo Simulation

#### In Python
##### Path Following
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map_with_car.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the control node:
    ```bash
    rosrun mpc control_node2.py --useEkf 
    ```

##### Follow Traffic Rules
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map_with_some_objects.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the object detctor node:
    ```bash
    roslaunch control signFastest.launch show:=true print:=true
    ```
3. Start the control node:
    ```bash
    rosrun mpc control_node2.py --useEkf --sign
    ```

#### In C++
##### Path Following
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map2023.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the control node:
    ```bash
    roslaunch mpc controller.launch ekf:=true
    ```
4. If ekf set to false, we use the ground truth from gazebo to get the vehicle pose. In this case, no need to launch the ekf node.

##### Follow Traffic Rules
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map2023objects.launch
    ```

2. Start the ekf node:
    ```bash
    roslaunch mpc ekf.launch
    ```
3. Start the object detctor node:
    ```bash
    roslaunch control signFastest.launch show:=true print:=true
    ```
3. Start the control node:
    ```bash
    roslaunch mpc controller.launch ekf:=true sign:=true
    ```

## More Details for Matthew and alex

the python version is now obsolete since we've migrated everthing to c++. 
all new features will be implemented in c++, so please start to get familiar with it
the only python files that are still useful are:
- global_planner2.py: generates a path from a starting location to destination in the new (2024) map and returns a list of waypoints, which is further processed in path.py. the different paths can be found in scripts/config/paths.yaml. for testing, you can define more paths inside the yaml file and change line 56 in mpc_acados2.py. I have defined several places by names inside global_planner2.py. feel free to add more as you see fit. you can refer to the map definition in scripts/maps/Competition_track_graph.png.
- path2.py: takes the waypoints generated from global planner for further processing. specifically, it uses spline interpolation to create a smooth path from those points and then generate waypoints of a desired density and of (almost) equal deistances between each other. furthermore, it assigns an attribute to each waypoint:
attributes = ["normal", "crosswalk", "intersection", "oneway", "highwayLeft", "highwayRight", "roundabout", "stopline", "dotted", "dotted_crosswalk"]. the density of the waypoints vary depending on the value of their attributes. for example crosswalk regions have a higher density (so they go slower) and highway regions the opposite. this way, we won't have to rely on object detection to figure out where the highways and crosswalks are.
- mpc_acados2.py: run this file to generate the c code needed to run the mpc algorithms, which the c++ code needs. the code will be generated in scripts/c_generated_code, and you will have to move it to src/c_generated_code (replace the one that's already there). the configurations (constraints and costs) of the mpc model are stored in scripts/config/mpc_config2.yaml. for testing, you will have to tweak values inside this file, and every time you do this, you will have to renegerate the c code and paste it inside src.

now onto the c++ files.

- optimizer.cpp:
this file mirrors control_node2.py, which i explained to you already. it contains a class optimizer which contains the waypoints (references states) and has functions that the controller node cna call to get the desired control inputs (steer & speed) at given states. almost everything is a direct translation of its python counterpart. I guess it's worth mentioning that in c++ we use the eigen library and (sometimes) vectors and arrays from the STL library. eigen is very similar to numpy, which you should be familiar with by now, but there are some key differences. i wont delve into them here, but you should be able to get used to it quickly as you start working with the code. overall c++ is pretty similar to java except for having to deal with reference/value passing and memory explicitly.

-utility.cpp:
same as utility.py, with some additional features that i added during the winter break. most noteworthy one is the addition of a container to store the positions of each unique car detected, which is useful during the parking maneuver to determine which spots are empty.

-controller.cpp:
almost the same as controller_node2.py. has the implementation of the state machine class and the main function in which we init a ros node, creates an instance of the statemachine class and runs the state machine. the state machine itself creates an instance of optimizer and utility and calls the functions that it needs when it needs them. i set everything in utility and optimizer to public so we wont have to bother with getters and setters.

## Your tasks

your task will be to do exhaustive testing within the gazebo environment of the current state machine (relying on localization, no lane detection). this includes:
- generating different paths, enough to cover every possible street and corners on the map, save them in paths.yaml and test them sequentially.
- chnage the positions of (especially) the vehicles, the lights and signs on the map. be creative and come up with complex configurations and test them.
- tweak the values of the mpc model. specifically for now, change the speed to 0.5, 0.35 and 0.2 and test each of them.
- generate a testing document like in 211. report any bugs and bad behaviors and discuss them with me. later, when you get more familiar with the code, you will try to fix them by yourself.
first i'd suggest getting at least somewhat familiar with c++, namely, the syntax, pointers, passing by ref vs by value, STL containers, the eigen library, and ros in c++.
i dont have time to document everything so just read through the code and try your best to understand it. 
as i said, except for some additional features that i implemented during the break, everything is a direct translation of the python code.
i think a week should be good for this.
once you feel ready, follow the instructions above to run the simulation and the controller, and start testing and have fun!
not everything in my implementation is clean and optimal, as my focus was turning my ideas into code and making them work first. if you have different ideas about implementation or approach please share them and we can discuss together.
if there are parts you can't wrap your head around or you'd like some clarifications on, we can talk about it during the next worksession, but i wont re-explain everything that i did last time, so please do your best to understand it first and come prepared with specific questions. also if you are stuck on something please let me know and dont waste 2 hours trying to figure it out on your own. 
Good luck




