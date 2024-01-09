cmake_minimum_required(VERSION 3.0.2)
project(mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set compiler standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -DEXT_DEP -fdiagnostics-show-option")
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park)
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)
file(GLOB ocp_solver
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/acados_solver_mobile_robot.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/acados_solver_park.c
        )

file(GLOB casadi_fun
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/mobile_robot_model/mobile_robot_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/mobile_robot_model/mobile_robot_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/mobile_robot_model/mobile_robot_expl_vde_adj.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/park_model/park_expl_ode_fun.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/park_model/park_expl_vde_forw.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/park_model/park_expl_vde_adj.c)

file(GLOB sim_solver
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code/acados_sim_solver_mobile_robot.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/c_generated_code_park/acados_sim_solver_park.c
        )

find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf
  message_generation
  utils
)


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime
#  LIBRARIES mpc
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} 
  include 
  # /usr/include
  # /usr/lib/gcc/x86_64-linux-gnu/9/include/
  # /usr/include/c++/9/
  ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src
)

# link_directories(
#   /usr/lib
#   /usr/local/lib 
# )
file(GLOB_RECURSE ALGLIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/include/alglib-cpp/src/*.cpp)
add_library(alglib SHARED ${ALGLIB_SOURCES})

add_library(ocp_shared_lib SHARED ${ocp_solver} ${casadi_fun} )
target_link_libraries(ocp_shared_lib acados hpipm blasfeo)

add_library(sim_shared_lib SHARED ${sim_solver} ${casadi_fun})
target_link_libraries(sim_shared_lib acados hpipm blasfeo)

add_executable(path_planner src/path_planner.cpp)
target_link_libraries(path_planner
alglib
)

add_executable(opt
src/optimizer_main.cpp
src/optimizer.cpp
)
target_link_libraries(opt 
ocp_shared_lib 
sim_shared_lib
alglib
${catkin_LIBRARIES}
)

add_executable(utility 
src/utility_main.cpp
src/utility.cpp
)
target_link_libraries(utility
${catkin_LIBRARIES}
)

add_executable(controller 
src/controller.cpp
src/optimizer.cpp
src/utility.cpp
)
target_link_libraries(controller
alglib
ocp_shared_lib 
sim_shared_lib
${catkin_LIBRARIES}
nlohmann_json::nlohmann_json
)

add_executable(bridge1
  src/bridge.cpp
)
target_link_libraries(bridge1
  ${catkin_LIBRARIES}
)
