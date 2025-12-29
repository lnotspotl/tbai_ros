cmake_minimum_required(VERSION 3.19.0)
project(tbai CXX)

find_package(catkin REQUIRED)
find_package(tbai REQUIRED)
find_package(Eigen3 REQUIRED)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    # tbai libraries
    tbai_core
    tbai_bob
    # tbai_wtw
    tbai_np3o
    tbai_static
    tbai_estim
    tbai_deploy_go2
    tbai_deploy_go2w
    tbai_deploy_g1
    tbai_mpc

    # unitree sdk
    unitree_sdk2

    # ocs2 libraries
    ocs2_core
    ocs2_ddp
    ocs2_frank_wolfe
    ocs2_oc
    ocs2_robotic_tools
    ocs2_perceptive
    ocs2_qp_solver
    ocs2_mpc
    ocs2_sqp
    ocs2_slp
    ocs2_ipm
    # ocs2_ocs2
    ocs2_pinocchio_interface
    ocs2_centroidal_model
    # ocs2_blasfeo
    ocs2_hpipm
)
