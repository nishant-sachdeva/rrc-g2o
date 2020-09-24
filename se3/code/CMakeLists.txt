cmake_minimum_required(VERSION 2.8)
project(testSIM3)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O0 -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)
FIND_PACKAGE(G2O REQUIRED)

IF (G2O_FOUND)
Include_directories (${G2O_INCLUDE_DIR})
Message ("lib is found: G2O" ${G2O_INCLUDE_DIR})
ENDIF (G2O_FOUND)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  "/usr/include/eigen3"
  ${CSPARSE_INCLUDE_DIR}
  ${Cholmod_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  /usr/include/suitesparse
)

# note that as these libraries will be linked using the "ld" we have to 
# drop the "lib" prefix in front of these libraries. So actually, each 
# library has "lib" in front but we have dropped it. For e.g. "g2o_cli" is 
# in fact "libg2o_cli" but we drop the "lib"
LIST(APPEND G2O_LIBS
  cxsparse
  cholmod
  g2o_cli g2o_ext_freeglut_minimal g2o_solver_eigen
  g2o_core g2o_interface g2o_solver_csparse g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)


ADD_EXECUTABLE(optimizeSE3 optimizeSE3.cpp Priors.hpp)

TARGET_LINK_LIBRARIES(optimizeSE3 ${G2O_LIBS})



#LIST(APPEND G2O_LIBS
#  cxsparse
#  cholmod
#  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
#  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_solver_eigen
#  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
#  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
#  g2o_opengl_helper g2o_solver_dense g2o_stuff
#  g2o_types_sclam2d g2o_parser g2o_solver_pcg
#  g2o_types_data g2o_types_sim3
#)

