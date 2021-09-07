cmake_minimum_required(VERSION 3.20)
project(mav_trajectory_generation)


find_package(catkin REQUIRED COMPONENTS glog_catkin cmake_modules mav_msgs)
catkin_package(
  INCLUDE_DIRS include/
  LIBRARIES ${PROJECT_NAME} nlopt
  CATKIN_DEPENDS cmake_modules mav_msgs glog_catkin)
include_directories(include ${catkin_INCLUDE_DIRS})
add_subdirectory(nlopt)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  # src/trajectory_sampling.cpp
  src/vertex.cpp
  # src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_subdirectory(pybind11)

pybind11_add_module(mav_traj_gen src/python.cpp)
target_compile_features(mav_traj_gen PUBLIC cxx_std_17)
target_link_libraries(mav_traj_gen PUBLIC ${PROJECT_NAME} nlopt)

############
# BINARIES #
############
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})

##########
# EXPORT #
##########
# cs_install()
# cs_export()
