cmake_minimum_required(VERSION 3.0.2)
project(pgo_toy_example)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/thirdparty/g2o/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
  )

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)

find_package(g2o REQUIRED)

catkin_package(
  )

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

set(G2O_LIBS
	g2o_core 
	g2o_types_sba 
	g2o_types_slam3d
	g2o_solver_csparse 
	g2o_stuff 
	g2o_csparse_extension
	)

#------------------------------------------
add_executable(pgo_toy_example
  src/pgo_toy_example_node.cpp
  src/pgo_toy_example.cpp
  )
target_link_libraries(pgo_toy_example
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
  )

