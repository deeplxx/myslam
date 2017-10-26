cmake_minimum_required(VERSION 2.8)
project(slam_study)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


##### DEPENDS #########

# opencv
find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# eigen
include_directories("/usr/include/eigen3")

# sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

## pcl
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# # ceres
# find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})

# g2o
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
	${Opencv_LIBS}
	${Sophus_LIBRARIES}
	g2o_core g2o_stuff g2o_types_sba)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
