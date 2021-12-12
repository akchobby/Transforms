cmake_minimum_required(VERSION 3.8)
project(transforms VERSION 1.0
	      DESCRIPTION "Library for transformation by Anil Kumar Chavali"
	      LANGUAGES CXX )

add_compile_options(-std=c++11)

enable_testing()
# add any system dependencies using find command
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# add sub directories

add_subdirectory(src)
add_subdirectory(tests)

# Defining variables
# set(SRC_LIST  src/quaternion.cpp)


