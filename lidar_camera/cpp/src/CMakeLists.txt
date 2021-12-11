# add any system dependencies using find command
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${TRANSFORMS_SOURCE_DIR}/src/include)

# Tests

add_library(${PROJECT_NAME}_quaternion quaternion.cpp)
target_link_libraries(${PROJECT_NAME}_quaternion Eigen3::Eigen)
