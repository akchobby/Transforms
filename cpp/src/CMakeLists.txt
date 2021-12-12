# Author: Anil Kumar Chavali

# add any system dependencies using find command
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR}/src/include)

# creating shared library for release, need to be further refined 

add_library(${PROJECT_NAME}_quaternion SHARED quaternion.cpp)
target_link_libraries(${PROJECT_NAME}_quaternion Eigen3::Eigen)

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/build/release)

install(TARGETS ${PROJECT_NAME}_quaternion DESTINATION ${PROJECT_SOURCE_DIR}/build/release)
install(FILES ${PROJECT_SOURCE_DIR}/src/include/quaternion.hpp DESTINATION ${PROJECT_SOURCE_DIR}/build/release)
