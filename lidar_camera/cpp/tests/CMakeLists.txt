# Author: Anil Kumar Chavali

# inlclude src files
include_directories(${TRANSFORMS_SOURCE_DIR}/src
                    ${TRANSFORMS_SOURCE_DIR}/src/include)

# Tests
add_executable(${PROJECT_NAME}_quaternion_test quaternion_main.cpp)

# link dependencies

target_link_libraries(${PROJECT_NAME}_quaternion_test ${PROJECT_NAME}_quaternion)

# add the test files
add_test(NAME testQuaternion COMMAND ${PROJECT_NAME}_quaternion_test)
