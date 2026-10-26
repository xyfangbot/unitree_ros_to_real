cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")
 
add_executable(example_walk_aliengo examples/example_walk_aliengo_sport.cpp)
target_link_libraries(example_walk_aliengo ${EXTRA_LIBS})

add_executable(example_start_aliengo examples/example_start_aliengo_sport.cpp)
target_link_libraries(example_start_aliengo ${EXTRA_LIBS})

add_executable(example_end_aliengo examples/example_end_aliengo_sport.cpp)
target_link_libraries(example_end_aliengo ${EXTRA_LIBS})

add_executable(example_joystick_aliengo examples/example_joystick_aliengo_sport.cpp)
target_link_libraries(example_joystick_aliengo ${EXTRA_LIBS})

add_executable(example_recovery_aliengo examples/example_recovery_aliengo_sport.cpp)
target_link_libraries(example_recovery_aliengo ${EXTRA_LIBS})

add_executable(my_example_walk_aliengo examples/my_example_walk_aliengo_sport.cpp)
target_link_libraries(my_example_walk_aliengo ${EXTRA_LIBS})

add_executable(read_state examples/read_state.cpp)
target_link_libraries(read_state ${EXTRA_LIBS})