cmake_minimum_required(VERSION 2.8)
project(Sonar)
SET(COMPILE_DEFINITION -Werror)
add_executable(Sonar sonar.cpp)
target_link_libraries(Sonar wiringPi)

