cmake_minimum_required(VERSION 3.12)
project(eskf)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(pyeskf src/pyESKF.cpp ESKF.cpp)
target_link_libraries(pyeskf PRIVATE Eigen3::Eigen)
