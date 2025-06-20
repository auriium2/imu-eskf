cmake_minimum_required(VERSION 3.12)
project(eskf)

# Enable compile commands export
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

pybind11_add_module(pyeskf bind/pyESKF.cpp src/ESKF.cpp)
target_link_libraries(pyeskf PRIVATE Eigen3::Eigen)

# Create a custom target to copy compile_commands.json
add_custom_target(copy_compile_commands ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_SOURCE_DIR}/compile_commands.json
  DEPENDS pyeskf
  COMMENT "Copying compile_commands.json to source directory"
  VERBATIM
)
