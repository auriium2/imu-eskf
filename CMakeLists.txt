cmake_minimum_required(VERSION 3.22)
project(imu-eskf LANGUAGES C CXX)

# Let the user choose:
option(BUILD_EMBEDDED   "Build embedded firmware via PlatformIO" ON)
option(BUILD_PYTHON_EXT "Build Python extension via pybind11" OFF)

# Bring in your header-only core
add_subdirectory(eskf_lib)

if (BUILD_EMBEDDED)
  # — include the PIO‐generated helper so core→Arduino-Eigen is available
  include(${CMAKE_CURRENT_LIST_DIR}/embedded/.pio/build/<env>/platformio.cmake)

  # Your embedded firmware target
  add_executable(firmware
    embedded/src/main.cpp
    # …other sources
  )
  target_link_libraries(firmware PRIVATE core)
endif()

if (BUILD_PYTHON_EXT)
  find_package(pybind11 REQUIRED)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)

  # A pybind11 module called “core_py”
  pybind11_add_module(core_py
    python/bindings.cpp
  )

  # core_py needs your core headers + Eigen3
  target_link_libraries(core_py PRIVATE
    core                # pulls in core/src
    Eigen3::Eigen       # pulls in real Eigen on desktop
  )

  # If you need any extra definitions:
  # target_compile_definitions(core_py PRIVATE YOUR_DEFINE=1)
endif()
