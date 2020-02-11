cmake_minimum_required(VERSION 3.12)
project(MCL)

# Use C++17
# SET(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export for intelliSense to find it
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Python packages
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

# Message out python version else report error
message(STATUS "Python version: ${PYTHONLIBS_VERSION_STRING}")

# Retrive numpy path
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/findumpyh.py "try:\n\timport numpy; print(numpy.get_include())\nexcept: pass\n")
exec_program("${PYTHON_EXECUTABLE}"
             ARGS "\"${CMAKE_CURRENT_BINARY_DIR}/findumpyh.py\""
             OUTPUT_VARIABLE NUMPY_PATH)

find_path(PYTHON_NUMPY_HEADER arrayobject.h ${NUMPY_PATH}/numpy)

if(NOT PYTHON_NUMPY_HEADER)
    message(FATAL_ERROR "NumPy header not found")
endif()

# Message out python and numpy path
message(STATUS "PYTHON_INCLUDE_DIR: ${PYTHON_INCLUDE_DIR}")
message(STATUS "NUMPYP_INCLUDE_DIR: ${NUMPY_PATH}")

# Include python and numpy directories
include_directories(${PYTHON_INCLUDE_DIR} ${NUMPY_PATH})

# Set source files in src directory
set(SOURCE_FILES
    include/matplotlibcpp.h
    include/mcl.h
    include/mcl.cpp)

# include src directory
include_directories(src)

# Make my src directory as a library by using the add_library()
add_library(${PROJECT_NAME}_LIB SHARED ${SOURCE_FILES})

# Link my libray with python and numpy libraries
target_link_libraries(${PROJECT_NAME}_LIB ${PYTHON_LIBRARY})

# Make executable

# main code
add_executable(${PROJECT_NAME} main.cpp)
# Resampling learning
add_executable(Resampling resampling.cpp)
# Resampling wheel
add_executable(Resampling_wheel resampling_wheel.cpp)
# Error Evaluation
add_executable(Error_evaluation error_eval.cpp)

# Link the executable with my library (which my library is linked to python and numpy library)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_LIB)
target_link_libraries(Resampling_wheel PUBLIC ${PROJECT_NAME}_LIB)
target_link_libraries(Error_evaluation PUBLIC ${PROJECT_NAME}_LIB)


# Please remember that the order of lines matters very much. 