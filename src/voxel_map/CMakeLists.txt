cmake_minimum_required(VERSION 2.8.3)
project(voxel_map)

find_package(Boost REQUIRED COMPONENTS chrono python system thread)

find_package(Matlab)
if(NOT ${Matlab_FOUND})
    message(STATUS "Set Matlab_ROOT_DIR to configure Matlab interface.")
endif()

find_package(OpenMP)
if(${OPENMP_FOUND})
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

enable_testing()
find_package(GTest)
if(${GTEST_FOUND})
    include_directories(BEFORE ${GTEST_INCLUDE_DIRS})
elseif(DEFINED GTEST_ROOT)
    # Includes or libraries not found in ${GTEST_ROOT}.
    # TODO: Check that all necessary files are available (headers and sources).
    message(STATUS "GTest source found at ${GTEST_ROOT}.")
    add_subdirectory(${GTEST_ROOT} ${CMAKE_BINARY_DIR}/gtest)
    set(GTEST_FOUND 1)
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
    include_directories(BEFORE ${GTEST_ROOT}/include)
elseif(EXISTS "/usr/src/gtest")
    message(STATUS "GTest source found at /usr/src/gtest.")
    add_subdirectory(/usr/src/gtest ${CMAKE_BINARY_DIR}/gtest)
    set(GTEST_FOUND 1)
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
else()
    message("Set GTEST_ROOT to configure tests.")
endif()

find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${Python_LIBRARY_DIRS})

include_directories(include)

#if(${GTEST_FOUND})
#    add_executable(voxel_map_test test/voxel_map_test.cpp)
#    target_link_libraries(voxel_map_test ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} ${GTEST_BOTH_LIBRARIES} ${OpenCV_LIBS})
#endif()

add_library(py_voxel_map SHARED python/voxel_map_module.cpp)
set_target_properties(py_voxel_map PROPERTIES
    OUTPUT_NAME voxel_map
    PREFIX "")
target_link_libraries(py_voxel_map ${PYTHON_LIBRARIES} ${Boost_LIBRARIES})

if(${Matlab_FOUND})
    matlab_add_mex(NAME mat_voxel_map SRC matlab/voxel_map.cpp OUTPUT_NAME voxel_map)
    matlab_add_mex(NAME mat_hash_map SRC matlab/hash_map.cpp OUTPUT_NAME hash_map)
endif()

install(TARGETS py_voxel_map
        LIBRARY DESTINATION lib/python2.7/dist-packages)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
