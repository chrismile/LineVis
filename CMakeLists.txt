cmake_minimum_required (VERSION 3.11)
option(USE_GTEST "USE_GTEST" OFF)

project (StressLineVis)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)
#MESSAGE(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

set (CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.c)
include_directories(src)

#make VERBOSE=1

#if(MSVC)
#	set(CMAKE_CXX_FLAGS "-W3 /EHsc")
#elseif(CMAKE_COMPILER_IS_GNUCXX)
#	set(CMAKE_CXX_FLAGS "-Wall")
#elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
#	set(CMAKE_CXX_FLAGS "-Wall")
#endif()

IF(WIN32)
	add_executable(StressLineVis WIN32 ${SOURCES})
ELSE()
	add_executable(StressLineVis ${SOURCES})
ENDIF()

IF(WIN32)
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mwindows")
	target_link_libraries(StressLineVis mingw32 SDL2main SDL2)
ELSE()
	target_link_libraries(StressLineVis SDL2)
ENDIF()


cmake_policy(SET CMP0012 NEW)
find_package(OpenMP REQUIRED)
find_package(sgl REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(NetCDF REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
target_link_libraries(StressLineVis tinyxml2)
target_link_libraries(StressLineVis sgl ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${NETCDF_LIBRARIES} ${JSONCPP_LIBRARIES})

include_directories(${sgl_INCLUDES} ${Boost_INCLUDES} ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDES} ${NETCDF_INCLUDES} ${JSONCPP_INCLUDES})

if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


if (USE_GTEST)
	include(GoogleTest)
	enable_testing()
	add_executable(StressLineVis_test test/TestKdTreeNearestNeighbor.cpp
			src/LineData/SearchStructures/NearestNeighborNaive.cpp
			src/LineData/SearchStructures/SearchStructure.cpp
			src/LineData/SearchStructures/KdTree.cpp)
	target_link_libraries(StressLineVis_test gtest gtest_main)
	gtest_add_tests(TARGET StressLineVis_test)
endif()
