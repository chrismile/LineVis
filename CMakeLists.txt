cmake_minimum_required (VERSION 3.5)
option(USE_GTEST "USE_GTEST" OFF)

project (LineVis)
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

# Prefer GLVND by default when available.
if (${CMAKE_VERSION} VERSION_GREATER "3.11")
	cmake_policy(SET CMP0072 NEW)
endif()

option(USE_PYTHON "Build with Python 3 support for replay scripts" ON)

find_package(PkgConfig REQUIRED)

if (USE_PYTHON)
	if (${CMAKE_VERSION} VERSION_GREATER "3.11")
		find_package(Python3 COMPONENTS Development QUIET)
	else()
		pkg_check_modules(Python3 QUIET python3)
	endif()
endif()

if (Python3_FOUND AND USE_PYTHON)
	MESSAGE(STATUS "Found Python 3. Enabling replay script support.")
else()
	list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Widgets/ReplayWidget.cpp)
	MESSAGE(STATUS "Python 3 couldn't be found. Switching off replay script support.")
endif()


IF(WIN32)
	add_executable(LineVis WIN32 ${SOURCES})
ELSE()
	add_executable(LineVis ${SOURCES})
ENDIF()

IF(WIN32)
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mwindows")
	target_link_libraries(LineVis PUBLIC mingw32 SDL2main SDL2)
ELSE()
	target_link_libraries(LineVis PUBLIC SDL2)
ENDIF()


cmake_policy(SET CMP0012 NEW)
find_package(OpenMP REQUIRED)
find_package(sgl REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(NetCDF REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
target_link_libraries(LineVis PUBLIC tinyxml2)
target_link_libraries(LineVis PUBLIC sgl ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} GLEW::GLEW ${NETCDF_LIBRARIES} ${JSONCPP_LIBRARIES})
include_directories(${sgl_INCLUDES} ${Boost_INCLUDES} ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDES} ${NETCDF_INCLUDES} ${JSONCPP_INCLUDES})

find_package(cppzmq QUIET)
if (${cppzmq_FOUND})
	target_link_libraries(LineVis PUBLIC cppzmq)
	if (${CMAKE_VERSION} VERSION_LESS "3.12")
		add_definitions(-DUSE_ZEROMQ)
	else()
		add_compile_definitions(USE_ZEROMQ)
	endif()
else()
	# https://stackoverflow.com/questions/41251474/how-to-import-zeromq-libraries-in-cmake/41252437
	pkg_check_modules(PC_ZeroMQ QUIET zmq)
	#if (${PC_ZeroMQ_FOUND})
		find_path(ZeroMQ_INCLUDE_DIR NAMES zmq.hpp PATHS ${PC_ZeroMQ_INCLUDE_DIRS})
		find_library(ZeroMQ_LIBRARY NAMES zmq PATHS ${PC_ZeroMQ_LIBRARY_DIRS})
		target_include_directories(LineVis PUBLIC ${ZeroMQ_INCLUDE_DIR})
		target_link_libraries(LineVis PUBLIC ${ZeroMQ_LIBRARY})
		if (${CMAKE_VERSION} VERSION_LESS "3.12")
			add_definitions(-DUSE_ZEROMQ)
		else()
			add_compile_definitions(USE_ZEROMQ)
		endif()
	#endif()
endif()

if (Python3_FOUND AND USE_PYTHON)
	add_definitions(-DUSE_PYTHON)
	if (${CMAKE_VERSION} VERSION_GREATER "3.11")
		target_link_libraries(LineVis PUBLIC Python3::Python)
	else()
		target_link_libraries(LineVis PUBLIC ${Python3_LIBRARY})
	endif()
	target_include_directories(LineVis PUBLIC ${Python3_INCLUDE_DIRS})
endif()

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	if (${CMAKE_VERSION} VERSION_LESS "3.10")
		add_definitions(-DOPENMP_NO_MEMBERS)
	else()
		# Put check into else, as OpenMP_CXX_VERSION might not be defined otherwise.
		if (${OpenMP_CXX_VERSION} VERSION_LESS "4.5")
			add_definitions(-DOPENMP_NO_MEMBERS)
		endif()
	endif()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


if (USE_GTEST)
	include(GoogleTest)
	enable_testing()
	add_executable(LineVis_test test/TestKdTreeNearestNeighbor.cpp
			src/LineData/SearchStructures/NearestNeighborNaive.cpp
			src/LineData/SearchStructures/SearchStructure.cpp
			src/LineData/SearchStructures/KdTree.cpp)
	target_link_libraries(LineVis_test gtest gtest_main)
	gtest_add_tests(TARGET LineVis_test)
endif()
