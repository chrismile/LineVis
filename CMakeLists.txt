cmake_minimum_required (VERSION 3.7)
cmake_policy(VERSION 3.7...3.20)

project(LineVis)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)
#MESSAGE(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCES src/*.cpp src/*.c src/*.hpp src/*.h Data/Shaders/*.glsl)
include_directories(src)

file(GLOB_RECURSE SHADER_FILES Data/Shaders/*.glsl)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/Data/Shaders" PREFIX "Shaders" FILES ${SHADER_FILES})

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus")
    # Avoid warning 'IMPORTED_IMPLIB not set for imported target "sgl" configuration "MinSizeRel/RelWithDebInfo".'
    set(CMAKE_CONFIGURATION_TYPES Debug Release)
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W3 /EHsc /Zc:__cplusplus")
elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

# Prefer GLVND by default when available.
if (${CMAKE_VERSION} VERSION_GREATER "3.11")
    cmake_policy(SET CMP0072 NEW)
endif()

if(VCPKG_TOOLCHAIN)
    if(EXISTS "${Z_VCPKG_ROOT_DIR}/triplets/${VCPKG_TARGET_TRIPLET}.cmake")
        include("${Z_VCPKG_ROOT_DIR}/triplets/${VCPKG_TARGET_TRIPLET}.cmake")
    elseif(EXISTS "${Z_VCPKG_ROOT_DIR}/triplets/community/${VCPKG_TARGET_TRIPLET}.cmake")
        include("${Z_VCPKG_ROOT_DIR}/triplets/community/${VCPKG_TARGET_TRIPLET}.cmake")
    else()
        message(FATAL_ERROR "Could not find a vcpkg triplet file for ${VCPKG_TARGET_TRIPLET}.")
    endif()
endif()

option(USE_GTEST "Build the GoogleTest test suites." OFF)
option(USE_PYTHON "Build with Python 3 support for replay scripts." ON)
option(VCPKG_EXTERNAL_EMBREE "Use external Embree package if vcpkg is used for compilation." OFF)
option(USE_STATIC_STD_LIBRARIES "Link with standard libraries statically (only supported on Linux for now)." OFF)

set(DATA_PATH "${CMAKE_SOURCE_DIR}/Data" CACHE PATH "Location of folder 'Data'")
add_definitions(-DDATA_PATH=\"${DATA_PATH}\")

if (USE_GTEST)
    set(
            GTEST_SOURCES
            # Test 1: Kd-tree functionality.
            ${CMAKE_CURRENT_SOURCE_DIR}/test/TestKdTreeNearestNeighbor.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/LineData/SearchStructures/NearestNeighborNaive.cpp
            # Test 2: Volumetric path tracing.
            ${CMAKE_CURRENT_SOURCE_DIR}/test/VolumetricPathTracingTestData.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/test/VolumetricPathTracingTestRenderer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/test/TestVolumetricPathTracing.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/LineData/Scattering/CloudData.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/OIT/MBOITUtils.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/PathTracer/VolumetricPathTracingPass.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/PathTracer/SuperVoxelGrid.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/PathTracer/OpenExrLoader.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/Denoiser/Denoiser.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/Denoiser/EAWDenoiser.cpp
    )
endif()

if ((UNIX OR MSYS) AND (NOT APPLE OR NOT VCPKG_TOOLCHAIN))
    find_package(PkgConfig REQUIRED)
endif()


if (USE_PYTHON)
    if (${CMAKE_VERSION} VERSION_GREATER "3.11")
        # 2021-05-06: CMake won't find vcpkg version if we don't search for the interpreter.
        # 2022-08-29: Finding interpreter fails with dynamic vcpkg build on Linux and macOS.
        if(VCPKG_TOOLCHAIN) #  AND (NOT UNIX OR VCPKG_LIBRARY_LINKAGE STREQUAL "static")
            find_package(Python3 COMPONENTS Development Interpreter REQUIRED)
        else()
            find_package(Python3 COMPONENTS Development REQUIRED)
        endif()
    else()
        pkg_check_modules(Python3 QUIET python3)
    endif()
endif()

if (Python3_FOUND AND USE_PYTHON)
    MESSAGE(STATUS "Found Python ${Python3_VERSION}. Enabling replay script support.")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/pythonversion.txt" "python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
else()
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Widgets/ReplayWidget.cpp)
    MESSAGE(STATUS "Python 3 couldn't be found. Switching off replay script support.")
endif()


set(FPHSA_NAME_MISMATCHED TRUE)
if(VCPKG_TOOLCHAIN)
    find_package(SDL2 CONFIG REQUIRED)
else()
    find_package(SDL2 REQUIRED)
endif()

find_package(sgl REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
if(VCPKG_TOOLCHAIN)
    find_package(netCDF CONFIG REQUIRED)
    find_package(tinyxml2 CONFIG REQUIRED)
    find_package(jsoncpp CONFIG REQUIRED)
else()
    find_package(NetCDF REQUIRED)
    find_package(TinyXML2 REQUIRED)
    find_package(jsoncpp QUIET)
    if (NOT jsoncpp_FOUND)
        pkg_check_modules(JSONCPP jsoncpp)
    endif()
endif()

find_package(eccodes QUIET)
if(NOT eccodes_FOUND)
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LineData/Flow/Loader/GribLoader.cpp)
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/LineData/Flow/Loader/GribLoader.hpp)
    MESSAGE(STATUS "ecCodes couldn't be found. Disabling GRIB file loading.")
endif()

get_target_property(SGL_INTERFACE_COMPILE_DEFINITIONS sgl INTERFACE_COMPILE_DEFINITIONS)
if (NOT "SUPPORT_VULKAN" IN_LIST SGL_INTERFACE_COMPILE_DEFINITIONS)
    message(FATAL_ERROR "sgl was not built with Vulkan support enabled.")
endif()

# Embree version 3.12.1-2 shipped by msys2 seems to be buggy and crashes on rtcCommitScene :(
if(NOT "${CMAKE_GENERATOR}" STREQUAL "MSYS Makefiles")
    if(VCPKG_TOOLCHAIN AND NOT ${VCPKG_EXTERNAL_EMBREE})
        find_package(TBB QUIET)
        find_package(embree 3 QUIET)
    else()
        find_package(embree 3.0 QUIET)
    endif()
endif()
if(embree_FOUND)
    MESSAGE(STATUS "Found Embree. Enabling Embree as as a ray-mesh intersection library for RTAO pre-baking.")
else()
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/AmbientOcclusion/EmbreeAmbientOcclusionBaker.cpp)
    MESSAGE(STATUS "Embree couldn't be found. Disabling Embree RTAO pre-baking support.")
endif()

if(embree_FOUND)
    find_package(ospray QUIET)
    if(ospray_FOUND)
        MESSAGE(STATUS "Found OSPRay. Enabling CPU ray tracing using OSPRay.")
    endif()
endif()

if(NOT embree_FOUND OR NOT ospray_FOUND)
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Ospray/OsprayRenderer.cpp)
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Ospray/OsprayRenderer.hpp)
    MESSAGE(STATUS "OSPRay couldn't be found. Disabling CPU ray tracing.")
endif()

if ("SUPPORT_CUDA_INTEROP" IN_LIST SGL_INTERFACE_COMPILE_DEFINITIONS)
    if (NOT OptiX_INSTALL_DIR)
        if (WIN32 AND NOT OptiX_INSTALL_DIR)
            file(GLOB OPTIX_DIR_LIST "C:/ProgramData/NVIDIA Corporation/OptiX*")
            if (OPTIX_DIR_LIST)
                list(LENGTH OPTIX_DIR_LIST OPTIX_DIR_LIST_LEN)
                math(EXPR OPTIX_DIR_LIST_LAST "${OPTIX_DIR_LIST_LEN} - 1")
                list(GET OPTIX_DIR_LIST ${OPTIX_DIR_LIST_LAST} OPTIX_DIR)
            endif()
        else()
            file(GLOB OPTIX_DIR_LIST "$ENV{HOME}/nvidia/NVIDIA-OptiX-SDK*")
            if (OPTIX_DIR_LIST)
                list(LENGTH OPTIX_DIR_LIST OPTIX_DIR_LIST_LEN)
                math(EXPR OPTIX_DIR_LIST_LAST "${OPTIX_DIR_LIST_LEN} - 1")
                list(GET OPTIX_DIR_LIST ${OPTIX_DIR_LIST_LAST} OPTIX_DIR)
            endif()
        endif()
        if (OPTIX_DIR)
            set(OptiX_INSTALL_DIR "${OPTIX_DIR}" CACHE PATH "Path to OptiX installation.")
        endif()
    endif()
    if (OptiX_INSTALL_DIR)
        list(APPEND CMAKE_MODULE_PATH ${OptiX_INSTALL_DIR}/SDK/CMake)
    endif()
    find_package(OptiX QUIET)
    if (OptiX_FOUND OR OptiX_INCLUDE)
        set(SUPPORT_OPTIX ON)
        MESSAGE(STATUS "NVIDIA OptiX found. Enabling OptiX denoiser support.")
        MESSAGE(STATUS "OptiX_INSTALL_DIR: " ${OptiX_INSTALL_DIR})
    else()
        MESSAGE(STATUS "OptiX couldn't be found. Disabling OptiX denoiser support.")
    endif()
else()
    MESSAGE(STATUS "sgl was not built with CUDA interoperability support. Disabling OptiX denoiser support.")
endif()
if (NOT SUPPORT_OPTIX)
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/Denoiser/OptixVptDenoiser.cpp)
endif()

find_package(Imath CONFIG QUIET)
if (NOT TARGET Imath::Imath)
    find_package(IlmBase CONFIG QUIET)
endif()
find_package(OpenEXR CONFIG QUIET)
if (NOT OpenEXR_FOUND AND NOT Imath_FOUND AND UNIX AND NOT APPLE)
    set(USE_OPENEXR_PKGCONFIG TRUE)
    pkg_check_modules(OpenEXR OpenEXR QUIET)
    pkg_check_modules(IlmBase IlmBase QUIET)
else()
    set(USE_OPENEXR_PKGCONFIG FALSE)
endif()
if (OpenEXR_FOUND AND (Imath_FOUND OR IlmBase_FOUND))
    if (${OpenEXR_VERSION} VERSION_LESS "2.3.0")
        MESSAGE(STATUS "Found OpenEXR version ${OpenEXR_VERSION} is too old. Please upgrade to version >= 2.3.0.")
    else()
        MESSAGE(STATUS "Found OpenEXR ${OpenEXR_VERSION}. Enabling OpenEXR image loading support.")
        set(SUPPORT_OPENEXR true)
    endif()
else()
    MESSAGE(STATUS "OpenEXR not found. Disabling OpenEXR image loading support.")
endif()
if (NOT SUPPORT_OPENEXR)
    list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/PathTracer/OpenExrLoader.cpp)
    if (USE_GTEST)
        list(REMOVE_ITEM GTEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/Renderers/Scattering/PathTracer/OpenExrLoader.cpp)
    endif()
endif()


if(WIN32)
    add_executable(LineVis WIN32 ${SOURCES})
elseif(APPLE)
    add_executable(LineVis MACOSX_BUNDLE ${SOURCES})
else()
    add_executable(LineVis ${SOURCES})
endif()

if(${USE_STATIC_STD_LIBRARIES})
    target_link_options(LineVis PRIVATE -static-libgcc -static-libstdc++)
endif()

if (APPLE)
    set(BUNDLE_PATH ${CMAKE_CURRENT_BINARY_DIR}/LineVis.app)
    set_target_properties(
            LineVis PROPERTIES
            OUTPUT_NAME "LineVis"
            MACOSX_BUNDLE_EXECUTABLE_NAME "LineVis"
            MACOSX_BUNDLE_BUNDLE_NAME "LineVis"
            FOLDER "LineVis"
            MACOSX_BUNDLE_SHORT_VERSION_STRING ""
            MACOSX_BUNDLE_LONG_VERSION_STRING ""
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/CMake/MacOSXBundleInfo.plist.in")
endif()

if (USE_GTEST)
    include(GoogleTest)
    enable_testing()
    add_executable(LineVis_test ${GTEST_SOURCES})
    target_link_libraries(LineVis_test PRIVATE gtest gtest_main)
    target_link_libraries(LineVis_test PRIVATE sgl ${Boost_LIBRARIES})
    target_include_directories(LineVis_test PRIVATE ${sgl_INCLUDES} ${Boost_INCLUDE_DIR})
    gtest_add_tests(TARGET LineVis_test)
endif()


# Hide console window in release build on Windows/MSVC.
# See: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/recipe/VSConfigSpecificSettings
if(MSVC)
    set_target_properties(LineVis PROPERTIES LINK_FLAGS_DEBUG "/SUBSYSTEM:CONSOLE")
    set_target_properties(LineVis PROPERTIES COMPILE_DEFINITIONS_DEBUG "_CONSOLE")
    set_target_properties(LineVis PROPERTIES LINK_FLAGS_RELWITHDEBINFO "/SUBSYSTEM:CONSOLE")
    set_target_properties(LineVis PROPERTIES COMPILE_DEFINITIONS_RELWITHDEBINFO "_CONSOLE")
    set_target_properties(LineVis PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
    set_target_properties(LineVis PROPERTIES LINK_FLAGS_MINSIZEREL "/SUBSYSTEM:WINDOWS")
endif()

if(MSVC)
    #target_compile_definitions(LineVis PRIVATE /wd4251)
    set_target_properties(LineVis PROPERTIES COMPILE_FLAGS "/wd4251")
    target_compile_options(LineVis PRIVATE "/MP")
endif()

add_subdirectory(submodules/IsosurfaceCpp)
if("TRACY_ENABLE" IN_LIST SGL_INTERFACE_COMPILE_DEFINITIONS)
    target_compile_definitions(IsosurfaceCpp PUBLIC TRACY_ENABLE)
    target_compile_definitions(IsosurfaceCpp PUBLIC TRACY_IMPORTS)
    target_include_directories(IsosurfaceCpp PUBLIC $<TARGET_PROPERTY:sgl,INTERFACE_INCLUDE_DIRECTORIES>)
endif()
target_link_libraries(LineVis PRIVATE IsosurfaceCpp)
target_include_directories(LineVis PRIVATE submodules)

add_subdirectory(submodules/bvh)
target_link_libraries(LineVis PRIVATE bvh)

if (NOT MSVC)
    if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        target_compile_definitions(LineVis PRIVATE NDEBUG)
    endif()
endif()

if((${CMAKE_GENERATOR} STREQUAL "MinGW Makefiles") OR (${CMAKE_GENERATOR} STREQUAL "MSYS Makefiles"))
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mconsole")
    else()
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mwindows")
    endif()
    target_link_libraries(LineVis PUBLIC mingw32)
endif()
if(VCPKG_TOOLCHAIN)
    target_link_libraries(LineVis PUBLIC SDL2::SDL2main)
else()
    target_link_libraries(LineVis PUBLIC SDL2::Main)
endif()

target_link_libraries(LineVis PUBLIC sgl ${Boost_LIBRARIES})
include_directories(${sgl_INCLUDES} ${Boost_INCLUDE_DIR})

if(VCPKG_TOOLCHAIN)
    target_link_libraries(LineVis PRIVATE tinyxml2::tinyxml2 netCDF::netcdf)
else()
    target_link_libraries(LineVis PRIVATE ${TINYXML2_LIBRARIES} ${NETCDF_LIBRARIES})
    target_include_directories(LineVis PRIVATE ${NETCDF_INCLUDE_DIR})
endif()
target_include_directories(LineVis PRIVATE ${sgl_INCLUDES} ${TINYXML2_INCLUDE_DIR})

if (eccodes_FOUND)
    MESSAGE(STATUS "Found ecCodes ${eccodes_VERSION}. Enabling GRIB file loading support.")
    target_link_libraries(LineVis PRIVATE ${eccodes_LIBRARIES} eccodes)
    target_include_directories(LineVis PRIVATE ${eccodes_INCLUDE_DIR})
    target_compile_definitions(LineVis PRIVATE USE_ECCODES)
endif()

if (JSONCPP_LIBRARIES)
    target_link_libraries(LineVis PRIVATE ${JSONCPP_LIBRARIES})
    target_link_directories(LineVis PRIVATE ${JSONCPP_LIBRARY_DIRS})
    target_include_directories(LineVis PRIVATE ${JSONCPP_INCLUDE_DIRS})
    if (USE_GTEST)
        target_link_libraries(LineVis_test PRIVATE ${JSONCPP_LIBRARIES})
        target_link_directories(LineVis_test PRIVATE ${JSONCPP_LIBRARY_DIRS})
        target_include_directories(LineVis_test PRIVATE ${JSONCPP_INCLUDE_DIRS})
    endif()
endif()
if (TARGET jsoncpp_lib)
    get_target_property(jsoncpp_INCLUDES jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
    target_link_libraries(LineVis PRIVATE jsoncpp_lib)
    if (USE_GTEST)
        target_link_libraries(LineVis_test PRIVATE jsoncpp_lib)
    endif()
elseif(TARGET jsoncpp_static)
    get_target_property(jsoncpp_INCLUDES jsoncpp_static INTERFACE_INCLUDE_DIRECTORIES)
    target_link_libraries(LineVis PRIVATE jsoncpp_static)
    if (USE_GTEST)
        target_link_libraries(LineVis_test PRIVATE jsoncpp_static)
    endif()
endif()
if (TARGET jsoncpp_object)
    target_link_libraries(LineVis PRIVATE jsoncpp_object)
    if (USE_GTEST)
        target_link_libraries(LineVis_test PRIVATE jsoncpp_object)
    endif()
endif()
target_include_directories(LineVis PRIVATE ${jsoncpp_INCLUDES})
if (USE_GTEST)
    target_include_directories(LineVis_test PRIVATE ${jsoncpp_INCLUDES})
endif()

if(VCPKG_TOOLCHAIN)
    find_package(cppzmq CONFIG QUIET)
else()
    find_package(cppzmq QUIET)
endif()
if (${cppzmq_FOUND})
    target_link_libraries(LineVis PRIVATE cppzmq)
    if (${CMAKE_VERSION} VERSION_LESS "3.12")
        target_compile_definitions(LineVis PRIVATE USE_ZEROMQ)
    else()
        add_compile_definitions(USE_ZEROMQ)
    endif()
else()
    # https://stackoverflow.com/questions/41251474/how-to-import-zeromq-libraries-in-cmake/41252437
    pkg_check_modules(PC_ZeroMQ QUIET zmq)
    #if (${PC_ZeroMQ_FOUND})
        find_path(ZeroMQ_INCLUDE_DIRS NAMES zmq.hpp PATHS ${PC_ZeroMQ_INCLUDE_DIRS})
        find_library(ZeroMQ_LIBRARIES NAMES zmq PATHS ${PC_ZeroMQ_LIBRARY_DIRS})
        target_include_directories(LineVis PRIVATE ${ZeroMQ_INCLUDE_DIRS})
        target_link_libraries(LineVis PRIVATE ${ZeroMQ_LIBRARIES})
        if (${CMAKE_VERSION} VERSION_LESS "3.12")
            target_compile_definitions(LineVis PRIVATE USE_ZEROMQ)
        else()
            add_compile_definitions(USE_ZEROMQ)
        endif()
    #endif()
endif()

# Eigen is an optional dependency for eigenvalue solving.
if(VCPKG_TOOLCHAIN)
    find_package(Eigen3 CONFIG QUIET)
else()
    find_package(Eigen3 QUIET)
endif()
if(Eigen3_FOUND)
    if(NOT (EIGEN3_VERSION_STRING LESS 3.3))
        target_link_libraries(LineVis PRIVATE Eigen3::Eigen)
    else()
        include_directories(${EIGEN3_INCLUDE_DIR})
    endif()
   target_compile_definitions(LineVis PRIVATE USE_EIGEN)
endif()

if (Python3_FOUND AND USE_PYTHON)
    target_compile_definitions(LineVis PRIVATE USE_PYTHON)
    if(${CMAKE_VERSION} VERSION_GREATER "3.11")
        target_link_libraries(LineVis PRIVATE Python3::Python)
    else()
        target_link_libraries(LineVis PRIVATE ${Python3_LIBRARIES})
    endif()
    if(UNIX AND NOT APPLE AND VCPKG_TOOLCHAIN AND ${CMAKE_VERSION} VERSION_GREATER "3.12")
        # Python linked statically in this case. See:
        # https://stackoverflow.com/questions/67891197/ctypes-cpython-39-x86-64-linux-gnu-so-undefined-symbol-pyfloat-type-in-embedd
        if(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
            target_link_options(LineVis PRIVATE "-Xlinker")
            target_link_options(LineVis PRIVATE "-export-dynamic")
        endif()
    endif()
    target_include_directories(LineVis PRIVATE ${Python3_INCLUDE_DIRS})

    # Add default PYTHONHOME path for vcpkg if no environment variable is set.
    if(DEFINED ENV{PYTHONHOME})
        set(PYTHONHOME "$ENV{PYTHONHOME}")
    endif()
    if(VCPKG_TOOLCHAIN AND (NOT APPLE OR NOT PYTHONHOME))
        if(NOT DEFINED ENV{PYTHONHOME})
            find_path(PYTHONHOME_PATH tools/python3 HINTS ${PYTHONHOME_PATH})
            if (MSVC)
                set(PYTHONHOME "${PYTHONHOME_PATH}/tools/python3")
            else()
                set(PYTHONHOME "${PYTHONHOME_PATH}")
            endif()
        endif()
    endif()
    if (PYTHONHOME)
        if (NOT PYTHONPATH AND (APPLE OR MSYS))
            set(PYTHONPATH "${PYTHONHOME}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")
        endif()
        set(PYTHONHOME "${PYTHONHOME}" CACHE STRING "PYTHONHOME path.")
        target_compile_definitions(LineVis PRIVATE PYTHONHOME=L\"${PYTHONHOME}\")
    endif()
    if (PYTHONPATH)
        set(PYTHONPATH "${PYTHONPATH}" CACHE STRING "PYTHONPATH path.")
        target_compile_definitions(LineVis PRIVATE PYTHONPATH=L\"${PYTHONPATH}\")
    endif()
endif()

# sgl provides Vulkan headers in its interface, which are provided if Vulkan was not found during its build.
find_package(Vulkan QUIET)
if (${Vulkan_FOUND} AND NOT "USE_SGL_VULKAN_HEADERS" IN_LIST SGL_INTERFACE_COMPILE_DEFINITIONS
        AND ${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.21.0")
    target_link_libraries(LineVis PUBLIC Vulkan::Headers)
endif()

if(embree_FOUND)
    if(VCPKG_TOOLCHAIN)
        target_link_libraries(LineVis PRIVATE embree)
    else()
        target_link_libraries(LineVis PRIVATE ${EMBREE_LIBRARIES})
    endif()
    target_include_directories(LineVis PRIVATE ${EMBREE_INCLUDE_DIRS})
    target_compile_definitions(LineVis PRIVATE USE_EMBREE)
endif()

if(ospray_FOUND)
    target_link_libraries(LineVis PRIVATE ospray::ospray)
    target_include_directories(LineVis PRIVATE ${OSPRAY_INCLUDE_DIRS})
    target_compile_definitions(LineVis PRIVATE USE_OSPRAY)
endif()

if (${SUPPORT_OPTIX})
    set(CMAKE_MODULE_PATH ${OptiX_INSTALL_DIR}/SDK/CMake ${CMAKE_MODULE_PATH})
    find_package(OptiX QUIET)
    if (OptiX_FOUND OR OptiX_INCLUDE)
        target_compile_definitions(LineVis PRIVATE SUPPORT_OPTIX)
        target_include_directories(LineVis PRIVATE ${OptiX_INCLUDE})
        if(UNIX AND NOT APPLE)
            target_link_libraries(LineVis PRIVATE dl)
        endif()
        if (MSYS)
            # GetProcAddress in optix_stubs.h performs no function pointer conversion.
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
            # Compensate for "#pragma comment(lib, "Cfgmgr32.lib")" not working with MinGW.
            target_link_libraries(LineVis PRIVATE cfgmgr32)
        endif()
    endif()
endif()

if (SUPPORT_OPENEXR)
    if (${USE_OPENEXR_PKGCONFIG})
        if (OpenEXR_FOUND)
            target_link_libraries(LineVis PRIVATE ${OpenEXR_LIBRARIES})
            if (USE_GTEST)
                target_link_libraries(LineVis_test PRIVATE ${OpenEXR_LIBRARIES})
            endif()
        endif()
        if (IlmBase_FOUND)
            target_link_libraries(LineVis PRIVATE ${IlmBase_LIBRARIES})
            if (USE_GTEST)
                target_link_libraries(LineVis_test PRIVATE ${IlmBase_LIBRARIES})
            endif()
        endif()
    else()
        target_link_libraries(
                LineVis PRIVATE
                # For OpenEXR/Imath 3.x:
                $<$<TARGET_EXISTS:OpenEXR::OpenEXR>:OpenEXR::OpenEXR>
                $<$<TARGET_EXISTS:Imath::Imath>:Imath::Imath>
                $<$<TARGET_EXISTS:Imath::Half>:Imath::Half>
                # For OpenEXR 2.4/2.5:
                $<$<TARGET_EXISTS:OpenEXR::IlmImf>:OpenEXR::IlmImf>
                $<$<TARGET_EXISTS:IlmBase::Imath>:IlmBase::Imath>
                $<$<TARGET_EXISTS:IlmBase::Half>:IlmBase::Half>
                $<$<TARGET_EXISTS:IlmBase::IlmThread>:IlmBase::IlmThread>
                $<$<TARGET_EXISTS:IlmBase::Iex>:IlmBase::Iex>
        )
        if (USE_GTEST)
            target_link_libraries(
                    LineVis_test PRIVATE
                    # For OpenEXR/Imath 3.x:
                    $<$<TARGET_EXISTS:OpenEXR::OpenEXR>:OpenEXR::OpenEXR>
                    $<$<TARGET_EXISTS:Imath::Imath>:Imath::Imath>
                    $<$<TARGET_EXISTS:Imath::Half>:Imath::Half>
                    # For OpenEXR 2.4/2.5:
                    $<$<TARGET_EXISTS:OpenEXR::IlmImf>:OpenEXR::IlmImf>
                    $<$<TARGET_EXISTS:IlmBase::Imath>:IlmBase::Imath>
                    $<$<TARGET_EXISTS:IlmBase::Half>:IlmBase::Half>
                    $<$<TARGET_EXISTS:IlmBase::IlmThread>:IlmBase::IlmThread>
                    $<$<TARGET_EXISTS:IlmBase::Iex>:IlmBase::Iex>
            )
        endif()
    endif()
    target_compile_definitions(LineVis PRIVATE SUPPORT_OPENEXR)
    if (USE_GTEST)
        target_compile_definitions(LineVis_test PRIVATE SUPPORT_OPENEXR)
    endif()
endif()

# According to https://devblogs.microsoft.com/cppblog/improved-openmp-support-for-cpp-in-visual-studio/,
# support for LLVM OpenMP was added with Visual Studio 2019 version 16.9. According to
# https://docs.microsoft.com/en-us/cpp/preprocessor/predefined-macros?view=msvc-170,
# both Visual Studio 2019 version 16.8 and 16.9 have _MSC_VER / MSVC_VERSION 1928, so we need to check
# for a minimum version 1929.
find_package(OpenMP REQUIRED)
if(MSVC)
    option(USE_MSVC_OPENMP_LLVM "Use LLVM OpenMP with MSVC (available since Visual Studio 2019 version 16.9)." ON)
    option(USE_MSVC_OPENMP_EXPERIMENTAL "Use experimental OpenMP with MSVC (available since Visual Studio 2019 version 16.9)." ON)

    if(${MSVC_VERSION} GREATER_EQUAL 1929 AND (USE_MSVC_OPENMP_LLVM OR USE_MSVC_OPENMP_EXPERIMENTAL))
        define_property(
                SOURCE
                PROPERTY COMPILE_FLAGS
                INHERITED
                BRIEF_DOCS "brief-doc"
                FULL_DOCS "full-doc"
        )
        if(USE_MSVC_OPENMP_LLVM)
            #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /openmp:llvm")
            #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp:llvm")
            set_directory_properties(PROPERTIES COMPILE_FLAGS "/openmp:llvm")
        endif()
        if(USE_MSVC_OPENMP_EXPERIMENTAL)
            #set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /openmp:experimental")
            #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp:experimental")
            set_directory_properties(PROPERTIES COMPILE_FLAGS "/openmp:experimental")
        endif()
        if(USE_MSVC_OPENMP_LLVM AND USE_MSVC_OPENMP_EXPERIMENTAL)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /w44005")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w44005")
        endif()
        set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/src/LineData/TrianglePayload/NodesBVHTreePayload.cpp" PROPERTIES COMPILE_FLAGS "")
    endif()

elseif(NOT TARGET OpenMP::OpenMP_CXX)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    target_link_libraries(LineVis PUBLIC OpenMP::OpenMP_CXX)
    if (USE_GTEST)
        target_link_libraries(LineVis_test PUBLIC OpenMP::OpenMP_CXX)
    endif()
endif()
