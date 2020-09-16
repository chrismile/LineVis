find_path(SGL_INCLUDE_DIR NAMES sgl)
find_library(SGL_LIBRARIES NAMES sgl)
if (NOT SGL_INCLUDE_DIR)
	set(SGL_INCLUDE_DIR "../sgl/src")
else()
	set(SGL_INCLUDE_DIR "${SGL_INCLUDE_DIR}/sgl")
endif()
include_directories(${SGL_INCLUDE_DIR})
#message(${SGL_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SGL DEFAULT_MSG SGL_LIBRARIES SGL_INCLUDE_DIR)

mark_as_advanced(SGL_INCLUDE_DIR SGL_LIBRARIES)
