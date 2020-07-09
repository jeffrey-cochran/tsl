cmake_minimum_required(VERSION 3.10)
project(tsl VERSION 1.0.0 LANGUAGES CXX)

# enable C++17 standard
set(CMAKE_CXX_STANDARD 17)

# Load OpenBLAS
find_package(BLAS)
if(${BLAS_FOUND})
    message(status "found blas")
else(${BLAS_FOUND})
    if(DEFINED ENV{OpenBLAS_INCLUDE})
	include_directories($ENV{OpenBLAS_INCLUDE})
    else(DEFINED ENV{OpenBLAS_INCLUDE})
        message(fatal_error "Could not locate BLAS")
    endif(DEFINED ENV{OpenBLAS_INCLUDE})
endif(${BLAS_FOUND})

# other deps
include_directories(ext/tinyobjloader)
if (NOT TARGET glm)
    add_subdirectory(ext/glm EXCLUDE_FROM_ALL)
endif()

if (NOT TARGET fmt)
    add_subdirectory(ext/fmt EXCLUDE_FROM_ALL)
endif()

# application
set(TSL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories(${TSL_INCLUDE_DIR})
add_subdirectory(src)
add_subdirectory(test)

# Make sure to copy eigenvalue files to the build location if building from tse
if(${TSE_BUILD})
    file(COPY eigenvalues DESTINATION ${COPY_TARGET})
endif()

# set location for copying the test data
set(COPY_TEST_TARGET ${CMAKE_CURRENT_BINARY_DIR}/test/src)
# copy eigenvalue and mesh files used in testing to the appropriate location
file(COPY eigenvalues DESTINATION ${COPY_TEST_TARGET})
file(COPY meshes/test_meshes DESTINATION ${COPY_TEST_TARGET})
