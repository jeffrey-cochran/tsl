cmake_minimum_required(VERSION 3.10)
project(tse VERSION 1.0.0 LANGUAGES CXX)

# enable C++17 standard
set(CMAKE_CXX_STANDARD 17)

# flag this as a TSE build
set(TSE_BUILD TRUE)

# where to put non-compiled files
set(COPY_TARGET ${CMAKE_CURRENT_BINARY_DIR}/src)
# where to put tsl binaries
set(TSL_BINARY_LOC tsl)

# deps
find_package(OpenGL REQUIRED)

add_subdirectory(ext/imgui EXCLUDE_FROM_ALL)
include_directories(ext/imgui)

include_directories(ext/pfd)

add_subdirectory(ext/glfw EXCLUDE_FROM_ALL)

add_definitions(-DGLEW_STATIC)
add_subdirectory(ext/glew EXCLUDE_FROM_ALL)

add_subdirectory(../tsl ${TSL_BINARY_LOC} EXCLUDE_FROM_ALL)
include_directories(../tsl/include)

# application
set(TSE_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories(${TSE_INCLUDE_DIR})
add_subdirectory(src)

# copy shader files
file(COPY shader DESTINATION ${COPY_TARGET})

