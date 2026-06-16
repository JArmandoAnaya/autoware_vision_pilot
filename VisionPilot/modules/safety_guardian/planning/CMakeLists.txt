cmake_minimum_required(VERSION 3.22.1)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Newer CppAD (e.g. 2024 on Ubuntu 24.04) moved some symbols such as
# CppAD::local::temp_file() out of the headers into a compiled library; link
# libcppad_lib when it is present (older CppAD is header-only and won't have it).
find_library(CPPAD_LIB cppad_lib)

# eigen-5.0.0
add_subdirectory(src/eigen-5.0.0)

add_library(planning
        src/planning.cpp
        src/longitudinal_planning.cpp
        src/lateral_planning.cpp

)

target_include_directories(planning
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
        ${IPOPT_INCLUDE_DIRS}
)

target_compile_definitions(planning
        PRIVATE
        HAVE_CSTDDEF
)

target_link_libraries(planning
        PUBLIC
        Eigen3::Eigen
        PRIVATE
        ${IPOPT_LIBRARIES}
)

if(CPPAD_LIB)
    target_link_libraries(planning PRIVATE ${CPPAD_LIB})
endif()