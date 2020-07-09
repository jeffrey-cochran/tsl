# TSL
add_library(tsl SHARED STATIC
    algorithm/generator.cpp
    algorithm/get_vertices.cpp
    algorithm/reduction.cpp
    evaluation/subdevision.cpp
    evaluation/surface_evaluator.cpp
    geometry/line.cpp
    geometry/line_segment.cpp
    geometry/rectangle.cpp
    geometry/tmesh/iterator.cpp
    geometry/tmesh/tmesh.cpp
    geometry/transform.cpp
    io/obj.cpp
)

if(${BLAS_FOUND})
    target_link_libraries(tsl
        PUBLIC fmt
        PUBLIC glm
        PUBLIC blas
    )
else(${BLAS_FOUND})
    if(DEFINED ENV{OpenBLAS_LIBS})
	target_link_libraries(tsl
	    PUBLIC fmt
	    PUBLIC glm
	    PUBLIC $ENV{OpenBLAS_LIBS}
	)
    else(DEFINED ENV{OpenBLAS_LIBS})
	message(fatal_error "Library for BLAS not defined")
    endif(DEFINED ENV{OpenBLAS_LIBS})
endif(${BLAS_FOUND})

# TSL Benchmark
add_executable(tsl_benchmark
    benchmark.cpp
)

target_link_libraries(tsl_benchmark
    tsl
)
