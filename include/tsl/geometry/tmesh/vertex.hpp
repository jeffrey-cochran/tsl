#ifndef TSL_VERTEX_HPP
#define TSL_VERTEX_HPP

#include "tsl/geometry/tmesh/handles.hpp"
#include "tsl/geometry/vector.hpp"

namespace tsl {

// Forward declarations
struct optional_half_edge_handle;

/**
 * @brief Represents a vertex in the tmesh data structure.
 */
struct vertex
{
    /// The edge starting at this vertex.
    optional_half_edge_handle outgoing;

    /// The 3D position of this vertex.
    vec3 pos;
};

}

#endif //TSL_VERTEX_HPP
