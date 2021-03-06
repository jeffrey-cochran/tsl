#ifndef TSL_VERTEX_HPP
#define TSL_VERTEX_HPP

#include "tsl/geometry/tmesh/handles.hpp"
#include "tsl/geometry/vector.hpp"

namespace tsl {

// Forward declarations
class optional_half_edge_handle;

/**
 * @brief Represents a vertex in the tmesh data structure.
 */
struct vertex
{
    /// The edge starting at this vertex.
    optional_half_edge_handle outgoing;

    /// The 3D position of this vertex.
    vec3 pos;

    /// if the vertex is a control mesh vertex or a virtual vertex of Bezier grid
    bool control_mesh_vertex;
};

}

#endif //TSL_VERTEX_HPP
