#ifndef TSL_HALF_EDGE_HPP
#define TSL_HALF_EDGE_HPP

#include <optional>

#include "handles.hpp"

using std::optional;

namespace tsl {

/**
 * @brief Represents a half edge in the tmesh data structure.
 */
struct half_edge
{
    /// The face this edge belongs to (or none, if this edge lies on the
    /// boundary).
    optional_face_handle face;

    /// The vertex this edge points to.
    vertex_handle target;

    /// The next edge of the face, ordered counter-clockwise.
    half_edge_handle next;

    /// The previous edge of the face, ordered counter-clockwise. Or in other words:
    /// the next edge of the face, ordered clockwise.
    half_edge_handle prev;

    /// Denotes, if the half edge points into a face corner. `nullopt` if no face is assigned.
    optional<bool> corner;

    /// Knot interval assigned to this half edge. `nullopt` if no face is assigned.
    optional<double> knot;

    /// Whether the half-edge described here is part of the control mesh or the bezier mesh
    bool control_mesh_half_edge;

private:
    /**
     * @brief Initializes all fields with dummy values (unsafe, thus private).
     */
    half_edge() : target(0), next(0), prev(0), control_mesh_half_edge(true) {}

    /// Several methods of T-Mesh need to invoke the unsafe ctor.
    friend class tmesh;
};

}

#endif //TSL_HALF_EDGE_HPP
