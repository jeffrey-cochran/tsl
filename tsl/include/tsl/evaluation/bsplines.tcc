#include <array>

#include "tsl/geometry/vector.hpp"

using std::array;

namespace tsl {

template<uint32_t degree>
vec2 get_bspline_with_der(double u, const vector<double>& knot_vector) {
    if (u < knot_vector[0] || u > knot_vector[degree + 1]) {
        return {0, 0};
    }
    // the unusual case of trying to actually evaluate the upper bound
    else if (u == knot_vector[degree + 1]) {
        vec2 out; 
	if (knot_vector[degree + 1] == knot_vector[1]) {
	    out.x = 1;
	} else {
	    out.x = 0;
	}

	if (degree > 0 && knot_vector[degree + 1] == knot_vector[2]) {
            out.y = degree;
	} else {
	    out.y = 0;
	}
	return out;
    }

    // Initialize zero degree funs
    array<array<double, degree + 1>, degree + 1> funs{};
    for (uint32_t j = 0; j <= degree; ++j) {
        if (u >= knot_vector[j] && u < knot_vector[j + 1]) {
            funs[j][0] = 1;
        } else {
            funs[j][0] = 0;
        }
    }

    // Compute triangular table
    for (uint32_t k = 1; k <= degree; ++k) {
        double saved;
        if (funs[0][k - 1] == 0) {
            saved = 0;
        } else {
            saved = ((u - knot_vector[0]) * funs[0][k - 1]) / (knot_vector[k] - knot_vector[0]);
        }

        for (uint32_t j = 0; j < degree - k + 1; ++j) {
            auto knot_left = knot_vector[j + 1];
            auto knot_right = knot_vector[j + k + 1];
            if (funs[j + 1][k - 1] == 0) {
                funs[j][k] = saved;
                saved = 0;
            } else {
                auto temp = funs[j + 1][k - 1] / (knot_right - knot_left);
                funs[j][k] = saved + (knot_right - u) * temp;
                saved = (u - knot_left) * temp;
            }
        }
    }

    vec2 out;
    out.x = funs[0][degree];
    out.y = 0;

    // Calc first derivative
    array<double, 2> funs_devs = {funs[0][degree - 1], funs[1][degree - 1]};
    auto first_denom = knot_vector[degree] - knot_vector[0];
    auto second_denom = knot_vector[degree + 1] - knot_vector[1];
    if (first_denom > 0) {
        out.y += funs_devs[0] / first_denom;
    }
    if (second_denom > 0) {
        out.y -= funs_devs[1] / second_denom;
    }
    out.y *= degree;
//    out.y = degree * (
//    (funs_devs[0] / (knot_vector[degree] - knot_vector[0]))
//    -
//    (funs_devs[1] / (knot_vector[degree + 1] - knot_vector[1]))
//    );

    return out;
}

}
