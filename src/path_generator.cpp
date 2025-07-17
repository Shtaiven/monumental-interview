#include "path_generator.h"

#include <cmath>

namespace path_generator {

Point eval(double t) {
    double k = 1.5 * M_PI;
    if (t < 20) {
        k = M_PI * (t / 10.0 - 0.5);
    }
    return Point{.x = -2 * std::sin(k) * std::cos(k),
                 .y = 2 * (std::sin(k) + 1)};
}

}  // namespace path_generator
