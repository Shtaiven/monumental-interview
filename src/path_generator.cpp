#include "path_generator.h"

#include <cmath>

namespace path_generator {

Vec2 eval(double t) {
    double k = 1.5 * M_PI;
    if (t < 20) {
        k = M_PI * (t / 10.0 - 0.5);
    }
    return Vec2{.x = -2 * std::sin(k) * std::cos(k),
                .y = 2 * (std::sin(k) + 1)};
}

}  // namespace path_generator
