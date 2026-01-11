#pragma once

#include <array>

namespace ocs2 {

enum class Color { blue, orange, yellow, purple, green, red, black };

std::array<double, 3> getRGB(Color color);

}  // namespace ocs2
