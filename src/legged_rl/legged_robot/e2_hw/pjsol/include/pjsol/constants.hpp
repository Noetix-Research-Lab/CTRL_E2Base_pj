#pragma once

namespace pjsol {
namespace math {

inline constexpr double kPi = 3.14159265358979323846;

inline constexpr double deg2rad(double deg) {
    return deg * (kPi / 180.0);
}

inline constexpr double rad2deg(double rad) {
    return rad * (180.0 / kPi);
}

}  // namespace math
}  // namespace pjsol
