/// \file
/// \brief Library for two-dimensional rigid body transformations.
#include "rigid2d.hpp"
#include <iostream>
#include <string>
#include <sstream>

namespace rigid2d
{

std::ostream &operator<<(std::ostream &os, const Vector2D &v)
{
    os << "[" << v.x << " " << v.y << "]" << std::endl;
    return os;
}

std::istream &operator>>(std::istream &is, Vector2D &v)
{
    std::string string_x, string_y;
    is >> string_x >> string_y;

    // erase or pop the "[" character
    string_x.erase(string_x.begin());
    string_y.pop_back();

    // convert string to double
    std::stringstream ss_x(string_x);
    ss_x >> v.x;
    std::stringstream ss_y(string_y);
    ss_y >> v.y;

    return is;
}

} // namespace rigid2d