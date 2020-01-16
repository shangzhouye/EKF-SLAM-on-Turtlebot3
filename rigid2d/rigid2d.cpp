/// \file
/// \brief Library for two-dimensional rigid body transformations.
#include "rigid2d.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

namespace rigid2d
{

std::ostream &operator<<(std::ostream &os, const Vector2D &v)
{
    os << "[" << v.x << " " << v.y << "]" << std::endl;
    return os;
}

std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
{
    os << "degrees:" << rad2deg(tf.radians_) << " "
       << "dx:" << tf.trans_.x << " "
       << "dy:" << tf.trans_.y << " " << std::endl;
    return os;
}

std::ostream &operator<<(std::ostream &os, const Twist2D &t)
{
    os << "[" << t.omega << " " << t.v_x << " " << t.v_y << "]" << std::endl;
    return os;
}

std::istream &operator>>(std::istream &is, Vector2D &v)
{
    std::string string_x, string_y;
    is >> string_x >> string_y;

    // if in the [x, y] format, erase or pop the "[" character
    if (string_x.substr(0, 1) == "[")
    {
        string_x.erase(string_x.begin());
        string_y.pop_back();
    }

    // convert string to double
    std::stringstream ss_x(string_x);
    ss_x >> v.x;
    std::stringstream ss_y(string_y);
    ss_y >> v.y;

    std::cout << v.x << " " << v.y << std::endl;

    return is;
}

std::istream &operator>>(std::istream &is, Transform2D &tf)
{
    std::string string_theta, string_x, string_y;
    is >> string_theta >> string_x >> string_y;

    // check which format is the input
    if (string_theta.substr(0, 1) == "d")
    {
        string_theta.erase(0, 8);
        string_x.erase(0, 3);
        string_y.erase(0, 3);
    }

    // convert string to double
    std::stringstream ss_theta(string_theta);
    // convert degrees to radians
    double degrees_temp;
    ss_theta >> degrees_temp;
    tf.radians_ = deg2rad(degrees_temp);
    std::stringstream ss_x(string_x);
    ss_x >> tf.trans_.x;
    std::stringstream ss_y(string_y);
    ss_y >> tf.trans_.y;

    return is;
}

std::istream &operator>>(std::istream &is, Twist2D &t)
{
    std::string string_omega, string_v_x, string_v_y;
    is >> string_omega >> string_v_x >> string_v_y;

    // erase or pop the "[" character
    string_omega.erase(string_omega.begin());
    string_v_y.pop_back();

    // convert string to double
    std::stringstream ss_omega(string_omega);
    ss_omega >> t.omega;
    std::stringstream ss_v_x(string_v_x);
    ss_v_x >> t.v_x;
    std::stringstream ss_v_y(string_v_y);
    ss_v_y >> t.v_y;

    return is;
}

Transform2D::Transform2D()
{
    trans_.x = 0;
    trans_.y = 0;
    radians_ = 0;
}

Transform2D::Transform2D(const Vector2D &trans)
{
    trans_.x = trans.x;
    trans_.y = trans.y;
    radians_ = 0;
}

Transform2D::Transform2D(double radians)
{
    trans_.x = 0;
    trans_.y = 0;
    radians_ = radians;
}

Transform2D::Transform2D(const Vector2D &trans, double radians)
{
    trans_.x = trans.x;
    trans_.y = trans.y;
    radians_ = radians;
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    Vector2D result;
    result.x = v.x * std::cos(this->radians_) - v.y * std::sin(this->radians_) + this->trans_.x;
    result.y = v.x * std::sin(this->radians_) + v.y * std::cos(this->radians_) + this->trans_.y;
    return result;
}

Twist2D Transform2D::operator()(Twist2D t) const
{
    Twist2D result;
    result.omega = t.omega;
    result.v_x = this->trans_.y * t.omega + std::cos(this->radians_) * t.v_x - std::sin(this->radians_) * t.v_y;
    result.v_y = -this->trans_.x * t.omega + std::sin(this->radians_) * t.v_x + std::cos(this->radians_) * t.v_y;
    return result;
}

Transform2D Transform2D::inv() const
{
    double inv_radians = -this->radians_;
    Vector2D inv_trans;
    inv_trans.x = -this->trans_.x * std::cos(this->radians_) - this->trans_.y * std::sin(this->radians_);
    inv_trans.y = this->trans_.x * std::sin(this->radians_) - this->trans_.y * std::cos(this->radians_);
    Transform2D inv_result(inv_trans, inv_radians);
    return inv_result;
}

Transform2D &Transform2D::operator*=(const Transform2D &rhs)
{
    this->radians_ = this->radians_ + rhs.radians_;
    this->trans_.x = rhs.trans_.x * std::cos(this->radians_) - rhs.trans_.y * std::sin(this->radians_) + this->trans_.x;
    this->trans_.y = rhs.trans_.x * std::sin(this->radians_) + rhs.trans_.y * std::cos(this->radians_) + this->trans_.y;
    return *this;
}

Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
{
    lhs *= rhs;
    return lhs;
}

} // namespace rigid2d