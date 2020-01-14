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
    os << "[" << tf.trans_.x << " " << tf.trans_.y << " " << tf.radians_ << "]" << std::endl;
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

std::istream &operator>>(std::istream &is, Transform2D &tf)
{
    std::string string_x, string_y, string_theta;
    is >> string_x >> string_y >> string_theta;

    // erase or pop the "[" character
    string_x.erase(string_x.begin());
    string_theta.pop_back();

    // convert string to double
    std::stringstream ss_x(string_x);
    ss_x >> tf.trans_.x;
    std::stringstream ss_y(string_y);
    ss_y >> tf.trans_.y;
    std::stringstream ss_theta(string_theta);
    ss_theta >> tf.radians_;

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
    lhs*=rhs;
    return lhs;
}

} // namespace rigid2d