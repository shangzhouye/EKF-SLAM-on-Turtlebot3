#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

/**************************
 * Format of input and output
 * 
 * Vector2D
 * - input: entered as two numbers separated by a newline or a space, or entered as [xcomponent ycomponent]
 * - output: [xcomponent ycomponent]
 * 
 * Twist2D
 * - input: [omega v_x v_y]
 * - output: [omega v_x v_y]
 * 
 * Transform2D
 * (When contructed by constructor, rotation has to be in radians)
 * - input: either as output by operator<< or as degrees, dx, dy separated by spaces or newlines
 * - output: degrees:90 dx:3 dy:5
 * 
 **************************/

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <iostream>

namespace rigid2d
{
/// \brief PI.  Not in C++ standard until C++20.
constexpr double PI = 3.14159265358979323846;

/// \brief approximately compare two floating-point numbers using
///        an absolute comparison
/// \param d1 - a number to compare
/// \param d2 - a second number to compare
/// \param epsilon - absolute threshold required for equality
/// \return true if abs(d1 - d2) < epsilon
/// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
/// be useful here
constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
{
    return std::abs(d1 - d2) < epsilon;
}

/// \brief convert degrees to radians
/// \param deg - angle in degrees
/// \returns radians
/// NOTE: implement this in the header file
/// constexpr means that the function can be computed at compile time
/// if given a compile-time constant as input
constexpr double deg2rad(double deg)
{
    return deg * (PI / 180);
}

/// \brief convert radians to degrees
/// \param rad - angle in radians
/// \returns the angle in degrees
constexpr double rad2deg(double rad)
{
    return rad * (180 / PI);
}

/// static_assertions test compile time assumptions.
/// You should write at least one more test for each function
/// You should also purposely (and temporarily) make one of these tests fail
/// just to see what happens
static_assert(almost_equal(0, 0), "is_zero failed");
static_assert(!almost_equal(0, 0.01), "is_zero failed");

static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
static_assert(almost_equal(0.001, 0.0015, 1.0e-3), "is_zero failed");

static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
static_assert(almost_equal(deg2rad(30), 0.523599, 1.0e-3), "deg2rad failed");

static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
static_assert(almost_equal(rad2deg(15), 859.437, 1.0e-3), "rad2deg failed");

static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

/// \brief angle wraping
/// \param rad - angle in radians
/// \returns the wrapped angle in radians
double normalize_angle(double rad);

/// \brief A 2-Dimensional Vector
struct Vector2D
{
public:
    double x;
    double y;

    Vector2D();

    Vector2D(double input_x, double input_y);

    /// \brief vector addition in place
    /// \param rhs - the vector to be added
    /// \return the added vector
    Vector2D &operator+=(const Vector2D &rhs);

    /// \brief vector subtraction in place
    /// \param rhs - the vector to be subtracted
    /// \return the subtracted vector
    Vector2D &operator-=(const Vector2D &rhs);

    /// \brief scalar multiplication
    /// \param rhs - the scalar
    /// \return the result vector
    Vector2D &operator*=(const double &rhs);
};

/// \brief calculate the length of the vector
/// \param v - the vector
/// \return the length
double length(const Vector2D &v);

/// \brief calculate the distance between two vectors
/// \param v1 - vector 1
/// \param v2 - vector 2
/// \return the distance
double distance(const Vector2D &v1, const Vector2D &v2);

/// \brief calculate the angle of the vector
/// \param v - the vector
/// \return the angle
double angle(const Vector2D &v);

/// \brief scalar multiplication (scalar at left)
/// \param lhs - the left hand
/// \param rhs - the right hand
/// \return the result vector
Vector2D operator*(double lhs, Vector2D rhs);

/// \brief scalar multiplication (scalar at right)
/// \param lhs - the left hand
/// \param rhs - the right hand
/// \return the result vector
Vector2D operator*(Vector2D lhs, double rhs);

/// \brief vector addition
/// \param lhs - the left hand
/// \param rhs - the right hand
/// \return the added vector
Vector2D operator+(Vector2D lhs, const Vector2D &rhs);

/// \brief vector subtraction
/// \param lhs - the left hand
/// \param rhs - the right hand
/// \return the subtracted vector
Vector2D operator-(Vector2D lhs, const Vector2D &rhs);

/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
/// os - stream to output to
/// v - the vector to print
std::ostream &operator<<(std::ostream &os, const Vector2D &v);

/// \brief input a 2 dimensional vector
///   You should be able to read vectors entered as two numbers
///   separated by a newline or a space, or entered as [xcomponent ycomponent]
/// is - stream from which to read
/// v [out] - output vector
/// Hint: The following may be useful:
/// https://en.cppreference.com/w/cpp/io/basic_istream/peek
/// https://en.cppreference.com/w/cpp/io/basic_istream/get
std::istream &operator>>(std::istream &is, Vector2D &v);

/// \brief normalize Vector2D
Vector2D normalize_vector(Vector2D vector_input);

/// \brief A 2-Dimensional Twist
struct Twist2D
{
public:
    double omega;
    double v_x;
    double v_y;

    // learned: default constructor is needed for all the classes when using gcc
    Twist2D();

    Twist2D(double init_omega, double init_v_x, double init_v_y);
};

/// \brief output a 2 dimensional twist as [omega v_x v_y]
/// os - stream to output to
/// t - the twist to print
std::ostream &operator<<(std::ostream &os, const Twist2D &t);

/// \brief input a 2 dimensional twist
/// entered as [omega v_x v_y]
/// is - stream from which to read
/// t [out] - output twist
std::istream &operator>>(std::istream &is, Twist2D &t);

/// \brief a rigid body transformation in 2 dimensions
class Transform2D
{
public:
    /// \brief Create an identity transformation
    Transform2D();

    /// \brief create a transformation that is a pure translation
    /// \param trans - the vector by which to translate
    explicit Transform2D(const Vector2D &trans);

    /// \brief create a pure rotation
    /// \param radians - angle of the rotation, in radians
    explicit Transform2D(double radians);

    /// \brief Create a transformation with a translational and rotational
    /// component
    /// \param trans - the translation
    /// \param rot - the rotation, in radians
    Transform2D(const Vector2D &trans, double radians);

    /// \brief apply a transformation to a Vector2D
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D operator()(Vector2D v) const;

    /// \brief apply a transformation to a Twist2D
    /// \param t - the twist to transform
    /// \return a twist in the new coordinate system
    Twist2D operator()(Twist2D t) const;

    /// \brief invert the transformation
    /// \return the inverse transformation.
    Transform2D inv() const;

    /// \brief compose this transform with another and store the result
    /// in this object
    /// \param rhs - the first transform to apply
    /// \returns a reference to the newly transformed operator
    Transform2D &operator*=(const Transform2D &rhs);

    /// \brief returns the x, y, and theta values from the transform
    /// \param &x, &y, &theta - reference for return multiple values
    /// \returns the x, y, and theta values from the transform
    int displacement(double &x, double &y, double &theta);

    /// \brief \see operator<<(...) (declared outside this class)
    /// for a description
    friend std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

    /// \brief \see operator>>(...) (declared outside this class)
    /// for a description
    friend std::istream &operator>>(std::istream &is, Transform2D &tf);

private:
    Vector2D trans_;
    double radians_;
};

/// \brief should print a human readable version of the transform:
/// An example output:
/// dtheta (degrees): 90 dx: 3 dy: 5
/// \param os - an output stream
/// \param tf - the transform to print
std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

/// \brief Read a transformation from stdin
/// Should be able to read input either as output by operator<< or
/// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
std::istream &operator>>(std::istream &is, Transform2D &tf);

/// \brief multiply two transforms together, returning their composition
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the composition of the two transforms
/// HINT: This function can be implemented in terms of *=
Transform2D operator*(Transform2D lhs, const Transform2D &rhs);

/// \brief compute the transformation corresponding to a rigid body following a constant twist for one time unit
/// \param twist - the constant twist
/// \return the corresponding transformation
Transform2D integrateTwist(Twist2D twist);

} // namespace rigid2d

#endif
