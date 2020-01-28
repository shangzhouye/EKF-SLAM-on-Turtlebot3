// Created by Shangzhou Ye, MSR, Northwestern University

#include <gtest/gtest.h>
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <sstream>
#include <cstring>
#include <string>

TEST(Rigid2dTest, TransformationOperator)
{
    rigid2d::Vector2D translation;
    translation.x = 0;
    translation.y = -1;
    rigid2d::Transform2D trans_test(translation, -(rigid2d::PI / 2.0));

    rigid2d::Vector2D vect_test;
    vect_test.x = 1;
    vect_test.y = 1;

    rigid2d::Vector2D result = trans_test(vect_test);

    ASSERT_DOUBLE_EQ(1.0, result.x);
    ASSERT_DOUBLE_EQ(-2.0, result.y);
}

TEST(Rigid2dTest, InputOutput)
{

    rigid2d::Transform2D trans_test;
    std::ostringstream out;
    std::istringstream in("90 1 4");

    in >> trans_test;
    out << trans_test;

    std::string string_ = out.str();
    const char *str1 = string_.c_str();

    const char *str2 = "degrees:90 dx:1 dy:4 \n";

    ASSERT_STREQ(str1, str2);
}

TEST(Rigid2dTest, Inverse)
{
    rigid2d::Vector2D translation;
    translation.x = 0;
    translation.y = -1;
    rigid2d::Transform2D trans_test(translation, rigid2d::PI * 3.0 / 2.0);

    rigid2d::Transform2D result = trans_test.inv();

    double result_x, result_y, result_theta;
    result.displacement(result_x, result_y, result_theta);

    ASSERT_DOUBLE_EQ(-1.0, result_x);
    ASSERT_NEAR(0.0, result_y, 1.0e-12);
    ASSERT_DOUBLE_EQ(-rigid2d::PI * 3.0 / 2.0, result_theta);
}

TEST(Rigid2dTest, Multiplication)
{
    rigid2d::Vector2D translation1;
    translation1.x = -1;
    translation1.y = 0;
    rigid2d::Transform2D trans_test1(translation1, rigid2d::PI / 2.0);

    rigid2d::Vector2D translation2;
    translation2.x = -1;
    translation2.y = -1;
    rigid2d::Transform2D trans_test2(translation2, rigid2d::PI);

    rigid2d::Transform2D result = trans_test1 * trans_test2;

    double result_x, result_y, result_theta;
    result.displacement(result_x, result_y, result_theta);

    ASSERT_NEAR(0.0, result_x, 1.0e-12);
    ASSERT_DOUBLE_EQ(-1.0, result_y);
    ASSERT_DOUBLE_EQ(rigid2d::normalize_angle(rigid2d::PI * 3.0 / 2.0), result_theta);
}

TEST(Rigid2dTest, integrateTwist)
{
    rigid2d::Twist2D twist;
    twist.omega = 1;
    twist.v_x = 2;
    twist.v_y = 7;
    rigid2d::Transform2D result;
    result = rigid2d::integrateTwist(twist);

    double result_x, result_y, result_theta;
    result.displacement(result_x, result_y, result_theta);

    ASSERT_NEAR(-1.5349, result_x, 1.0e-3);
    ASSERT_NEAR(6.80969, result_y, 1.0e-3);
    ASSERT_DOUBLE_EQ(1, result_theta);
}

TEST(Rigid2dTest, Addition)
{
    rigid2d::Vector2D vec_1;
    vec_1.x = 3;
    vec_1.y = 4;
    rigid2d::Vector2D vec_2;
    vec_2.x = 1;
    vec_2.y = 3;
    rigid2d::Vector2D result;
    result = vec_1 + vec_2;

    ASSERT_DOUBLE_EQ(4, result.x);
    ASSERT_DOUBLE_EQ(7, result.y);
}

TEST(Rigid2dTest, Subtraction)
{
    rigid2d::Vector2D vec_1;
    vec_1.x = 3;
    vec_1.y = 4;
    rigid2d::Vector2D vec_2;
    vec_2.x = 1;
    vec_2.y = 3;
    rigid2d::Vector2D result;
    result = vec_1 - vec_2;

    ASSERT_DOUBLE_EQ(2, result.x);
    ASSERT_DOUBLE_EQ(1, result.y);
}

TEST(Rigid2dTest, InplaceMultiplication)
{
    rigid2d::Vector2D vec_1;
    vec_1.x = 3;
    vec_1.y = 4;
    vec_1 *= 2;

    ASSERT_DOUBLE_EQ(6, vec_1.x);
    ASSERT_DOUBLE_EQ(8, vec_1.y);
}

TEST(Rigid2dTest, VectorMultiplication)
{
    rigid2d::Vector2D vec_1;
    vec_1.x = 3;
    vec_1.y = 4;
    double scalar = 3.0;

    rigid2d::Vector2D result_left;
    result_left = scalar * vec_1;

    rigid2d::Vector2D result_right;
    result_right = vec_1 * scalar;

    ASSERT_DOUBLE_EQ(9.0, result_left.x);
    ASSERT_DOUBLE_EQ(12.0, result_left.y);
    ASSERT_DOUBLE_EQ(9, result_right.x);
    ASSERT_DOUBLE_EQ(12, result_right.y);
}

TEST(Rigid2dTest, VectorLength)
{
    rigid2d::Vector2D vec(3,4);
    double length;
    length = rigid2d::length(vec);

    ASSERT_DOUBLE_EQ(5, length);
}

TEST(Rigid2dTest, VectordDistance)
{
    rigid2d::Vector2D vec1(3,4);
    rigid2d::Vector2D vec2;
    double distance;
    distance = rigid2d::distance(vec1, vec2);

    ASSERT_DOUBLE_EQ(5, distance);
}

TEST(Rigid2dTest, VectorAngle)
{
    rigid2d::Vector2D vec(3,3);
    double angle;
    angle = rigid2d::angle(vec);

    ASSERT_DOUBLE_EQ(rigid2d::deg2rad(45), angle);
}

// google test main function
int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}