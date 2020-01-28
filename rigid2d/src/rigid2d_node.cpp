/// \file
/// \brief Rigid transformations using rigid2d library.
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include "ros/ros.h"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/waypoints.hpp"

int main(int argc, char **argv)
{

    /* Start: Testing code ******************************
    // rigid2d::Vector2D vect;
    // std::cin >> vect;
    // std::cout << vect;
    // rigid2d::Vector2D vect;
    // vect.x = 1;
    // vect.y = 0;
    // rigid2d::Transform2D test_trans(3.1415926/2);
    // test_trans = test_trans.inv();
    // std::cout << test_trans(vect);
    // std::cout << test_trans;
    // rigid2d::Transform2D in_trans;
    // std::cin >> in_trans;
    // std::cout << in_trans;
    // End: Testing code ******************************
    */

    // test waypoints class
    rigid2d::Waypoints my_waypoints;
    my_waypoints.pipeline(600);
    std::cout << "Waypoints tested above;" << std::endl;

    // init the node
    ros::init(argc, argv, "node");
    ros::NodeHandle nh;

    // input two transforms
    std::cout << "Enter transform Tab: degrees, dx, dy:" << std::endl;
    rigid2d::Transform2D T_ab, T_bc, T_ba, T_cb, T_ac, T_ca;
    std::cin >> T_ab;
    std::cout << "Enter transform Tbc: degrees, dx, dy:" << std::endl;
    std::cin >> T_bc;

    std::cout << "Tab:" << std::endl;
    std::cout << T_ab;

    T_ba = T_ab.inv();
    std::cout << "Tba:" << std::endl;
    std::cout << T_ba;

    std::cout << "Tbc:" << std::endl;
    std::cout << T_bc;

    T_cb = T_bc.inv();
    std::cout << "Tcb:" << std::endl;
    std::cout << T_cb;

    T_ac = T_ab * T_bc;
    std::cout << "Tac:" << std::endl;
    std::cout << T_ac;

    T_ca = T_ac.inv();
    std::cout << "Tca:" << std::endl;
    std::cout << T_ca;

    rigid2d::Vector2D vect;
    std::cout << "Enter a vector [x y]:" << std::endl;
    std::cin >> vect;
    std::cout << "Which frame it is defined in:" << std::endl;
    char frame_id;
    std::cin >> frame_id;
    std::cout << "Enter a twist (in the same frame) [omega v_x v_y]:" << std::endl;
    rigid2d::Twist2D twi;
    std::cin >> twi;
    if (frame_id == 97)
    {
        std::cout << "(First line is vector, second line is twist)" << std::endl;
        std::cout << "In frame a:" << std::endl;
        std::cout << vect;
        std::cout << twi;
        std::cout << "In frame b:" << std::endl;
        std::cout << T_ba(vect);
        std::cout << T_ba(twi);
        std::cout << "In frame c:" << std::endl;
        std::cout << T_ca(vect);
        std::cout << T_ca(twi);
    }
    else if (frame_id == 98)
    {
        std::cout << "(First line is vector, second line is twist)" << std::endl;
        std::cout << "In frame a:" << std::endl;
        std::cout << T_ab(vect);
        std::cout << T_ab(twi);
        std::cout << "In frame b:" << std::endl;
        std::cout << vect;
        std::cout << twi;
        std::cout << "In frame c:" << std::endl;
        std::cout << T_cb(vect);
        std::cout << T_cb(twi);
    }
    else if (frame_id == 99)
    {
        std::cout << "(First line is vector, second line is twist)" << std::endl;
        std::cout << "In frame a:" << std::endl;
        std::cout << T_ac(vect);
        std::cout << T_ac(twi);
        std::cout << "In frame b:" << std::endl;
        std::cout << T_bc(vect);
        std::cout << T_bc(twi);
        std::cout << "In frame c:" << std::endl;
        std::cout << vect;
        std::cout << twi;
    }
    else
    {
        std::cout << "Input is invalid." << std::endl;
    }

    return 0;
}