/// \file
/// \brief Rigid transformations using rigid2d library.
#include "rigid2d.hpp"
#include <iostream>

int main()
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

    // input two transforms
    std::cout << "Enter transform Tab:" << std::endl;
    rigid2d::Transform2D T_ab, T_bc, T_ba, T_cb, T_ac, T_ca;
    std::cin >> T_ab;
    std::cout << "Enter transform Tbc:" << std::endl;
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
    std::cout << "Enter a vector:" << std::endl;
    std::cin >> vect;
    std::cout << "Which frame it is defined in:" << std::endl;
    char frame_id;
    std::cin >> frame_id;
    if (frame_id == 97)
    {
        std::cout << "In frame a:" << std::endl;
        std::cout << vect;
        std::cout << "In frame b:" << std::endl;
        std::cout << T_ba(vect);
        std::cout << "In frame c:" << std::endl;
        std::cout << T_ca(vect);
    }
    else if (frame_id == 98)
    {
        std::cout << "In frame a:" << std::endl;
        std::cout << T_ab(vect);
        std::cout << "In frame b:" << std::endl;
        std::cout << vect;
        std::cout << "In frame c:" << std::endl;
        std::cout << T_cb(vect);
    }
    else if (frame_id == 99)
    {
        std::cout << "In frame a:" << std::endl;
        std::cout << T_ac(vect);
        std::cout << "In frame b:" << std::endl;
        std::cout << T_bc(vect);
        std::cout << "In frame c:" << std::endl;
        std::cout << vect;
    }
    else
    {
        std::cout << "Input is invalid." << std::endl;
    }

    return 0;
}