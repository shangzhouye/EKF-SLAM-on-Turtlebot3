#include "rigid2d.hpp"
#include <iostream>

int main()
{
    // Start: Testing code
    // rigid2d::Vector2D vect;
    // std::cin >> vect;
    // std::cout << vect;
    rigid2d::Vector2D vect;
    vect.x = 1;
    vect.y = 0;
    rigid2d::Transform2D test_trans(3.1415926/2);
    test_trans = test_trans.inv();
    std::cout << test_trans(vect);
    std::cout << test_trans;
    rigid2d::Transform2D in_trans;
    std::cin >> in_trans;
    std::cout << in_trans;
    // End: Testing code
    return 0;
}