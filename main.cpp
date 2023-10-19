#include <iostream>
#include "PointCloud.hpp"

int main(int argc, char *argv[])
try
{
    std::cout<<"aaa"<<std::endl;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}