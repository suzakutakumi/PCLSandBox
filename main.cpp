#include <iostream>
#include "PointCloud.hpp"

int main(int argc, char *argv[])
try
{
    PointCloud p;
    p.load_pcd("pcd/frontfilterTestalpha0.25.pcd");
    p.searchPlane();
    p.save_pcd("pcd/searchPlane.pcd");
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}