#include <iostream>
#include "PointCloud.hpp"

int main(int argc, char *argv[])
try
{
    std::string name = "pcd/frontfilterTestalpha0.25.pcd";
    if (argc >= 2)
    {
        name = argv[1];
    }
    PointCloud p;
    p.load_pcd(name);
    p.searchPlane();
    p.pointsToPlaneDistance();
    p.save_pcd("pcd/searchPlaneFront");
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}