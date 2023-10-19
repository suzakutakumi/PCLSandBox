#include "PointCloud.hpp"
PointCloud::PointCloud() {
    cloud=pc_rgb_ptr(new pc_rgb);
}

void PointCloud::load_pcd(const std::string &filename)
{
    pcl::io::loadPCDFile(filename, *cloud);
}

void PointCloud::save_pcd(const std::string &n) const
{
    std::string name = n;

    if (name.size() <= 4 || name.substr(name.size() - 4) != ".pcd")
    {
        name = name + std::string(".pcd");
    }

    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
    Cloud_Filter.setInputCloud(cloud);               // Input generated cloud to filter
    Cloud_Filter.setFilterFieldName("z");            // Set field name to Z-coordinate
    Cloud_Filter.setFilterLimits(-3.0, 3.0);         // Set accepted interval values
    Cloud_Filter.filter(*cloud);                     // Filtered Cloud Outputted

    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter2; // Create the filtering object
    Cloud_Filter2.setInputCloud(cloud);               // Input generated cloud to filter
    Cloud_Filter2.setFilterFieldName("x");            // Set field name to Z-coordinate
    Cloud_Filter2.setFilterLimits(-3.0, 3.0);         // Set accepted interval values
    Cloud_Filter2.filter(*cloud);                     // Filtered Cloud Outputted

    pcl::io::savePCDFileBinary(name, *cloud);
}

void PointCloud::filter(void (*filter_func)(pcl::PointXYZRGB &))
{
    for (auto &p : cloud->points)
    {
        filter_func(p);
    }
}

void PointCloud::filter(pcl::PointXYZRGB &(*filter_func)(const pcl::PointXYZRGB &))
{
    for (auto &p : cloud->points)
    {
        p = filter_func(p);
    }
}

PointCloud PointCloud::extended(const PointCloud &other)
{
    auto &p1 = *cloud;
    auto p2 = other.get_cloud();
    p1.insert(p1.end(), p2->begin(), p2->end());
    return *this;
}