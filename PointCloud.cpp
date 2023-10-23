#include "PointCloud.hpp"
PointCloud::PointCloud()
{
    cloud = pc_rgb_ptr(new pc_rgb);
}

void PointCloud::load_pcd(const std::string &filename)
{
    pcl::io::loadPCDFile(filename, *cloud);
}

void PointCloud::searchPlane()
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // RANSACによる検出．
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);     // 外れ値の存在を前提とし最適化を行う
    seg.setModelType(pcl::SACMODEL_PLANE); // モードを平面検出に設定
    seg.setMethodType(pcl::SAC_RANSAC);    // 検出方法をRANSACに設定
    seg.setDistanceThreshold(0.01);        // しきい値を設定

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pc_rgb_ptr filtered = cloud;

    while (true)
    {
        seg.setInputCloud(filtered);          // 入力点群をセット
        seg.segment(*inliers, *coefficients); // 検出を行う

        if (inliers->indices.size() <= 0)
        {
            break;
        }

        pc_rgb points;
        for (size_t i = 0; i < inliers->indices.size(); ++i)
        {
            auto p = (*filtered)[inliers->indices[i]];
            points.push_back(p);
        }
        points_list.push_back(points);

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*filtered);
    }
}

void PointCloud::save_pcd(const std::string &n) const
{
    for (int i = 0; i < points_list.size(); i++)
    {
        auto points = points_list[i];
        std::string name = n + std::to_string(i);

        if (name.size() <= 4 || name.substr(name.size() - 4) != ".pcd")
        {
            name = name + std::string(".pcd");
        }

        // pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
        // Cloud_Filter.setInputCloud(cloud);               // Input generated cloud to filter
        // Cloud_Filter.setFilterFieldName("z");            // Set field name to Z-coordinate
        // Cloud_Filter.setFilterLimits(-3.0, 3.0);         // Set accepted interval values
        // Cloud_Filter.filter(*cloud);                     // Filtered Cloud Outputted

        // pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter2; // Create the filtering object
        // Cloud_Filter2.setInputCloud(cloud);               // Input generated cloud to filter
        // Cloud_Filter2.setFilterFieldName("x");            // Set field name to Z-coordinate
        // Cloud_Filter2.setFilterLimits(-3.0, 3.0);         // Set accepted interval values
        // Cloud_Filter2.filter(*cloud);                     // Filtered Cloud Outputted

        pcl::io::savePCDFileBinary(name, *cloud);
    }
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