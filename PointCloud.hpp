#pragma once

#include <iostream>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloud
{
public:
    using pc_rgb = pcl::PointCloud<pcl::PointXYZRGB>;
    using pc_rgb_ptr = pc_rgb::Ptr;
    PointCloud();

    void load_pcd(const std::string &);
    void save_pcd(const std::string &) const;
    pc_rgb_ptr get_cloud() const { return cloud; }
    void filter(void (*func)(pcl::PointXYZRGB &));
    void filter(pcl::PointXYZRGB &(*func)(const pcl::PointXYZRGB &));
    PointCloud extended(const PointCloud &);

private:
    pc_rgb_ptr cloud;
};