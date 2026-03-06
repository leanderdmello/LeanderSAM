// Copyright 2025 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <common_lib.h>
namespace CommonLib
{
template float common_lib::pointDistance<pcl::PointXYZI>(const pcl::PointXYZI &p);
template float common_lib::pointDistance<pcl::PointXYZINormal>(const pcl::PointXYZINormal &p);

template float common_lib::pointDistance<pcl::PointXYZI>(const pcl::PointXYZI &p1,
                                                         const pcl::PointXYZI &p2);
template float common_lib::pointDistance<pcl::PointXYZINormal>(const pcl::PointXYZINormal &p1,
                                                               const pcl::PointXYZINormal &p2);

common_lib::common_lib(const std::string &pkg_mode_) {}

common_lib::~common_lib() {}

template <typename T>
float common_lib::pointDistance(const T &p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename T>
float common_lib::pointDistance(const T &p1, const T &p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

template <typename T>
bool common_lib::remapping(const T &value, const T &graph)
{
    std::cout << "waiting ........" << std::endl;
    return -1;
}
} // namespace CommonLib