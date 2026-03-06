// Copyright 2025 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#ifndef _COMMON_LIB_H_
#define _COMMON_LIB_H_
#include <iostream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace CommonLib
{
class common_lib
{
public:
    common_lib(const std::string &pkg_mode_);
    ~common_lib();

    template <typename T>
    float pointDistance(const T &p);

    template <typename T>
    float pointDistance(const T &p1, const T &p2);

    template <typename T>
    bool remapping(const T &value, const T &graph);
};
} // namespace CommonLib

#endif