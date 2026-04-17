#pragma once

#include <string>
#include <vector>
#include <gtsam/geometry/Point3.h>

struct JuliaImuResult
{
    std::vector<double> mean;
    std::vector<double> covariance;
};

bool writeImuPacketJson(const std::string& path, const std::string& json_content);
bool runJuliaImuInference(const std::string& input_json, const std::string& output_json);
bool readJuliaImuResult(const std::string& path, JuliaImuResult& result);
bool runJuliaImuFromPacketString(const std::string& packet_json, JuliaImuResult& result);

std::string buildSimplePositionPacket(
    const gtsam::Point3& prev_position,
    const gtsam::Point3& predicted_position,
    const gtsam::Point3& observed_position);