#include "julia_imu_bridge.hpp"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <gtsam/geometry/Point3.h>
#include <iomanip>

bool writeImuPacketJson(const std::string& path, const std::string& json_content)
{
    std::ofstream out(path);
    if (!out.is_open()) return false;
    out << json_content;
    return true;
}

bool runJuliaImuInference(const std::string& input_json, const std::string& output_json)
{
    std::stringstream cmd;
    cmd << "/workspaces/telerob-main/julia/run_imu_infer.sh "
        << input_json << " "
        << output_json;

    int ret = std::system(cmd.str().c_str());
    return ret == 0;
}

static bool extractArray(const std::string& content, const std::string& key, std::vector<double>& values)
{
    values.clear();

    const std::string token = "\"" + key + "\"";
    std::size_t key_pos = content.find(token);
    if (key_pos == std::string::npos) return false;

    std::size_t start = content.find('[', key_pos);
    if (start == std::string::npos) return false;

    std::size_t end = content.find(']', start);
    if (end == std::string::npos) return false;

    std::string array_text = content.substr(start + 1, end - start - 1);
    std::stringstream ss(array_text);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        std::stringstream item_stream(item);
        double value;
        item_stream >> value;
        if (!item_stream.fail())
        {
            values.push_back(value);
        }
    }

    return !values.empty();
}

bool readJuliaImuResult(const std::string& path, JuliaImuResult& result)
{
    std::ifstream in(path);
    if (!in.is_open()) return false;

    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string content = buffer.str();

    bool ok_mean = extractArray(content, "mean", result.mean);
    bool ok_cov = extractArray(content, "covariance", result.covariance);

    return ok_mean && ok_cov;
}

bool runJuliaImuFromPacketString(const std::string& packet_json, JuliaImuResult& result)
{
    const std::string input_path = "/tmp/liorf_imu_packet.json";
    const std::string output_path = "/tmp/liorf_imu_result.json";

    if (!writeImuPacketJson(input_path, packet_json))
    {
        return false;
    }

    if (!runJuliaImuInference(input_path, output_path))
    {
        return false;
    }

    if (!readJuliaImuResult(output_path, result))
    {
        return false;
    }

    return true;
}

std::string buildSimplePositionPacket(
    const gtsam::Point3& prev_position,
    const gtsam::Point3& predicted_position,
    const gtsam::Point3& observed_position)
{
    const double cx = predicted_position.x() - prev_position.x();
    const double cy = predicted_position.y() - prev_position.y();
    const double cz = predicted_position.z() - prev_position.z();

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(9);
    ss << "{\n";
    ss << "  \"n\": 2,\n";
    ss << "  \"state_dim\": 3,\n";
    ss << "  \"obs_dim\": 3,\n";
    ss << "  \"m0\": [" << prev_position.x() << ", " << prev_position.y() << ", " << prev_position.z() << "],\n";
    ss << "  \"P0\": [\n";
    ss << "    1.0, 0.0, 0.0,\n";
    ss << "    0.0, 1.0, 0.0,\n";
    ss << "    0.0, 0.0, 1.0\n";
    ss << "  ],\n";
    ss << "  \"F\": [\n";
    ss << "    [\n";
    ss << "      1.0, 0.0, 0.0,\n";
    ss << "      0.0, 1.0, 0.0,\n";
    ss << "      0.0, 0.0, 1.0\n";
    ss << "    ]\n";
    ss << "  ],\n";
    ss << "  \"c\": [\n";
    ss << "    [" << cx << ", " << cy << ", " << cz << "]\n";
    ss << "  ],\n";
    ss << "  \"Q\": [\n";
    ss << "    [\n";
    ss << "      0.05, 0.0, 0.0,\n";
    ss << "      0.0, 0.05, 0.0,\n";
    ss << "      0.0, 0.0, 0.05\n";
    ss << "    ]\n";
    ss << "  ],\n";
    ss << "  \"H\": [\n";
    ss << "    [\n";
    ss << "      1.0, 0.0, 0.0,\n";
    ss << "      0.0, 1.0, 0.0,\n";
    ss << "      0.0, 0.0, 1.0\n";
    ss << "    ],\n";
    ss << "    [\n";
    ss << "      1.0, 0.0, 0.0,\n";
    ss << "      0.0, 1.0, 0.0,\n";
    ss << "      0.0, 0.0, 1.0\n";
    ss << "    ]\n";
    ss << "  ],\n";
    ss << "  \"d\": [\n";
    ss << "    [0.0, 0.0, 0.0],\n";
    ss << "    [0.0, 0.0, 0.0]\n";
    ss << "  ],\n";
    ss << "  \"R\": [\n";
    ss << "    [\n";
    ss << "      0.1, 0.0, 0.0,\n";
    ss << "      0.0, 0.1, 0.0,\n";
    ss << "      0.0, 0.0, 0.1\n";
    ss << "    ],\n";
    ss << "    [\n";
    ss << "      0.1, 0.0, 0.0,\n";
    ss << "      0.0, 0.1, 0.0,\n";
    ss << "      0.0, 0.0, 0.1\n";
    ss << "    ]\n";
    ss << "  ],\n";
    ss << "  \"y\": [\n";
    ss << "    [" << prev_position.x() << ", " << prev_position.y() << ", " << prev_position.z() << "],\n";
    ss << "    [" << observed_position.x() << ", " << observed_position.y() << ", " << observed_position.z() << "]\n";
    ss << "  ]\n";
    ss << "}\n";

    return ss.str();
}