// Copyright 2025 Avular Holding B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include "stb_wrapper.h"

/*
By defining STB_IMAGE_IMPLEMENTATION the preprocessor modifies the header file
such that it only contains the relevant definition source code, effectively
turning the header file into a .cpp file
*/

#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif // !STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#ifndef STBI_MSC_SECURE_CRT
#define STBI_MSC_SECURE_CRT
#endif // !STBI_MSC_SECURE_CRT

#ifndef STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#endif // !STB_IMAGE_WRITE_IMPLEMENTATION

#include <stb_image_write.h>

#include <iostream>

void StbImage::FlipVertical(std::vector<unsigned char> *data, int width, int height, int channels) {
    stbi__vertical_flip(data->data(), width, height, channels);
}

int StbImage::Load(const rclcpp::Logger &logger, const std::string &path, std::vector<unsigned char> *data, int *width, int *height, int *channels) {
    unsigned char* tmp = stbi_load(path.c_str(), width, height, channels, 0);
    if (!tmp) {
        RCLCPP_ERROR(logger, "Failed to load resource: %s", path.c_str());
        return -1;
    }

    data->resize(*width * *height * *channels);
    data->assign(tmp, tmp + data->size());

    // no longer needed
    stbi_image_free(tmp);

    return 0;
}

void StbImage::Save(const std::string &path, std::vector<unsigned char> *data, int width, int height, int channels) {
    stbi_write_png(path.c_str(), width, height, channels, data->data(), width * channels);
}