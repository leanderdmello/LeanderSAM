// Copyright 2024 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <gtest/gtest.h>

#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  auto test_result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return test_result;
}