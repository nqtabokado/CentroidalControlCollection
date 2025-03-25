#pragma once

#ifdef CCC_STANDALONE
#  include <iostream>
#  define CCC_ERROR_STREAM(x) std::cerr << x << "\n"
#  define CCC_WARN_STREAM(x) std::cerr << x << "\n"
#  define CCC_INFO_STREAM(x) std::cout << x << "\n"
#else
#  include <rclcpp/rclcpp.hpp>
#  define CCC_ERROR_STREAM(x) RCLCPP_ERROR(rclcpp::get_logger("CCC"), "%s", (x).c_str())
#  define CCC_WARN_STREAM(x) RCLCPP_WARN(rclcpp::get_logger("CCC"), "%s", std::string(x).c_str())
#  define CCC_INFO_STREAM(x) RCLCPP_INFO(rclcpp::get_logger("CCC"), "%s", (x).c_str())
#endif
