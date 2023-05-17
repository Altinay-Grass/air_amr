/**
 * @file amr_nav_client.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef AMR_NAV_CLIENT_HPP
#define AMR_NAV_CLIENT_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <optional>

struct NavPoint
{
    std::string goalPointName;
    double x;
    double y;
    double w;
};

typedef std::vector<NavPoint> NavPoints;

std::optional<NavPoints> getPointsFromConfigFile(const std::string& path_to_config_file);

#endif // AMR_NAV_CLIENT_HPP