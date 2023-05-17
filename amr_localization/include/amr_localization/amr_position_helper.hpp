/**
 * @file amr_position_helper.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AMR_POSITION_HELPER_HPP
#define AMR_POSITION_HELPER_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <yaml-cpp/yaml.h>
#include <memory>

typedef geometry_msgs::PoseWithCovarianceStamped Position;

class PositionHelper
{
    public:

    PositionHelper();

    ~PositionHelper(){}
    private:

    ros::NodeHandle m_Node;
    
    ros::Subscriber m_CurrentPositionSub;

    std::string DEFAULT_POSITION_SUB_TOPIC_NAME = "/amcl_pose";

    ros::Timer m_SavePositionTimer;

    double m_TimerInterval = 50.0;

    YAML::Node m_StoragePtr;

    std::shared_ptr<Position> m_LastPosPtr;

    std::string PATH_TO_POSITON_STORAGE_FILE;

    void savePositionTimerCB(const ros::TimerEvent&);

};

#endif // AMR_POSITION_HELPER_HPP