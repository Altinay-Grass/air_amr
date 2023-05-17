#include "../include/amr_localization/amr_position_helper.hpp"


PositionHelper::PositionHelper()
{
    
    if(m_Node.hasParam("/position_helper/timer_interval"))
        m_Node.getParam("/position_helper/timer_interval", m_TimerInterval);

    if(m_Node.hasParam("/position_helper/position_sub_topic"))
        m_Node.getParam("/position_helper/position_sub_topic", DEFAULT_POSITION_SUB_TOPIC_NAME);

    if(m_Node.hasParam("/position_helper/path_to_position_storage_file"))
    {
        m_Node.getParam("/position_helper/path_to_position_storage_file", PATH_TO_POSITON_STORAGE_FILE);
    }
    else
    {

        ros::shutdown();
    }

    Position emptyPose;
    emptyPose.header.frame_id = "map";
    emptyPose.header.stamp = ros::Time::now();
    emptyPose.pose.pose.position.x = 0.0;
    emptyPose.pose.pose.position.y = 0.0;
    emptyPose.pose.pose.position.z = 0.0;

    emptyPose.pose.pose.orientation.x = 0.0;
    emptyPose.pose.pose.orientation.y = 0.0;
    emptyPose.pose.pose.orientation.z = 0.0;
    emptyPose.pose.pose.orientation.w = 0.0;

    m_LastPosPtr = std::make_shared<Position>(emptyPose);

    m_CurrentPositionSub = m_Node.subscribe<Position>(
        DEFAULT_POSITION_SUB_TOPIC_NAME,
        10,
        [this](const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
            *this->m_LastPosPtr = *msg;
        }
    );

    m_StoragePtr = YAML::LoadFile(PATH_TO_POSITON_STORAGE_FILE);

    m_SavePositionTimer = m_Node.createTimer(
        ros::Duration(m_TimerInterval),
        std::bind(
            &PositionHelper::savePositionTimerCB,
            this,
            std::placeholders::_1
        ),
        false
    );
}

void PositionHelper::savePositionTimerCB(const ros::TimerEvent&)
{

    if(!m_LastPosPtr)
    {
        return;
    }

    const Position lastPosition = *m_LastPosPtr;

    auto lastPositionNode = m_StoragePtr["last_position"];

    lastPositionNode["pos"]["x"] = lastPosition.pose.pose.position.x;
    lastPositionNode["pos"]["y"] = lastPosition.pose.pose.position.y;
    lastPositionNode["pos"]["z"] = lastPosition.pose.pose.position.z;

    lastPositionNode["ori"]["x"] = lastPosition.pose.pose.orientation.x;
    lastPositionNode["ori"]["y"] = lastPosition.pose.pose.orientation.y;
    lastPositionNode["ori"]["z"] = lastPosition.pose.pose.orientation.z;
    lastPositionNode["ori"]["w"] = lastPosition.pose.pose.orientation.w;    

    const auto covariance = lastPosition.pose.covariance;

    std::vector<double> covarianceVec;
    covarianceVec.assign(
        &covariance[0],
        &covariance[covariance.size() -1]
    );
    //covarianceVec.resize(covariance.size());

    /* std::copy(
        covariance.begin(),
        covariance.end(),
        covarianceVec
    ); */

    lastPositionNode["covariance"] = covarianceVec;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_position_helper");

    PositionHelper ph;
    
    ros::spin();
    ros::shutdown();

    return 0;
}