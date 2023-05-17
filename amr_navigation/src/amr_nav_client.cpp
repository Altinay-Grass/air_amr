
#include "../include/amr_nav_client.hpp"

std::optional<NavPoints> getPointsFromConfigFile(const std::string& path_to_config_file)
{
    NavPoints navPoints;

    if(const auto configFile = YAML::LoadFile(path_to_config_file))
    {
        const auto points = configFile["nav_points"];

        for(const auto& point : points)
        {
            NavPoint navPoint;

            navPoint.goalPointName = point["point_name"].as<std::string>();
            navPoint.x = point["c_x"].as<double>();
            navPoint.y = point["c_y"].as<double>();
            navPoint.w = point["c_w"].as<double>();

            navPoints.emplace_back(std::move(navPoint));
        }


        if(!navPoints.empty())
        {
            return navPoints;
        }
    }


    return std::nullopt;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "amr_navigation_client");

    NavPoints* navPoints = nullptr;

    ros::NodeHandle amrNavNode;

    std::string pathToPointsFile;

    if(amrNavNode.hasParam("/amr_nav_config/path_to_points_file"))
    {
        amrNavNode.getParam("/amr_nav_config/path_to_points_file", pathToPointsFile);
    }
    else
    {   
        std::cout << "Can't find config file for map points" << std::endl;
        return 0;
    }

    auto navPointsOpt = getPointsFromConfigFile(pathToPointsFile);
    if(navPointsOpt == std::nullopt)
    {
        std::cout << "No points found in the config file" << std::endl;
        return 0;
    }

    navPoints = new NavPoints(navPointsOpt.value());
    bool runClient = true;
    char choice = 'y';
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveAcClient("amr_nav_action_client", true);

    while(runClient && ros::ok())
    {
        //std::cout << "Select navigation mode" << std::endl;
        //std::cout << "Relative to the robot: 1" << std::endl;
        //std::cout << "Relative to the map: 2" << std::endl;
//
        //int mode = 2;
        //std::cin >> mode;
        move_base_msgs::MoveBaseGoal goal;

        //if(mode == 1)
        //{
        //    goal.target_pose.header.frame_id = "base_link";
        //    goal.target_pose.header.stamp = ros::Time::now();
//
        //    double target_x;
        //    double target_w;
//
        //    std::cout << "Enter x and w of the robot";
        //}
        std::cout << "Enter target point's name:" << std::endl;
        std::string inputPointName;
        std::getline(std::cin, inputPointName);
        if(inputPointName.empty())
        {   
            std::cout << "Empty point name" << std::endl;
            break;
        }
        auto res = std::find_if(
            navPoints->begin(),
            navPoints->end(),
            [&inputPointName](const NavPoint& p){
                return inputPointName == p.goalPointName; 
            }
        );
        if(res == navPoints->end())
        {
            std::cout << "Can't find map point with name " << inputPointName << std::endl;
            return 0;
        }
        const NavPoint goalPoint = *res;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = goalPoint.x;
        goal.target_pose.pose.position.x = goalPoint.y;
        goal.target_pose.pose.orientation.w = goalPoint.w;

        moveAcClient.sendGoal(goal);

        moveAcClient.waitForResult();

        if(moveAcClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("The robot has arrived at the goal location");
        }
        else
        {
            ROS_WARN("Robot could not arrive at the goal location");
        }
        
        do{

            std::cout << "\nExit the program?" << std::endl;
            std::cin >> choice;

        }while (choice != 'n' && choice != 'y'); 

        if(choice = 'n')
        {
            runClient = false;
        }
    }
    

    ros::shutdown();
    return 0;
}