/**
 * @file amr_hardware_interface.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AMR_HARDWARE_INTERFACE_HPP_
#define AMR_HARDWARE_INTERFACE_HPP_

#include <ros/ros.h>
#include <ros/time.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <controller_manager/controller_manager.h>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <pthread.h>

#include "ethercat_interface/controller.hpp"
#include "amr_hwi_utility.hpp"

extern "C"
{
    #include "ethercat_interface/dc_sync.h"
}

class AmrEcController : public ethercat_interface::controller::Controller
{
    public:

    AmrEcController(const std::string& config_file_path)
    : Controller(config_file_path)
    {

    }

    ~AmrEcController(){joinThread();}

    bool runEcThread = true;

    bool m_SystemEnabled = false;

    void startTask() override
    {
        this->setThreadParams(SCHED_FIFO, 99);
        m_CyclicTaskThread = std::thread(
            &AmrEcController::cyclicTask,
            this
        );
        this->updateThreadInfo();
    }

    private:

    void cyclicTask() override;

};

namespace amr
{
    class HardwareInterface : public hardware_interface::RobotHW
    {
        public:

        HardwareInterface(ros::NodeHandle& nh);
        ~HardwareInterface();

        std::shared_ptr<controller_manager::ControllerManager> cm;

        std::unique_ptr<AmrEcController> m_EthercatController;

        utils::VelocityHelper m_DriverInfo;

        utils::PositionHelper m_PositionHelper;

        inline void setJointInfo(
            double left_pos,
            double right_pos,
            double left_vel,
            double right_vel
        )
        {
            m_JointPositions[0] = left_pos;
            m_JointPositions[1] = right_pos;

            m_JointVelocities[0] = left_vel;
            m_JointVelocities[1] = right_vel;
        }

        inline std::pair<double, double> getTargetVelocities()
        {
            return std::make_pair(m_VelocityCommands[0] / 10.0, m_VelocityCommands[1] / 10.0); // Left - Right
        }

        double m_WheelRadius = 0.1; // [m]

        struct
        {
            bool initialEcRead = true;
            double leftPosDiff = 0.0;
            double rightPosDiff = 0.0;
        }m_WheelHomingHelper;

        private:

        ros::NodeHandle m_NodeHandle;

        hardware_interface::JointStateInterface m_JointStateInterface;

        hardware_interface::VelocityJointInterface m_VelJointInterface;

        std::vector<std::string> m_JointNames;

        std::size_t m_NumJoints; 
        std::vector<double> m_JointPositions;
        std::vector<double> m_JointVelocities;
        std::vector<double> m_JointEfforts;
        std::vector<double> m_VelocityCommands;

        double m_LoopFrequency = 50;

        

        void loadParams();


    };
}

#endif // AMR_HARDWARE_INTERFACE_HPP_