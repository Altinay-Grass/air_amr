#include "../include/amr_hwi_utility.hpp"

#include <ros/ros.h>

namespace amr
{
    namespace utils
    {
        double driverVelToLinear(int32_t driver_vel, const VelocityHelper& vel_helper)
        {

            double linearVel = ( driver_vel * M_PI * vel_helper.wheelDiameter * vel_helper.motorSideGear ) / 
            (vel_helper.wheelSideGear * vel_helper.motorGearHeat * vel_helper.velocityEncoderResolution );

            return linearVel;
        }

        int32_t linearVelToDriverCmd(const double linear_vel, const VelocityHelper& vel_helper)
        {
            int32_t driverCmd = ( linear_vel * vel_helper.wheelSideGear * vel_helper.motorGearHeat * vel_helper.velocityEncoderResolution ) /
            ( M_PI * vel_helper.wheelDiameter * vel_helper.motorSideGear );

            return driverCmd;
        }

        double motorPositionToWheelPositionRad(const int32_t encoder_count, const PositionHelper& pos_helper)
        {
            double encoderCount = (double)encoder_count * 1.0;
            //double pos_rad = ((encoderCount / pos_helper.encoderResolution) * (2.0 * M_PI)) / pos_helper.gearRatio;
            double pos_rad = (encoderCount / 1048576.0) * (2.0 * M_PI) / (45.7143); 
            //ROS_WARN(std::to_string(pos_rad).c_str());
            return pos_rad;
        }
    }
}