#ifndef CALIBRATE_AXIS_H
#define CALIBRATE_AXIS_H

#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/calibrate_axis.hpp"
#include "rex_interfaces/msg/vesc_motor_command.hpp"
#include "ros_constants/RosCanConstants.hpp"
#include <cmath>
#include <limits>
#include <array>
#include <map>
#include <vector>
#include <algorithm>
#include <functional>

extern "C"
{
#include <libVescCan/VESC.h>
}

struct PositionStamped
{
    float position;
    rclcpp::Time receivedAt;
};

struct VelocityStamped
{
    int erpm;
    rclcpp::Time receivedAt;
};

class CalibrateAxis : public rclcpp::Node
{
public:
    CalibrateAxis(const rclcpp::NodeOptions &options);

private:
    rclcpp::Node::SharedPtr mNh;

    std::array<VESC_Id_t, 4> mCalibrationMotors;
    std::map<VESC_Id_t, PositionStamped> mMotorPositions;
    std::map<VESC_Id_t, VelocityStamped> mMotorVelocities;
    std::map<VESC_Id_t, rclcpp::TimerBase::SharedPtr> mSpeedStopTimers;

    std::map<std::string, float> mFloatParams;
    std::map<std::string, int> mIntParams;

    float mOffset;
    VESC_Id_t mCurrentMotorID;

    rex_interfaces::msg::RoverStatus::ConstSharedPtr mLastRoverStatus;

    rclcpp::Publisher<rex_interfaces::msg::VescMotorCommand>::SharedPtr mCalibrationMotorCommandPub;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr mVescStatusSub;
    rclcpp::Subscription<rex_interfaces::msg::CalibrateAxis>::SharedPtr mCalibrateAxisSub;
    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mRoverStatusSub;

    void initParams();

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr mParamCallbackHandle;

    bool calibrationMotorsContains(VESC_Id_t vescID);

    void handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg);
    void handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg);
    void handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg);

    bool isRecordedVelocityValid(VESC_Id_t vescID);
    bool isRecordedPositionValid(VESC_Id_t vescID);

    void initTimeoutTimers();
    void startTimeout(VESC_Id_t vescID);
    void cancelTimeout(VESC_Id_t vescID);

    void stopMotor(VESC_Id_t vescID);

    rex_interfaces::msg::VescMotorCommand frameStop(VESC_Id_t vescID);
    rex_interfaces::msg::VescMotorCommand frameSetOrigin(VESC_Id_t vescID);
    rex_interfaces::msg::VescMotorCommand frameSetPosition(VESC_Id_t vescID, float position);
    rex_interfaces::msg::VescMotorCommand frameSetVelocity(VESC_Id_t vescID, float speed);
};

#endif // CALIBRATE_AXIS_H