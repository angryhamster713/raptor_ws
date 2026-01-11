#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/calibrate_axis.hpp"
#include "rex_interfaces/msg/vesc_motor_command.hpp"
#include "can_bridge/RosCanConstants.hpp"
#include "can_bridge/VescInterop.hpp"
#include <can_msgs/msg/frame.hpp>
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

class CalibrateAxis
{
public:
    CalibrateAxis(rclcpp::Node::SharedPtr &nh);

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

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr mVescStatusSub;
    rclcpp::Subscription<rex_interfaces::msg::CalibrateAxis>::SharedPtr mCalibrateAxisSub;
    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mRoverStatusSub;

    void initParams();

    bool calibrationMotorsContains(VESC_Id_t vescID);

    void handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg);
    void handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg);
    void handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg);

    bool isRecordedVelocityValid(VESC_Id_t vescID);
    bool isRecordedPositionValid(VESC_Id_t vescID);

    void startTimeout(VESC_Id_t vescID);
    void cancelTimeout(VESC_Id_t vescID);

    void stopMotor(VESC_Id_t vescID);

    can_msgs::msg::Frame frameStop(VESC_Id_t vescID);
    can_msgs::msg::Frame frameSetOrigin(VESC_Id_t vescID);
    can_msgs::msg::Frame frameSetPosition(VESC_Id_t vescID, float position);
    can_msgs::msg::Frame frameSetVelocity(VESC_Id_t vescID, float speed);
};
