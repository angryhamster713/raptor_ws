#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/calibrate_axis.hpp"
#include "rex_interfaces/msg/vesc_motor_command.hpp"
#include "can_bridge/RosCanConstants.hpp"
#include <cmath>
#include <limits>
#include <can_msgs/msg/frame.hpp>
#include "can_bridge/VescInterop.hpp"

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

    float mOffset;
    VESC_Id_t mCurrentMotorID;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr mVescStatusSub;
    rclcpp::Subscription<rex_interfaces::msg::CalibrateAxis>::SharedPtr mCalibrateAxisSub;

    void handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg);
    void handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg);

    bool calibrationMotorsContains(VESC_Id_t vescID);

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
