#include "can_bridge/CalibrateAxis.hpp"

constexpr float CALIBRATION_SPEED = 3.0;
constexpr float CALIBRATION_ACCELERATION = 3.0;

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

CalibrateAxis::CalibrateAxis(rclcpp::Node::SharedPtr &nh) : mNh(nh) {
    const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

    RCLCPP_INFO(mNh->get_logger(), "Setting up");

    mVescStatusSub = mNh->create_subscription<rex_interfaces::msg::VescStatus>(
        RosCanConstants::RosTopics::can_vesc_status,
        qos, std::bind(&CalibrateAxis::handleVescStatus, this, std::placeholders::_1)
    );
    mCalibrateAxisSub = mNh->create_subscription<rex_interfaces::msg::CalibrateAxis>(
        "/MQTT/CalibrateAxis",
        qos, std::bind(&CalibrateAxis::handleCalibrateAxis, this, std::placeholders::_1)
    );
	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);

    std::map<VESC_Id_t, float> mMotorPositions;
    mIdsToCalibrate = {
        RosCanConstants::VescIds::front_left_stepper, 
        RosCanConstants::VescIds::front_right_stepper, 
        RosCanConstants::VescIds::rear_left_stepper, 
        RosCanConstants::VescIds::rear_right_stepper, 
    };

    mOffset = NaN;
    mCurrentMotorID = 0;
};


bool CalibrateAxis::calibratedMotorListContains(VESC_Id_t id)
{
    return std::find(mIdsToCalibrate.begin(), mIdsToCalibrate.end(), id) != mIdsToCalibrate.end();
}

void CalibrateAxis::handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg)
{
    if (!calibratedMotorListContains(msg->vesc_id)) {
        return;
    }
    mMotorPositions[msg->vesc_id] = msg->pid_pos;
}

void CalibrateAxis::handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg)
{
    // Reject messages to motors not on the list
    if (!calibratedMotorListContains(msg->vesc_id))
    {
        RCLCPP_ERROR(mNh->get_logger(), "Attempted to calibrate motor with invalid VESC ID: %#x", msg->vesc_id);
    }

    // TODO: If motor is moving, only accept STOP or CANCEL

    if (msg->action_type != rex_interfaces::msg::CalibrateAxis::ACTION_TYPE_OFFSET)
    {
        // Forget offset
        mOffset = NaN;
    }
    if (mCurrentMotorID != msg->vesc_id)
    {
        // Message to a different motor
        if (mCurrentMotorID) stopMotor(mCurrentMotorID);
        mCurrentMotorID = msg->vesc_id;
        mOffset = NaN;
    }

    using CalibrateMsg = rex_interfaces::msg::CalibrateAxis;
    can_msgs::msg::Frame fr;
    switch (msg->action_type) {
        case CalibrateMsg::ACTION_TYPE_STOP:
            stopMotor(msg->vesc_id);
            break;
        case CalibrateMsg::ACTION_TYPE_RETURN_TO_ORIGIN:
            // TODO: Reject if motor is moving
            fr = frameSetPosition(msg->vesc_id, 0.0f);
            mRawCanPub->publish(fr);
            break;
        case CalibrateMsg::ACTION_TYPE_CONFIRM:
            // TODO: Reject if motor is moving
            stopMotor(msg->vesc_id);
            fr = frameSetOrigin(msg->vesc_id);
            mRawCanPub->publish(fr);
            mOffset = NaN;
            mCurrentMotorID = 0;
            break;
        case CalibrateMsg::ACTION_TYPE_CANCEL:
            stopMotor(msg->vesc_id);
            mOffset = NaN;
            mCurrentMotorID = 0;
            break;
        case CalibrateMsg::ACTION_TYPE_OFFSET:
            // TODO: Reject if motor is moving
            // TODO: check if copied value present && not outdated
            if (std::isnan(mOffset)) mOffset = mMotorPositions[msg->vesc_id];
            // TODO: Value limit
            mOffset += msg->value;
            fr = frameSetPosition(msg->vesc_id, mOffset);
            mRawCanPub->publish(fr);
            break;
        case CalibrateMsg::ACTION_TYPE_SET_VELOCITY:
            // TODO: Reject if motor is moving
            // TODO: check if copied value present && not outdated
            if (std::isnan(mOffset)) mOffset = mMotorPositions[msg->vesc_id];
            // TODO: Value limit
            mOffset += msg->value;
            fr = frameSetPosition(msg->vesc_id, mOffset);
            mRawCanPub->publish(fr);
            break;
    }
}

can_msgs::msg::Frame CalibrateAxis::frameStop(VESC_Id_t vescID)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    cmdf.command = VESC_COMMAND_SET_CURRENT;
    cmdf.commandData = 0.0;
    cmdf.vescID = vescID;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}

void CalibrateAxis::stopMotor(VESC_Id_t vescID)
{
    can_msgs::msg::Frame fr = frameStop(vescID);
    mRawCanPub->publish(fr);
}

can_msgs::msg::Frame CalibrateAxis::frameSetOrigin(VESC_Id_t vescID)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    cmdf.command = VESC_COMMAND_SET_ORIGIN;
    cmdf.commandDataExB = 0.0;
    cmdf.vescID = vescID;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}

can_msgs::msg::Frame CalibrateAxis::frameSetPosition(VESC_Id_t vescID, float position)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    cmdf.command = VESC_COMMAND_SET_POS_SPEED_LOOP;
    cmdf.commandDataEx_0 = position;
    cmdf.commandDataEx_1 = CALIBRATION_SPEED;
    cmdf.commandDataEx_2 = CALIBRATION_ACCELERATION;
    cmdf.vescID = vescID;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}

can_msgs::msg::Frame CalibrateAxis::frameSetSpeed(VESC_Id_t vescID, float speed)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    cmdf.command = VESC_COMMAND_SET_RPM;
    cmdf.commandDataEx_0 = speed;
    cmdf.commandDataEx_1 = CALIBRATION_SPEED;
    cmdf.commandDataEx_2 = CALIBRATION_ACCELERATION;
    cmdf.vescID = vescID;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}

// can_msgs::msg::Frame CalibrateAxis::encodeMotorVel(const rex_interfaces::msg::VescMotorCommand &vescMotorCommand, const VESC_Id_t vescId)
// {
// 	VESC_CommandFrame cmdf;
// 	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

// 	switch (vescMotorCommand.command_id)
// 	{
// 	case VESC_COMMAND_SET_ORIGIN:
// 		cmdf.commandDataExB = vescMotorCommand.set_origin_data;
// 		break;
// 	case VESC_COMMAND_SET_POS_SPEED_LOOP:
// 		cmdf.commandDataEx_0 = vescMotorCommand.set_pos_speed_loop_position;
// 		cmdf.commandDataEx_1 = vescMotorCommand.set_pos_speed_loop_speed;
// 		cmdf.commandDataEx_2 = vescMotorCommand.set_pos_speed_loop_acceleration;
// 		break;
// 	default:
// 		cmdf.commandData = vescMotorCommand.set_value;
// 	}

// 	cmdf.command = vescMotorCommand.command_id;
// 	cmdf.vescID = vescId;

// 	VESC_RawFrame rf;
// 	VESC_ZeroMemory(&rf, sizeof(rf));
// 	VESC_convertCmdToRaw(&rf, &cmdf);

// 	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
// 	fr.header.stamp = rclcpp::Clock().now();

// 	return fr;
// }