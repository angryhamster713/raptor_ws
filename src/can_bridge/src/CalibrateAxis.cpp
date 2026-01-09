#include "can_bridge/CalibrateAxis.hpp"

constexpr float CALIBRATION_POS_SPEED = 3.0f;
constexpr float CALIBRATION_POS_ACCELERATION = 3.0f;

constexpr float CALIBRATION_MAX_SPEED = 3.0f;
constexpr float CALIBRATION_MAX_OFFSET_SHIFT = 30.0f;

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

constexpr float OUTDATED_DURATION = 0.3f;

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

    mCalibrationMotors = {
        RosCanConstants::VescIds::front_left_stepper, 
        RosCanConstants::VescIds::front_right_stepper, 
        RosCanConstants::VescIds::rear_left_stepper, 
        RosCanConstants::VescIds::rear_right_stepper, 
    };

    mOffset = NaN;
    mCurrentMotorID = 0;
};


bool CalibrateAxis::motorCanBeCalibrated(VESC_Id_t vescID)
{
    return std::find(mCalibrationMotors.begin(), mCalibrationMotors.end(), vescID) != mCalibrationMotors.end();
}

void CalibrateAxis::handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg)
{
    if (!motorCanBeCalibrated(msg->vesc_id)) {
        return;
    }
    rclcpp::Time now = rclcpp::Clock().now();
    mMotorPositions[msg->vesc_id] = {msg->pid_pos, now};
    mMotorVelocities[msg->vesc_id] = {msg->erpm, now};
}

void CalibrateAxis::handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg)
{
    using CalibrateMsg = rex_interfaces::msg::CalibrateAxis;

    // Reject messages to motors not on the list
    if (!motorCanBeCalibrated(msg->vesc_id))
    {
        RCLCPP_ERROR(mNh->get_logger(), "Attempted to calibrate motor with invalid VESC ID: %#x", msg->vesc_id);
        return;
    }

    // If velocity missing or outdated
    if (!isRecordedVelocityValid(msg->vesc_id))
    {
        return;
    }

    // If motor is still moving, only STOP or CANCEL are fine
    if (abs(mMotorVelocities[msg->vesc_id].erpm) > 0) {
        if (!(msg->action_type == CalibrateMsg::ACTION_TYPE_STOP || msg->action_type == CalibrateMsg::ACTION_TYPE_CANCEL)) {
            RCLCPP_DEBUG(mNh->get_logger(), "Calibration request rejected, the motor is still moving");
            return;
        }
    }

    // --------------------

    // Action any other than OFFSET
    if (msg->action_type != rex_interfaces::msg::CalibrateAxis::ACTION_TYPE_OFFSET)
    {
        mOffset = NaN;
    }

    // Message to a different motor
    if (mCurrentMotorID != msg->vesc_id)
    {
        if (mCurrentMotorID) stopMotor(mCurrentMotorID);
        mCurrentMotorID = msg->vesc_id;
        mOffset = NaN;
    }

    // --------------------

    can_msgs::msg::Frame fr;
    switch (msg->action_type)
    {
        case CalibrateMsg::ACTION_TYPE_STOP:
            stopMotor(msg->vesc_id);
            break;

        case CalibrateMsg::ACTION_TYPE_RETURN_TO_ORIGIN:
            fr = frameSetPosition(msg->vesc_id, 0.0f);
            mOffset = 0.0;
            mRawCanPub->publish(fr);
            break;

        case CalibrateMsg::ACTION_TYPE_CONFIRM:
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
            if (std::isnan(mOffset))
            {
                if (!isRecordedPositionValid(msg->vesc_id)) {
                    return;
                }
                // Copy realtime-tracked position
                mOffset = mMotorPositions[msg->vesc_id].position;
            }

            // Limit value
            float offsetShift = msg->value;
            if (std::abs(offsetShift) > CALIBRATION_MAX_OFFSET_SHIFT)
            {
                RCLCPP_WARN(mNh->get_logger(), "Calibration: provided a relative offset that's too large (max %f)", CALIBRATION_MAX_OFFSET_SHIFT);
                offsetShift = std::clamp(offsetShift, -CALIBRATION_MAX_OFFSET_SHIFT, CALIBRATION_MAX_OFFSET_SHIFT);
            }

            mOffset += offsetShift;
            fr = frameSetPosition(msg->vesc_id, mOffset);
            mRawCanPub->publish(fr);
            break;

        case CalibrateMsg::ACTION_TYPE_SET_VELOCITY:
            // Limit value
            float velocity = msg->value;
            if (std::abs(velocity) > CALIBRATION_MAX_SPEED) {
                RCLCPP_WARN(mNh->get_logger(), "Calibration: provided a velocity that's too large (max %f)", CALIBRATION_MAX_SPEED);
                velocity = std::clamp(velocity, -CALIBRATION_MAX_SPEED, CALIBRATION_MAX_SPEED);
            }

            fr = frameSetVelocity(msg->vesc_id, mOffset);
            mRawCanPub->publish(fr);
            break;
    }
}

bool CalibrateAxis::isRecordedVelocityValid(VESC_Id_t vescID) {
    if (!mMotorVelocities.count(vescID)) {
        RCLCPP_WARN(mNh->get_logger(), "Calibration failed, no recorded velocity for motor with ID %#x", vescID);
        return false;
    }
    rclcpp::Time now = rclcpp::Clock().now();
    if ((now-mMotorVelocities[vescID].receivedAt).seconds() > OUTDATED_DURATION) {
        RCLCPP_WARN(mNh->get_logger(), "Calibration failed, recorded velocity for motor with id %#x is outdated", vescID);
    }
}

bool CalibrateAxis::isRecordedPositionValid(VESC_Id_t vescID) {
    if (!mMotorPositions.count(vescID)) {
        RCLCPP_WARN(mNh->get_logger(), "Calibration failed, no recorded position for motor with ID %#x", vescID);
        return false;
    }

    rclcpp::Time now = rclcpp::Clock().now();
    if ((now-mMotorPositions[vescID].receivedAt).seconds() > OUTDATED_DURATION) {
        RCLCPP_WARN(mNh->get_logger(), "Calibration failed, recorded position for motor with id %#x is outdated", vescID);
    }
}

void CalibrateAxis::stopMotor(VESC_Id_t vescID)
{
    can_msgs::msg::Frame fr = frameStop(vescID);
    mRawCanPub->publish(fr);
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
    cmdf.commandDataEx_1 = CALIBRATION_POS_SPEED;
    cmdf.commandDataEx_2 = CALIBRATION_POS_ACCELERATION;
    cmdf.vescID = vescID;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}

can_msgs::msg::Frame CalibrateAxis::frameSetVelocity(VESC_Id_t vescID, float velocity)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    cmdf.command = VESC_COMMAND_SET_RPM;
    cmdf.commandData = velocity;
    cmdf.vescID = vescID;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}