#include "calibration_manager/CalibrateAxis.hpp"

const std::string CALIBRATION_POS_SPEED = "calibration.pos_speed";
const std::string CALIBRATION_POS_ACCELERATION = "calibration.pos_acceleration";

const std::string CALIBRATION_MAX_SPEED = "calibration.max_speed";
const std::string CALIBRATION_MAX_OFFSET_SHIFT = "calibration.max_offset_shift";

const std::string CALIBRATION_OUTDATED_DURATION_S = "calibration.outdated_duration_s";
const std::string CALIBRATION_SPEED_TIMEOUT_MS = "calibration.speed_timeout_ms";

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

CalibrateAxis::CalibrateAxis(const rclcpp::NodeOptions &options) : Node("calibrate_axis", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	initParams();

	mCalibrationMotorCommandPub = this->create_publisher<rex_interfaces::msg::VescMotorCommand>(
		RosCanConstants::RosTopics::can_calibration_motor_command, qos);
	mVescStatusSub = this->create_subscription<rex_interfaces::msg::VescStatus>(
		RosCanConstants::RosTopics::can_vesc_status,
		qos, std::bind(&CalibrateAxis::handleVescStatus, this, std::placeholders::_1));
	mCalibrateAxisSub = this->create_subscription<rex_interfaces::msg::CalibrateAxis>(
		RosCanConstants::RosTopics::mqtt_calibrate_axis,
		qos, std::bind(&CalibrateAxis::handleCalibrateAxis, this, std::placeholders::_1));
	mRoverStatusSub = this->create_subscription<rex_interfaces::msg::RoverStatus>(
		RosCanConstants::RosTopics::mqtt_rover_status,
		qos, std::bind(&CalibrateAxis::handleRoverStatus, this, std::placeholders::_1));

	mCalibrationMotors = {
		RosCanConstants::VescIds::front_left_stepper,
		RosCanConstants::VescIds::front_right_stepper,
		RosCanConstants::VescIds::rear_left_stepper,
		RosCanConstants::VescIds::rear_right_stepper,
	};

	mOffset = NaN;
	mCurrentMotorID = 0;

	RCLCPP_DEBUG(this->get_logger(), "Calibration module started.");
};

void CalibrateAxis::initParams()
{
	mFloatParams = {
		{CALIBRATION_POS_SPEED, 3.0f},
		{CALIBRATION_POS_ACCELERATION, 3.0f},
		{CALIBRATION_MAX_SPEED, 3.0f},
		{CALIBRATION_MAX_OFFSET_SHIFT, 30.0f},
		{CALIBRATION_OUTDATED_DURATION_S, 0.3f}};
	for (auto &[name, value] : mFloatParams)
	{
		this->declare_parameter(name, value);
		mFloatParams[name] = this->get_parameter(name).as_double();
	}

	mIntParams = {
		{CALIBRATION_SPEED_TIMEOUT_MS, 500}};
	for (auto &[name, value] : mIntParams)
	{
		this->declare_parameter(name, value);
		mIntParams[name] = this->get_parameter(name).as_int();
	}

	mParamCallbackHandle = this->add_post_set_parameters_callback(
		[this](const std::vector<rclcpp::Parameter> &params)
		{
			for (const auto &param : params)
			{
				const std::string name = param.get_name();
				if (mFloatParams.count(name))
				{
					mFloatParams[name] = param.as_double();
				}
				else if (mIntParams.count(name))
				{
					mIntParams[name] = param.as_int();
				}
			}
		});
}

bool CalibrateAxis::calibrationMotorsContains(VESC_Id_t vescID)
{
	return std::find(mCalibrationMotors.begin(), mCalibrationMotors.end(), vescID) != mCalibrationMotors.end();
}

void CalibrateAxis::handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg)
{
	if (!calibrationMotorsContains(msg->vesc_id))
	{
		return;
	}
	rclcpp::Time now = rclcpp::Clock().now();
	mMotorPositions[msg->vesc_id] = {static_cast<float>(msg->precise_pos), now};
	mMotorVelocities[msg->vesc_id] = {msg->erpm, now};
}

void CalibrateAxis::handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg)
{
	using CalibrateMsg = rex_interfaces::msg::CalibrateAxis;

	if (!mLastRoverStatus || mLastRoverStatus->control_mode != rex_interfaces::msg::RoverStatus::CONTROL_MODE_ESTOP)
	{
		RCLCPP_ERROR(this->get_logger(), "Rover not in ESTOP, calibration not permitted.");
		return;
	}

	if (!calibrationMotorsContains(msg->vesc_id))
	{
		RCLCPP_ERROR(this->get_logger(), "Attempted to calibrate motor with invalid VESC ID: %#x", msg->vesc_id);
		return;
	}

	// If velocity missing or outdated
	if (!isRecordedVelocityValid(msg->vesc_id))
	{
		return;
	}

	// If motor is still moving, only STOP or CANCEL are fine
	if (std::abs(mMotorVelocities[msg->vesc_id].erpm) > 0)
	{
		if (!(msg->action_type == CalibrateMsg::ACTION_TYPE_STOP || msg->action_type == CalibrateMsg::ACTION_TYPE_CANCEL))
		{
			RCLCPP_DEBUG(this->get_logger(), "Calibration request rejected, the motor is still moving");
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
		if (mCurrentMotorID)
			stopMotor(mCurrentMotorID);
		mCurrentMotorID = msg->vesc_id;
		mOffset = NaN;
	}

	// --------------------

	rex_interfaces::msg::VescMotorCommand fr;
	float offsetShift;
	switch (msg->action_type)
	{
	case CalibrateMsg::ACTION_TYPE_STOP:
		stopMotor(msg->vesc_id);
		break;

	case CalibrateMsg::ACTION_TYPE_RETURN_TO_ORIGIN:
		fr = frameSetPosition(msg->vesc_id, 0.0f);
		mOffset = 0.0;
		mCalibrationMotorCommandPub->publish(fr);
		break;

	case CalibrateMsg::ACTION_TYPE_CONFIRM:
		stopMotor(msg->vesc_id);
		fr = frameSetOrigin(msg->vesc_id);
		mCalibrationMotorCommandPub->publish(fr);
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
			if (!isRecordedPositionValid(msg->vesc_id))
			{
				return;
			}
			// Copy realtime-tracked position
			mOffset = mMotorPositions[msg->vesc_id].position;
		}

		// Limit value
		offsetShift = msg->value;
		if (std::abs(offsetShift) > mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT])
		{
			RCLCPP_WARN(this->get_logger(), "Calibration: provided a relative offset that's too large (max %f)", mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT]);
			offsetShift = std::clamp(offsetShift, -mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT], mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT]);
		}

		mOffset += offsetShift;
		fr = frameSetPosition(msg->vesc_id, mOffset);
		mCalibrationMotorCommandPub->publish(fr);
		break;

	case CalibrateMsg::ACTION_TYPE_SET_VELOCITY:
		// Limit value
		float velocity = msg->value;
		if (std::abs(velocity) > mFloatParams[CALIBRATION_MAX_SPEED])
		{
			RCLCPP_WARN(this->get_logger(), "Calibration: provided a velocity that's too large (max %f)", mFloatParams[CALIBRATION_MAX_SPEED]);
			velocity = std::clamp(velocity, -mFloatParams[CALIBRATION_MAX_SPEED], mFloatParams[CALIBRATION_MAX_SPEED]);
		}

		// Handle the stop timers
		if (velocity == 0.0f)
			cancelTimeout(msg->vesc_id);
		else
			startTimeout(msg->vesc_id);

		fr = frameSetVelocity(msg->vesc_id, velocity);
		mCalibrationMotorCommandPub->publish(fr);
		break;
	}
}

void CalibrateAxis::handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg)
{
	mLastRoverStatus = msg;
}

bool CalibrateAxis::isRecordedVelocityValid(VESC_Id_t vescID)
{
	// Checks if the tracked velocity is missing or outdated

	if (!mMotorVelocities.count(vescID))
	{
		RCLCPP_WARN(this->get_logger(), "Calibration failed, no recorded velocity for motor with ID %#x", vescID);
		return false;
	}
	rclcpp::Time now = rclcpp::Clock().now();
	if ((now - mMotorVelocities[vescID].receivedAt).seconds() > mFloatParams[CALIBRATION_OUTDATED_DURATION_S])
	{
		RCLCPP_WARN(this->get_logger(), "Calibration failed, recorded velocity for motor with id %#x is outdated", vescID);
		return false;
	}
	return true;
}

bool CalibrateAxis::isRecordedPositionValid(VESC_Id_t vescID)
{
	// Checks if the tracked position is missing or outdated

	if (!mMotorPositions.count(vescID))
	{
		RCLCPP_WARN(this->get_logger(), "Calibration failed, no recorded position for motor with ID %#x", vescID);
		return false;
	}

	rclcpp::Time now = rclcpp::Clock().now();
	if ((now - mMotorPositions[vescID].receivedAt).seconds() > mFloatParams[CALIBRATION_OUTDATED_DURATION_S])
	{
		RCLCPP_WARN(this->get_logger(), "Calibration failed, recorded position for motor with id %#x is outdated", vescID);
		return false;
	}
	return true;
}

void CalibrateAxis::startTimeout(VESC_Id_t vescID)
{
	// Cancel previous timeout
	cancelTimeout(vescID);
	mSpeedStopTimers[vescID] = this->create_timer(
		std::chrono::milliseconds(mIntParams[CALIBRATION_SPEED_TIMEOUT_MS]),
		[this, vescID]()
		{
			stopMotor(vescID);
			// cancelTimeout(vescID); // Redundant as stopMotor already calls cancelTimeout()
		});
}

void CalibrateAxis::cancelTimeout(VESC_Id_t vescID)
{
	// Check if ID in map. If it is, check if the pointer is non-null. If not, cancel.
	auto iterator = mSpeedStopTimers.find(vescID);
	if (iterator == mSpeedStopTimers.end() || !iterator->second || iterator->second->is_canceled())
	{
		// Nothing to cancel
		return;
	}
	iterator->second->cancel();
}

void CalibrateAxis::stopMotor(VESC_Id_t vescID)
{
	cancelTimeout(vescID);

	rex_interfaces::msg::VescMotorCommand fr = frameStop(vescID);
	mCalibrationMotorCommandPub->publish(fr);
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameStop(VESC_Id_t vescID)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_CURRENT;
	msg.set_value = 0.0f;

	msg.header.stamp = rclcpp::Clock().now();

	return msg;
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameSetOrigin(VESC_Id_t vescID)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_ORIGIN;
	msg.set_origin_data = 0.0f;

	msg.header.stamp = rclcpp::Clock().now();

	return msg;
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameSetPosition(VESC_Id_t vescID, float position)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_POS_SPEED_LOOP;
	msg.set_pos_speed_loop_position = position;
	msg.set_pos_speed_loop_speed = mFloatParams[CALIBRATION_POS_SPEED];
	msg.set_pos_speed_loop_acceleration = mFloatParams[CALIBRATION_POS_ACCELERATION];

	msg.header.stamp = rclcpp::Clock().now();

	return msg;
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameSetVelocity(VESC_Id_t vescID, float velocity)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_RPM;
	msg.set_value = velocity;

	msg.header.stamp = rclcpp::Clock().now();

	return msg;
}
