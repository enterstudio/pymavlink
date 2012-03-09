// MESSAGE SET_ROLL_PITCH_YAW_SPEED_THRUST PACKING

#define MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST 57

typedef struct __mavlink_set_roll_pitch_yaw_speed_thrust_t
{
 Single roll_speed; ///< Desired roll angular speed in rad/s
 Single pitch_speed; ///< Desired pitch angular speed in rad/s
 Single yaw_speed; ///< Desired yaw angular speed in rad/s
 Single thrust; ///< Collective thrust, normalized to 0 .. 1
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
} mavlink_set_roll_pitch_yaw_speed_thrust_t;

#define MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST_LEN 18
#define MAVLINK_MSG_ID_57_LEN 18



#define MAVLINK_MESSAGE_INFO_SET_ROLL_PITCH_YAW_SPEED_THRUST { \
	"SET_ROLL_PITCH_YAW_SPEED_THRUST", \
	6, \
	{  { "roll_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_roll_pitch_yaw_speed_thrust_t, roll_speed) }, \
         { "pitch_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_roll_pitch_yaw_speed_thrust_t, pitch_speed) }, \
         { "yaw_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_roll_pitch_yaw_speed_thrust_t, yaw_speed) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_set_roll_pitch_yaw_speed_thrust_t, thrust) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_set_roll_pitch_yaw_speed_thrust_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_set_roll_pitch_yaw_speed_thrust_t, target_component) }, \
         } \
}


/**
 * @brief Pack a set_roll_pitch_yaw_speed_thrust message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll_speed Desired roll angular speed in rad/s
 * @param pitch_speed Desired pitch angular speed in rad/s
 * @param yaw_speed Desired yaw angular speed in rad/s
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, Single roll_speed, Single pitch_speed, Single yaw_speed, Single thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_Single(buf, 0, roll_speed);
	_mav_put_Single(buf, 4, pitch_speed);
	_mav_put_Single(buf, 8, yaw_speed);
	_mav_put_Single(buf, 12, thrust);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet;
	packet.roll_speed = roll_speed;
	packet.pitch_speed = pitch_speed;
	packet.yaw_speed = yaw_speed;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 24);
}

/**
 * @brief Pack a set_roll_pitch_yaw_speed_thrust message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll_speed Desired roll angular speed in rad/s
 * @param pitch_speed Desired pitch angular speed in rad/s
 * @param yaw_speed Desired yaw angular speed in rad/s
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,Single roll_speed,Single pitch_speed,Single yaw_speed,Single thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_Single(buf, 0, roll_speed);
	_mav_put_Single(buf, 4, pitch_speed);
	_mav_put_Single(buf, 8, yaw_speed);
	_mav_put_Single(buf, 12, thrust);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet;
	packet.roll_speed = roll_speed;
	packet.pitch_speed = pitch_speed;
	packet.yaw_speed = yaw_speed;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 24);
}

/**
 * @brief Encode a set_roll_pitch_yaw_speed_thrust struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_roll_pitch_yaw_speed_thrust C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_speed_thrust_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_roll_pitch_yaw_speed_thrust_t* set_roll_pitch_yaw_speed_thrust)
{
	return mavlink_msg_set_roll_pitch_yaw_speed_thrust_pack(system_id, component_id, msg, set_roll_pitch_yaw_speed_thrust->target_system, set_roll_pitch_yaw_speed_thrust->target_component, set_roll_pitch_yaw_speed_thrust->roll_speed, set_roll_pitch_yaw_speed_thrust->pitch_speed, set_roll_pitch_yaw_speed_thrust->yaw_speed, set_roll_pitch_yaw_speed_thrust->thrust);
}

/**
 * @brief Send a set_roll_pitch_yaw_speed_thrust message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll_speed Desired roll angular speed in rad/s
 * @param pitch_speed Desired pitch angular speed in rad/s
 * @param yaw_speed Desired yaw angular speed in rad/s
 * @param thrust Collective thrust, normalized to 0 .. 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_roll_pitch_yaw_speed_thrust_send(mavlink_channel_t chan, byte target_system, byte target_component, Single roll_speed, Single pitch_speed, Single yaw_speed, Single thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_Single(buf, 0, roll_speed);
	_mav_put_Single(buf, 4, pitch_speed);
	_mav_put_Single(buf, 8, yaw_speed);
	_mav_put_Single(buf, 12, thrust);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST, buf, 18, 24);
#else
	mavlink_set_roll_pitch_yaw_speed_thrust_t packet;
	packet.roll_speed = roll_speed;
	packet.pitch_speed = pitch_speed;
	packet.yaw_speed = yaw_speed;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_SPEED_THRUST, (const char *)&packet, 18, 24);
#endif
}

#endif

// MESSAGE SET_ROLL_PITCH_YAW_SPEED_THRUST UNPACKING


/**
 * @brief Get field target_system from set_roll_pitch_yaw_speed_thrust message
 *
 * @return System ID
 */
static inline byte mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  16);
}

/**
 * @brief Get field target_component from set_roll_pitch_yaw_speed_thrust message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  17);
}

/**
 * @brief Get field roll_speed from set_roll_pitch_yaw_speed_thrust message
 *
 * @return Desired roll angular speed in rad/s
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_roll_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  0);
}

/**
 * @brief Get field pitch_speed from set_roll_pitch_yaw_speed_thrust message
 *
 * @return Desired pitch angular speed in rad/s
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_pitch_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  4);
}

/**
 * @brief Get field yaw_speed from set_roll_pitch_yaw_speed_thrust message
 *
 * @return Desired yaw angular speed in rad/s
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_yaw_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  8);
}

/**
 * @brief Get field thrust from set_roll_pitch_yaw_speed_thrust message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  12);
}

/**
 * @brief Decode a set_roll_pitch_yaw_speed_thrust message into a struct
 *
 * @param msg The message to decode
 * @param set_roll_pitch_yaw_speed_thrust C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_roll_pitch_yaw_speed_thrust_decode(const mavlink_message_t* msg, mavlink_set_roll_pitch_yaw_speed_thrust_t* set_roll_pitch_yaw_speed_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_roll_pitch_yaw_speed_thrust->roll_speed = mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_roll_speed(msg);
	set_roll_pitch_yaw_speed_thrust->pitch_speed = mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_pitch_speed(msg);
	set_roll_pitch_yaw_speed_thrust->yaw_speed = mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_yaw_speed(msg);
	set_roll_pitch_yaw_speed_thrust->thrust = mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_thrust(msg);
	set_roll_pitch_yaw_speed_thrust->target_system = mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_target_system(msg);
	set_roll_pitch_yaw_speed_thrust->target_component = mavlink_msg_set_roll_pitch_yaw_speed_thrust_get_target_component(msg);
#else
	memcpy(set_roll_pitch_yaw_speed_thrust, _MAV_PAYLOAD(msg), 18);
#endif
}
