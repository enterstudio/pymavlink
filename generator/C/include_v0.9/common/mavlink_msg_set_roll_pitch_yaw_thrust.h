// MESSAGE SET_ROLL_PITCH_YAW_THRUST PACKING

#define MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST 55

typedef struct __mavlink_set_roll_pitch_yaw_thrust_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 Single roll; ///< Desired roll angle in radians
 Single pitch; ///< Desired pitch angle in radians
 Single yaw; ///< Desired yaw angle in radians
 Single thrust; ///< Collective thrust, normalized to 0 .. 1
} mavlink_set_roll_pitch_yaw_thrust_t;

#define MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST_LEN 18
#define MAVLINK_MSG_ID_55_LEN 18



#define MAVLINK_MESSAGE_INFO_SET_ROLL_PITCH_YAW_THRUST { \
	"SET_ROLL_PITCH_YAW_THRUST", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_roll_pitch_yaw_thrust_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_roll_pitch_yaw_thrust_t, target_component) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 2, offsetof(mavlink_set_roll_pitch_yaw_thrust_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 6, offsetof(mavlink_set_roll_pitch_yaw_thrust_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 10, offsetof(mavlink_set_roll_pitch_yaw_thrust_t, yaw) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 14, offsetof(mavlink_set_roll_pitch_yaw_thrust_t, thrust) }, \
         } \
}


/**
 * @brief Pack a set_roll_pitch_yaw_thrust message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_thrust_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, Single roll, Single pitch, Single yaw, Single thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, roll);
	_mav_put_Single(buf, 6, pitch);
	_mav_put_Single(buf, 10, yaw);
	_mav_put_Single(buf, 14, thrust);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_set_roll_pitch_yaw_thrust_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST;
	return mavlink_finalize_message(msg, system_id, component_id, 18);
}

/**
 * @brief Pack a set_roll_pitch_yaw_thrust message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_thrust_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,Single roll,Single pitch,Single yaw,Single thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, roll);
	_mav_put_Single(buf, 6, pitch);
	_mav_put_Single(buf, 10, yaw);
	_mav_put_Single(buf, 14, thrust);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_set_roll_pitch_yaw_thrust_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18);
}

/**
 * @brief Encode a set_roll_pitch_yaw_thrust struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_roll_pitch_yaw_thrust C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_thrust_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_roll_pitch_yaw_thrust_t* set_roll_pitch_yaw_thrust)
{
	return mavlink_msg_set_roll_pitch_yaw_thrust_pack(system_id, component_id, msg, set_roll_pitch_yaw_thrust->target_system, set_roll_pitch_yaw_thrust->target_component, set_roll_pitch_yaw_thrust->roll, set_roll_pitch_yaw_thrust->pitch, set_roll_pitch_yaw_thrust->yaw, set_roll_pitch_yaw_thrust->thrust);
}

/**
 * @brief Send a set_roll_pitch_yaw_thrust message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_roll_pitch_yaw_thrust_send(mavlink_channel_t chan, byte target_system, byte target_component, Single roll, Single pitch, Single yaw, Single thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, roll);
	_mav_put_Single(buf, 6, pitch);
	_mav_put_Single(buf, 10, yaw);
	_mav_put_Single(buf, 14, thrust);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST, buf, 18);
#else
	mavlink_set_roll_pitch_yaw_thrust_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST, (const char *)&packet, 18);
#endif
}

#endif

// MESSAGE SET_ROLL_PITCH_YAW_THRUST UNPACKING


/**
 * @brief Get field target_system from set_roll_pitch_yaw_thrust message
 *
 * @return System ID
 */
static inline byte mavlink_msg_set_roll_pitch_yaw_thrust_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from set_roll_pitch_yaw_thrust message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_set_roll_pitch_yaw_thrust_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field roll from set_roll_pitch_yaw_thrust message
 *
 * @return Desired roll angle in radians
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_thrust_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  2);
}

/**
 * @brief Get field pitch from set_roll_pitch_yaw_thrust message
 *
 * @return Desired pitch angle in radians
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_thrust_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  6);
}

/**
 * @brief Get field yaw from set_roll_pitch_yaw_thrust message
 *
 * @return Desired yaw angle in radians
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_thrust_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  10);
}

/**
 * @brief Get field thrust from set_roll_pitch_yaw_thrust message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
static inline Single mavlink_msg_set_roll_pitch_yaw_thrust_get_thrust(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  14);
}

/**
 * @brief Decode a set_roll_pitch_yaw_thrust message into a struct
 *
 * @param msg The message to decode
 * @param set_roll_pitch_yaw_thrust C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_roll_pitch_yaw_thrust_decode(const mavlink_message_t* msg, mavlink_set_roll_pitch_yaw_thrust_t* set_roll_pitch_yaw_thrust)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_roll_pitch_yaw_thrust->target_system = mavlink_msg_set_roll_pitch_yaw_thrust_get_target_system(msg);
	set_roll_pitch_yaw_thrust->target_component = mavlink_msg_set_roll_pitch_yaw_thrust_get_target_component(msg);
	set_roll_pitch_yaw_thrust->roll = mavlink_msg_set_roll_pitch_yaw_thrust_get_roll(msg);
	set_roll_pitch_yaw_thrust->pitch = mavlink_msg_set_roll_pitch_yaw_thrust_get_pitch(msg);
	set_roll_pitch_yaw_thrust->yaw = mavlink_msg_set_roll_pitch_yaw_thrust_get_yaw(msg);
	set_roll_pitch_yaw_thrust->thrust = mavlink_msg_set_roll_pitch_yaw_thrust_get_thrust(msg);
#else
	memcpy(set_roll_pitch_yaw_thrust, _MAV_PAYLOAD(msg), 18);
#endif
}
