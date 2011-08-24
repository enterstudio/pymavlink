// MESSAGE SERVO_OUTPUT_RAW PACKING

#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 37

typedef struct __mavlink_servo_output_raw_t
{
 uint16_t servo1_raw; ///< Servo output 1 value, in microseconds
 uint16_t servo2_raw; ///< Servo output 2 value, in microseconds
 uint16_t servo3_raw; ///< Servo output 3 value, in microseconds
 uint16_t servo4_raw; ///< Servo output 4 value, in microseconds
 uint16_t servo5_raw; ///< Servo output 5 value, in microseconds
 uint16_t servo6_raw; ///< Servo output 6 value, in microseconds
 uint16_t servo7_raw; ///< Servo output 7 value, in microseconds
 uint16_t servo8_raw; ///< Servo output 8 value, in microseconds
} mavlink_servo_output_raw_t;

/**
 * @brief Pack a servo_output_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_output_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw)
{
	msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;

	put_uint16_t_by_index(servo1_raw, 0,  msg->payload); // Servo output 1 value, in microseconds
	put_uint16_t_by_index(servo2_raw, 2,  msg->payload); // Servo output 2 value, in microseconds
	put_uint16_t_by_index(servo3_raw, 4,  msg->payload); // Servo output 3 value, in microseconds
	put_uint16_t_by_index(servo4_raw, 6,  msg->payload); // Servo output 4 value, in microseconds
	put_uint16_t_by_index(servo5_raw, 8,  msg->payload); // Servo output 5 value, in microseconds
	put_uint16_t_by_index(servo6_raw, 10,  msg->payload); // Servo output 6 value, in microseconds
	put_uint16_t_by_index(servo7_raw, 12,  msg->payload); // Servo output 7 value, in microseconds
	put_uint16_t_by_index(servo8_raw, 14,  msg->payload); // Servo output 8 value, in microseconds

	return mavlink_finalize_message(msg, system_id, component_id, 16, 223);
}

/**
 * @brief Pack a servo_output_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_output_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t servo1_raw,uint16_t servo2_raw,uint16_t servo3_raw,uint16_t servo4_raw,uint16_t servo5_raw,uint16_t servo6_raw,uint16_t servo7_raw,uint16_t servo8_raw)
{
	msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;

	put_uint16_t_by_index(servo1_raw, 0,  msg->payload); // Servo output 1 value, in microseconds
	put_uint16_t_by_index(servo2_raw, 2,  msg->payload); // Servo output 2 value, in microseconds
	put_uint16_t_by_index(servo3_raw, 4,  msg->payload); // Servo output 3 value, in microseconds
	put_uint16_t_by_index(servo4_raw, 6,  msg->payload); // Servo output 4 value, in microseconds
	put_uint16_t_by_index(servo5_raw, 8,  msg->payload); // Servo output 5 value, in microseconds
	put_uint16_t_by_index(servo6_raw, 10,  msg->payload); // Servo output 6 value, in microseconds
	put_uint16_t_by_index(servo7_raw, 12,  msg->payload); // Servo output 7 value, in microseconds
	put_uint16_t_by_index(servo8_raw, 14,  msg->payload); // Servo output 8 value, in microseconds

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16, 223);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a servo_output_raw message on a channel and send
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 */
static inline void mavlink_msg_servo_output_raw_pack_chan_send(mavlink_channel_t chan,
							   mavlink_message_t* msg,
						           uint16_t servo1_raw,uint16_t servo2_raw,uint16_t servo3_raw,uint16_t servo4_raw,uint16_t servo5_raw,uint16_t servo6_raw,uint16_t servo7_raw,uint16_t servo8_raw)
{
	msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;

	put_uint16_t_by_index(servo1_raw, 0,  msg->payload); // Servo output 1 value, in microseconds
	put_uint16_t_by_index(servo2_raw, 2,  msg->payload); // Servo output 2 value, in microseconds
	put_uint16_t_by_index(servo3_raw, 4,  msg->payload); // Servo output 3 value, in microseconds
	put_uint16_t_by_index(servo4_raw, 6,  msg->payload); // Servo output 4 value, in microseconds
	put_uint16_t_by_index(servo5_raw, 8,  msg->payload); // Servo output 5 value, in microseconds
	put_uint16_t_by_index(servo6_raw, 10,  msg->payload); // Servo output 6 value, in microseconds
	put_uint16_t_by_index(servo7_raw, 12,  msg->payload); // Servo output 7 value, in microseconds
	put_uint16_t_by_index(servo8_raw, 14,  msg->payload); // Servo output 8 value, in microseconds

	mavlink_finalize_message_chan_send(msg, chan, 16, 223);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS


/**
 * @brief Encode a servo_output_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param servo_output_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_servo_output_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw)
{
	return mavlink_msg_servo_output_raw_pack(system_id, component_id, msg, servo_output_raw->servo1_raw, servo_output_raw->servo2_raw, servo_output_raw->servo3_raw, servo_output_raw->servo4_raw, servo_output_raw->servo5_raw, servo_output_raw->servo6_raw, servo_output_raw->servo7_raw, servo_output_raw->servo8_raw);
}

/**
 * @brief Send a servo_output_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw)
{
	MAVLINK_ALIGNED_MESSAGE(msg, 16);
	mavlink_msg_servo_output_raw_pack_chan_send(chan, msg, servo1_raw, servo2_raw, servo3_raw, servo4_raw, servo5_raw, servo6_raw, servo7_raw, servo8_raw);
}

#endif

// MESSAGE SERVO_OUTPUT_RAW UNPACKING


/**
 * @brief Get field servo1_raw from servo_output_raw message
 *
 * @return Servo output 1 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo1_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field servo2_raw from servo_output_raw message
 *
 * @return Servo output 2 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo2_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field servo3_raw from servo_output_raw message
 *
 * @return Servo output 3 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo3_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field servo4_raw from servo_output_raw message
 *
 * @return Servo output 4 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo4_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field servo5_raw from servo_output_raw message
 *
 * @return Servo output 5 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo5_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field servo6_raw from servo_output_raw message
 *
 * @return Servo output 6 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo6_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field servo7_raw from servo_output_raw message
 *
 * @return Servo output 7 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo7_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field servo8_raw from servo_output_raw message
 *
 * @return Servo output 8 value, in microseconds
 */
static inline uint16_t mavlink_msg_servo_output_raw_get_servo8_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Decode a servo_output_raw message into a struct
 *
 * @param msg The message to decode
 * @param servo_output_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_servo_output_raw_decode(const mavlink_message_t* msg, mavlink_servo_output_raw_t* servo_output_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	servo_output_raw->servo1_raw = mavlink_msg_servo_output_raw_get_servo1_raw(msg);
	servo_output_raw->servo2_raw = mavlink_msg_servo_output_raw_get_servo2_raw(msg);
	servo_output_raw->servo3_raw = mavlink_msg_servo_output_raw_get_servo3_raw(msg);
	servo_output_raw->servo4_raw = mavlink_msg_servo_output_raw_get_servo4_raw(msg);
	servo_output_raw->servo5_raw = mavlink_msg_servo_output_raw_get_servo5_raw(msg);
	servo_output_raw->servo6_raw = mavlink_msg_servo_output_raw_get_servo6_raw(msg);
	servo_output_raw->servo7_raw = mavlink_msg_servo_output_raw_get_servo7_raw(msg);
	servo_output_raw->servo8_raw = mavlink_msg_servo_output_raw_get_servo8_raw(msg);
#else
	memcpy(servo_output_raw, msg->payload, 16);
#endif
}