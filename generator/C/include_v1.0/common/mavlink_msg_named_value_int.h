// MESSAGE NAMED_VALUE_INT PACKING

#define MAVLINK_MSG_ID_NAMED_VALUE_INT 253

typedef struct __mavlink_named_value_int_t
{
 int32_t value; ///< Signed integer value
 char name[10]; ///< Name of the debug variable
} mavlink_named_value_int_t;

/**
 * @brief Pack a named_value_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char name[10], int32_t value)
{
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;

	put_int32_t_by_index(value, 0,  msg->payload); // Signed integer value
	put_char_array_by_index(name, 4, 10,  msg->payload); // Name of the debug variable

	return mavlink_finalize_message(msg, system_id, component_id, 14, 22);
}

/**
 * @brief Pack a named_value_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_value_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char name[10],int32_t value)
{
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;

	put_int32_t_by_index(value, 0,  msg->payload); // Signed integer value
	put_char_array_by_index(name, 4, 10,  msg->payload); // Name of the debug variable

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 22);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a named_value_int message on a channel and send
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the debug variable
 * @param value Signed integer value
 */
static inline void mavlink_msg_named_value_int_pack_chan_send(mavlink_channel_t chan,
							   mavlink_message_t* msg,
						           const char name[10],int32_t value)
{
	msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;

	put_int32_t_by_index(value, 0,  msg->payload); // Signed integer value
	put_char_array_by_index(name, 4, 10,  msg->payload); // Name of the debug variable

	mavlink_finalize_message_chan_send(msg, chan, 14, 22);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS


/**
 * @brief Encode a named_value_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_value_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_int_t* named_value_int)
{
	return mavlink_msg_named_value_int_pack(system_id, component_id, msg, named_value_int->name, named_value_int->value);
}

/**
 * @brief Send a named_value_int message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the debug variable
 * @param value Signed integer value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_int_send(mavlink_channel_t chan, const char name[10], int32_t value)
{
	MAVLINK_ALIGNED_MESSAGE(msg, 14);
	mavlink_msg_named_value_int_pack_chan_send(chan, msg, name, value);
}

#endif

// MESSAGE NAMED_VALUE_INT UNPACKING


/**
 * @brief Get field name from named_value_int message
 *
 * @return Name of the debug variable
 */
static inline uint16_t mavlink_msg_named_value_int_get_name(const mavlink_message_t* msg, char *name)
{
	return MAVLINK_MSG_RETURN_char_array(msg, name, 10,  4);
}

/**
 * @brief Get field value from named_value_int message
 *
 * @return Signed integer value
 */
static inline int32_t mavlink_msg_named_value_int_get_value(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_int32_t(msg,  0);
}

/**
 * @brief Decode a named_value_int message into a struct
 *
 * @param msg The message to decode
 * @param named_value_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_value_int_decode(const mavlink_message_t* msg, mavlink_named_value_int_t* named_value_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	named_value_int->value = mavlink_msg_named_value_int_get_value(msg);
	mavlink_msg_named_value_int_get_name(msg, named_value_int->name);
#else
	memcpy(named_value_int, msg->payload, 14);
#endif
}