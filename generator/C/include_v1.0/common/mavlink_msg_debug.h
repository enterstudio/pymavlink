// MESSAGE DEBUG PACKING

#define MAVLINK_MSG_ID_DEBUG 254

typedef struct __mavlink_debug_t
{
 UInt32 time_boot_ms; ///< Timestamp (milliseconds since system boot)
 Single value; ///< DEBUG value
 byte ind; ///< index of debug variable
} mavlink_debug_t;

#define MAVLINK_MSG_ID_DEBUG_LEN 9
#define MAVLINK_MSG_ID_254_LEN 9



#define MAVLINK_MESSAGE_INFO_DEBUG { \
	"DEBUG", \
	3, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_debug_t, time_boot_ms) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_debug_t, value) }, \
         { "ind", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_debug_t, ind) }, \
         } \
}


/**
 * @brief Pack a debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt32 time_boot_ms, byte ind, Single value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, value);
	_mav_put_byte(buf, 8, ind);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_debug_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.ind = ind;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
	return mavlink_finalize_message(msg, system_id, component_id, 9, 46);
}

/**
 * @brief Pack a debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt32 time_boot_ms,byte ind,Single value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, value);
	_mav_put_byte(buf, 8, ind);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_debug_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.ind = ind;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9, 46);
}

/**
 * @brief Encode a debug struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_t* debug)
{
	return mavlink_msg_debug_pack(system_id, component_id, msg, debug->time_boot_ms, debug->ind, debug->value);
}

/**
 * @brief Send a debug message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param ind index of debug variable
 * @param value DEBUG value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_send(mavlink_channel_t chan, UInt32 time_boot_ms, byte ind, Single value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, value);
	_mav_put_byte(buf, 8, ind);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, 9, 46);
#else
	mavlink_debug_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.ind = ind;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)&packet, 9, 46);
#endif
}

#endif

// MESSAGE DEBUG UNPACKING


/**
 * @brief Get field time_boot_ms from debug message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline UInt32 mavlink_msg_debug_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  0);
}

/**
 * @brief Get field ind from debug message
 *
 * @return index of debug variable
 */
static inline byte mavlink_msg_debug_get_ind(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  8);
}

/**
 * @brief Get field value from debug message
 *
 * @return DEBUG value
 */
static inline Single mavlink_msg_debug_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  4);
}

/**
 * @brief Decode a debug message into a struct
 *
 * @param msg The message to decode
 * @param debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_debug_decode(const mavlink_message_t* msg, mavlink_debug_t* debug)
{
#if MAVLINK_NEED_BYTE_SWAP
	debug->time_boot_ms = mavlink_msg_debug_get_time_boot_ms(msg);
	debug->value = mavlink_msg_debug_get_value(msg);
	debug->ind = mavlink_msg_debug_get_ind(msg);
#else
	memcpy(debug, _MAV_PAYLOAD(msg), 9);
#endif
}
