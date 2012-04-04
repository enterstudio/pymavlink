// MESSAGE RC_CHANNELS_TRIM_SET PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET 14

typedef struct __mavlink_rc_channels_trim_set_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 byte chan_id; ///< RC channel id
 UInt16 chan_min; ///< RC channel 1 min value, in microseconds
 UInt16 chan_zero; ///< RC channel 1 zero value, in microseconds
 UInt16 chan_max; ///< RC channel 1 max value, in microseconds
} mavlink_rc_channels_trim_set_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET_LEN 9
#define MAVLINK_MSG_ID_14_LEN 9



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_TRIM_SET { \
	"RC_CHANNELS_TRIM_SET", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rc_channels_trim_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rc_channels_trim_set_t, target_component) }, \
         { "chan_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rc_channels_trim_set_t, chan_id) }, \
         { "chan_min", NULL, MAVLINK_TYPE_UINT16_T, 0, 3, offsetof(mavlink_rc_channels_trim_set_t, chan_min) }, \
         { "chan_zero", NULL, MAVLINK_TYPE_UINT16_T, 0, 5, offsetof(mavlink_rc_channels_trim_set_t, chan_zero) }, \
         { "chan_max", NULL, MAVLINK_TYPE_UINT16_T, 0, 7, offsetof(mavlink_rc_channels_trim_set_t, chan_max) }, \
         } \
}


/**
 * @brief Pack a rc_channels_trim_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan_id RC channel id
 * @param chan_min RC channel 1 min value, in microseconds
 * @param chan_zero RC channel 1 zero value, in microseconds
 * @param chan_max RC channel 1 max value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_trim_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, byte chan_id, UInt16 chan_min, UInt16 chan_zero, UInt16 chan_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, chan_id);
	_mav_put_UInt16(buf, 3, chan_min);
	_mav_put_UInt16(buf, 5, chan_zero);
	_mav_put_UInt16(buf, 7, chan_max);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_rc_channels_trim_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.chan_id = chan_id;
	packet.chan_min = chan_min;
	packet.chan_zero = chan_zero;
	packet.chan_max = chan_max;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET;
	return mavlink_finalize_message(msg, system_id, component_id, 9);
}

/**
 * @brief Pack a rc_channels_trim_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan_id RC channel id
 * @param chan_min RC channel 1 min value, in microseconds
 * @param chan_zero RC channel 1 zero value, in microseconds
 * @param chan_max RC channel 1 max value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_trim_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,byte chan_id,UInt16 chan_min,UInt16 chan_zero,UInt16 chan_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, chan_id);
	_mav_put_UInt16(buf, 3, chan_min);
	_mav_put_UInt16(buf, 5, chan_zero);
	_mav_put_UInt16(buf, 7, chan_max);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_rc_channels_trim_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.chan_id = chan_id;
	packet.chan_min = chan_min;
	packet.chan_zero = chan_zero;
	packet.chan_max = chan_max;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9);
}

/**
 * @brief Encode a rc_channels_trim_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_trim_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_trim_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_trim_set_t* rc_channels_trim_set)
{
	return mavlink_msg_rc_channels_trim_set_pack(system_id, component_id, msg, rc_channels_trim_set->target_system, rc_channels_trim_set->target_component, rc_channels_trim_set->chan_id, rc_channels_trim_set->chan_min, rc_channels_trim_set->chan_zero, rc_channels_trim_set->chan_max);
}

/**
 * @brief Send a rc_channels_trim_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan_id RC channel id
 * @param chan_min RC channel 1 min value, in microseconds
 * @param chan_zero RC channel 1 zero value, in microseconds
 * @param chan_max RC channel 1 max value, in microseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_trim_set_send(mavlink_channel_t chan, byte target_system, byte target_component, byte chan_id, UInt16 chan_min, UInt16 chan_zero, UInt16 chan_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, chan_id);
	_mav_put_UInt16(buf, 3, chan_min);
	_mav_put_UInt16(buf, 5, chan_zero);
	_mav_put_UInt16(buf, 7, chan_max);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET, buf, 9);
#else
	mavlink_rc_channels_trim_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.chan_id = chan_id;
	packet.chan_min = chan_min;
	packet.chan_zero = chan_zero;
	packet.chan_max = chan_max;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_TRIM_SET, (const char *)&packet, 9);
#endif
}

#endif

// MESSAGE RC_CHANNELS_TRIM_SET UNPACKING


/**
 * @brief Get field target_system from rc_channels_trim_set message
 *
 * @return System ID
 */
static inline byte mavlink_msg_rc_channels_trim_set_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from rc_channels_trim_set message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_rc_channels_trim_set_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field chan_id from rc_channels_trim_set message
 *
 * @return RC channel id
 */
static inline byte mavlink_msg_rc_channels_trim_set_get_chan_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Get field chan_min from rc_channels_trim_set message
 *
 * @return RC channel 1 min value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_trim_set_get_chan_min(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  3);
}

/**
 * @brief Get field chan_zero from rc_channels_trim_set message
 *
 * @return RC channel 1 zero value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_trim_set_get_chan_zero(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  5);
}

/**
 * @brief Get field chan_max from rc_channels_trim_set message
 *
 * @return RC channel 1 max value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_trim_set_get_chan_max(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  7);
}

/**
 * @brief Decode a rc_channels_trim_set message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_trim_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_trim_set_decode(const mavlink_message_t* msg, mavlink_rc_channels_trim_set_t* rc_channels_trim_set)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_trim_set->target_system = mavlink_msg_rc_channels_trim_set_get_target_system(msg);
	rc_channels_trim_set->target_component = mavlink_msg_rc_channels_trim_set_get_target_component(msg);
	rc_channels_trim_set->chan_id = mavlink_msg_rc_channels_trim_set_get_chan_id(msg);
	rc_channels_trim_set->chan_min = mavlink_msg_rc_channels_trim_set_get_chan_min(msg);
	rc_channels_trim_set->chan_zero = mavlink_msg_rc_channels_trim_set_get_chan_zero(msg);
	rc_channels_trim_set->chan_max = mavlink_msg_rc_channels_trim_set_get_chan_max(msg);
#else
	memcpy(rc_channels_trim_set, _MAV_PAYLOAD(msg), 9);
#endif
}
