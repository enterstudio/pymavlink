// MESSAGE RC_CHANNELS_TRIM PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_TRIM 35

typedef struct __mavlink_rc_channels_trim_t
{
 byte chan_id; ///< RC channel id
 UInt16 chan_min; ///< RC channel 1 min value, in microseconds
 UInt16 chan_zero; ///< RC channel 1 zero value, in microseconds
 UInt16 chan_max; ///< RC channel 1 max value, in microseconds
} mavlink_rc_channels_trim_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_TRIM_LEN 7
#define MAVLINK_MSG_ID_35_LEN 7



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_TRIM { \
	"RC_CHANNELS_TRIM", \
	4, \
	{  { "chan_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rc_channels_trim_t, chan_id) }, \
         { "chan_min", NULL, MAVLINK_TYPE_UINT16_T, 0, 1, offsetof(mavlink_rc_channels_trim_t, chan_min) }, \
         { "chan_zero", NULL, MAVLINK_TYPE_UINT16_T, 0, 3, offsetof(mavlink_rc_channels_trim_t, chan_zero) }, \
         { "chan_max", NULL, MAVLINK_TYPE_UINT16_T, 0, 5, offsetof(mavlink_rc_channels_trim_t, chan_max) }, \
         } \
}


/**
 * @brief Pack a rc_channels_trim message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan_id RC channel id
 * @param chan_min RC channel 1 min value, in microseconds
 * @param chan_zero RC channel 1 zero value, in microseconds
 * @param chan_max RC channel 1 max value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_trim_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte chan_id, UInt16 chan_min, UInt16 chan_zero, UInt16 chan_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_byte(buf, 0, chan_id);
	_mav_put_UInt16(buf, 1, chan_min);
	_mav_put_UInt16(buf, 3, chan_zero);
	_mav_put_UInt16(buf, 5, chan_max);

        memcpy(_MAV_PAYLOAD(msg), buf, 7);
#else
	mavlink_rc_channels_trim_t packet;
	packet.chan_id = chan_id;
	packet.chan_min = chan_min;
	packet.chan_zero = chan_zero;
	packet.chan_max = chan_max;

        memcpy(_MAV_PAYLOAD(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_TRIM;
	return mavlink_finalize_message(msg, system_id, component_id, 7);
}

/**
 * @brief Pack a rc_channels_trim message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param chan_id RC channel id
 * @param chan_min RC channel 1 min value, in microseconds
 * @param chan_zero RC channel 1 zero value, in microseconds
 * @param chan_max RC channel 1 max value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_trim_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte chan_id,UInt16 chan_min,UInt16 chan_zero,UInt16 chan_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_byte(buf, 0, chan_id);
	_mav_put_UInt16(buf, 1, chan_min);
	_mav_put_UInt16(buf, 3, chan_zero);
	_mav_put_UInt16(buf, 5, chan_max);

        memcpy(_MAV_PAYLOAD(msg), buf, 7);
#else
	mavlink_rc_channels_trim_t packet;
	packet.chan_id = chan_id;
	packet.chan_min = chan_min;
	packet.chan_zero = chan_zero;
	packet.chan_max = chan_max;

        memcpy(_MAV_PAYLOAD(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_TRIM;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 7);
}

/**
 * @brief Encode a rc_channels_trim struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_trim C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_trim_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_trim_t* rc_channels_trim)
{
	return mavlink_msg_rc_channels_trim_pack(system_id, component_id, msg, rc_channels_trim->chan_id, rc_channels_trim->chan_min, rc_channels_trim->chan_zero, rc_channels_trim->chan_max);
}

/**
 * @brief Send a rc_channels_trim message
 * @param chan MAVLink channel to send the message
 *
 * @param chan_id RC channel id
 * @param chan_min RC channel 1 min value, in microseconds
 * @param chan_zero RC channel 1 zero value, in microseconds
 * @param chan_max RC channel 1 max value, in microseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_trim_send(mavlink_channel_t chan, byte chan_id, UInt16 chan_min, UInt16 chan_zero, UInt16 chan_max)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_byte(buf, 0, chan_id);
	_mav_put_UInt16(buf, 1, chan_min);
	_mav_put_UInt16(buf, 3, chan_zero);
	_mav_put_UInt16(buf, 5, chan_max);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_TRIM, buf, 7);
#else
	mavlink_rc_channels_trim_t packet;
	packet.chan_id = chan_id;
	packet.chan_min = chan_min;
	packet.chan_zero = chan_zero;
	packet.chan_max = chan_max;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_TRIM, (const char *)&packet, 7);
#endif
}

#endif

// MESSAGE RC_CHANNELS_TRIM UNPACKING


/**
 * @brief Get field chan_id from rc_channels_trim message
 *
 * @return RC channel id
 */
static inline byte mavlink_msg_rc_channels_trim_get_chan_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field chan_min from rc_channels_trim message
 *
 * @return RC channel 1 min value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_trim_get_chan_min(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  1);
}

/**
 * @brief Get field chan_zero from rc_channels_trim message
 *
 * @return RC channel 1 zero value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_trim_get_chan_zero(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  3);
}

/**
 * @brief Get field chan_max from rc_channels_trim message
 *
 * @return RC channel 1 max value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_trim_get_chan_max(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  5);
}

/**
 * @brief Decode a rc_channels_trim message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_trim C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_trim_decode(const mavlink_message_t* msg, mavlink_rc_channels_trim_t* rc_channels_trim)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_trim->chan_id = mavlink_msg_rc_channels_trim_get_chan_id(msg);
	rc_channels_trim->chan_min = mavlink_msg_rc_channels_trim_get_chan_min(msg);
	rc_channels_trim->chan_zero = mavlink_msg_rc_channels_trim_get_chan_zero(msg);
	rc_channels_trim->chan_max = mavlink_msg_rc_channels_trim_get_chan_max(msg);
#else
	memcpy(rc_channels_trim, _MAV_PAYLOAD(msg), 7);
#endif
}
