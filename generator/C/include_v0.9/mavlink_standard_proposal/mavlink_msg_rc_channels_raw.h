// MESSAGE RC_CHANNELS_RAW PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 56

typedef struct __mavlink_rc_channels_raw_t
{
 UInt16 chan1; ///< RC channel 1 value, in microseconds
 UInt16 chan2; ///< RC channel 2 value, in microseconds
 UInt16 chan3; ///< RC channel 3 value, in microseconds
 UInt16 chan4; ///< RC channel 3 value, in microseconds
 UInt16 chan5; ///< RC channel 3 value, in microseconds
} mavlink_rc_channels_raw_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 10
#define MAVLINK_MSG_ID_56_LEN 10



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW { \
	"RC_CHANNELS_RAW", \
	5, \
	{  { "chan1", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rc_channels_raw_t, chan1) }, \
         { "chan2", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_rc_channels_raw_t, chan2) }, \
         { "chan3", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rc_channels_raw_t, chan3) }, \
         { "chan4", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_rc_channels_raw_t, chan4) }, \
         { "chan5", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_rc_channels_raw_t, chan5) }, \
         } \
}


/**
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan1 RC channel 1 value, in microseconds
 * @param chan2 RC channel 2 value, in microseconds
 * @param chan3 RC channel 3 value, in microseconds
 * @param chan4 RC channel 3 value, in microseconds
 * @param chan5 RC channel 3 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt16 chan1, UInt16 chan2, UInt16 chan3, UInt16 chan4, UInt16 chan5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_UInt16(buf, 0, chan1);
	_mav_put_UInt16(buf, 2, chan2);
	_mav_put_UInt16(buf, 4, chan3);
	_mav_put_UInt16(buf, 6, chan4);
	_mav_put_UInt16(buf, 8, chan5);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_rc_channels_raw_t packet;
	packet.chan1 = chan1;
	packet.chan2 = chan2;
	packet.chan3 = chan3;
	packet.chan4 = chan4;
	packet.chan5 = chan5;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 10);
}

/**
 * @brief Pack a rc_channels_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param chan1 RC channel 1 value, in microseconds
 * @param chan2 RC channel 2 value, in microseconds
 * @param chan3 RC channel 3 value, in microseconds
 * @param chan4 RC channel 3 value, in microseconds
 * @param chan5 RC channel 3 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt16 chan1,UInt16 chan2,UInt16 chan3,UInt16 chan4,UInt16 chan5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_UInt16(buf, 0, chan1);
	_mav_put_UInt16(buf, 2, chan2);
	_mav_put_UInt16(buf, 4, chan3);
	_mav_put_UInt16(buf, 6, chan4);
	_mav_put_UInt16(buf, 8, chan5);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_rc_channels_raw_t packet;
	packet.chan1 = chan1;
	packet.chan2 = chan2;
	packet.chan3 = chan3;
	packet.chan4 = chan4;
	packet.chan5 = chan5;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 10);
}

/**
 * @brief Encode a rc_channels_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw)
{
	return mavlink_msg_rc_channels_raw_pack(system_id, component_id, msg, rc_channels_raw->chan1, rc_channels_raw->chan2, rc_channels_raw->chan3, rc_channels_raw->chan4, rc_channels_raw->chan5);
}

/**
 * @brief Send a rc_channels_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param chan1 RC channel 1 value, in microseconds
 * @param chan2 RC channel 2 value, in microseconds
 * @param chan3 RC channel 3 value, in microseconds
 * @param chan4 RC channel 3 value, in microseconds
 * @param chan5 RC channel 3 value, in microseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, UInt16 chan1, UInt16 chan2, UInt16 chan3, UInt16 chan4, UInt16 chan5)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_UInt16(buf, 0, chan1);
	_mav_put_UInt16(buf, 2, chan2);
	_mav_put_UInt16(buf, 4, chan3);
	_mav_put_UInt16(buf, 6, chan4);
	_mav_put_UInt16(buf, 8, chan5);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, buf, 10);
#else
	mavlink_rc_channels_raw_t packet;
	packet.chan1 = chan1;
	packet.chan2 = chan2;
	packet.chan3 = chan3;
	packet.chan4 = chan4;
	packet.chan5 = chan5;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, (const char *)&packet, 10);
#endif
}

#endif

// MESSAGE RC_CHANNELS_RAW UNPACKING


/**
 * @brief Get field chan1 from rc_channels_raw message
 *
 * @return RC channel 1 value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_raw_get_chan1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  0);
}

/**
 * @brief Get field chan2 from rc_channels_raw message
 *
 * @return RC channel 2 value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_raw_get_chan2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  2);
}

/**
 * @brief Get field chan3 from rc_channels_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_raw_get_chan3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  4);
}

/**
 * @brief Get field chan4 from rc_channels_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_raw_get_chan4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  6);
}

/**
 * @brief Get field chan5 from rc_channels_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
static inline UInt16 mavlink_msg_rc_channels_raw_get_chan5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  8);
}

/**
 * @brief Decode a rc_channels_raw message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_raw_decode(const mavlink_message_t* msg, mavlink_rc_channels_raw_t* rc_channels_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_raw->chan1 = mavlink_msg_rc_channels_raw_get_chan1(msg);
	rc_channels_raw->chan2 = mavlink_msg_rc_channels_raw_get_chan2(msg);
	rc_channels_raw->chan3 = mavlink_msg_rc_channels_raw_get_chan3(msg);
	rc_channels_raw->chan4 = mavlink_msg_rc_channels_raw_get_chan4(msg);
	rc_channels_raw->chan5 = mavlink_msg_rc_channels_raw_get_chan5(msg);
#else
	memcpy(rc_channels_raw, _MAV_PAYLOAD(msg), 10);
#endif
}
