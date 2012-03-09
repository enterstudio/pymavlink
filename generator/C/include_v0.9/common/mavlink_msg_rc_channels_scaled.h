// MESSAGE RC_CHANNELS_SCALED PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED 36

typedef struct __mavlink_rc_channels_scaled_t
{
 Int16 chan1_scaled; ///< RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan2_scaled; ///< RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan3_scaled; ///< RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan4_scaled; ///< RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan5_scaled; ///< RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan6_scaled; ///< RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan7_scaled; ///< RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 Int16 chan8_scaled; ///< RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 byte rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%
} mavlink_rc_channels_scaled_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN 17
#define MAVLINK_MSG_ID_36_LEN 17



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED { \
	"RC_CHANNELS_SCALED", \
	9, \
	{  { "chan1_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_rc_channels_scaled_t, chan1_scaled) }, \
         { "chan2_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_rc_channels_scaled_t, chan2_scaled) }, \
         { "chan3_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_rc_channels_scaled_t, chan3_scaled) }, \
         { "chan4_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_rc_channels_scaled_t, chan4_scaled) }, \
         { "chan5_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_rc_channels_scaled_t, chan5_scaled) }, \
         { "chan6_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_rc_channels_scaled_t, chan6_scaled) }, \
         { "chan7_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_rc_channels_scaled_t, chan7_scaled) }, \
         { "chan8_scaled", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_rc_channels_scaled_t, chan8_scaled) }, \
         { "rssi", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_rc_channels_scaled_t, rssi) }, \
         } \
}


/**
 * @brief Pack a rc_channels_scaled message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_scaled_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       Int16 chan1_scaled, Int16 chan2_scaled, Int16 chan3_scaled, Int16 chan4_scaled, Int16 chan5_scaled, Int16 chan6_scaled, Int16 chan7_scaled, Int16 chan8_scaled, byte rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[17];
	_mav_put_Int16(buf, 0, chan1_scaled);
	_mav_put_Int16(buf, 2, chan2_scaled);
	_mav_put_Int16(buf, 4, chan3_scaled);
	_mav_put_Int16(buf, 6, chan4_scaled);
	_mav_put_Int16(buf, 8, chan5_scaled);
	_mav_put_Int16(buf, 10, chan6_scaled);
	_mav_put_Int16(buf, 12, chan7_scaled);
	_mav_put_Int16(buf, 14, chan8_scaled);
	_mav_put_byte(buf, 16, rssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 17);
#else
	mavlink_rc_channels_scaled_t packet;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 17);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_SCALED;
	return mavlink_finalize_message(msg, system_id, component_id, 17);
}

/**
 * @brief Pack a rc_channels_scaled message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_scaled_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           Int16 chan1_scaled,Int16 chan2_scaled,Int16 chan3_scaled,Int16 chan4_scaled,Int16 chan5_scaled,Int16 chan6_scaled,Int16 chan7_scaled,Int16 chan8_scaled,byte rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[17];
	_mav_put_Int16(buf, 0, chan1_scaled);
	_mav_put_Int16(buf, 2, chan2_scaled);
	_mav_put_Int16(buf, 4, chan3_scaled);
	_mav_put_Int16(buf, 6, chan4_scaled);
	_mav_put_Int16(buf, 8, chan5_scaled);
	_mav_put_Int16(buf, 10, chan6_scaled);
	_mav_put_Int16(buf, 12, chan7_scaled);
	_mav_put_Int16(buf, 14, chan8_scaled);
	_mav_put_byte(buf, 16, rssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 17);
#else
	mavlink_rc_channels_scaled_t packet;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 17);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_SCALED;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 17);
}

/**
 * @brief Encode a rc_channels_scaled struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_scaled C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_scaled_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_scaled_t* rc_channels_scaled)
{
	return mavlink_msg_rc_channels_scaled_pack(system_id, component_id, msg, rc_channels_scaled->chan1_scaled, rc_channels_scaled->chan2_scaled, rc_channels_scaled->chan3_scaled, rc_channels_scaled->chan4_scaled, rc_channels_scaled->chan5_scaled, rc_channels_scaled->chan6_scaled, rc_channels_scaled->chan7_scaled, rc_channels_scaled->chan8_scaled, rc_channels_scaled->rssi);
}

/**
 * @brief Send a rc_channels_scaled message
 * @param chan MAVLink channel to send the message
 *
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_scaled_send(mavlink_channel_t chan, Int16 chan1_scaled, Int16 chan2_scaled, Int16 chan3_scaled, Int16 chan4_scaled, Int16 chan5_scaled, Int16 chan6_scaled, Int16 chan7_scaled, Int16 chan8_scaled, byte rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[17];
	_mav_put_Int16(buf, 0, chan1_scaled);
	_mav_put_Int16(buf, 2, chan2_scaled);
	_mav_put_Int16(buf, 4, chan3_scaled);
	_mav_put_Int16(buf, 6, chan4_scaled);
	_mav_put_Int16(buf, 8, chan5_scaled);
	_mav_put_Int16(buf, 10, chan6_scaled);
	_mav_put_Int16(buf, 12, chan7_scaled);
	_mav_put_Int16(buf, 14, chan8_scaled);
	_mav_put_byte(buf, 16, rssi);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, buf, 17);
#else
	mavlink_rc_channels_scaled_t packet;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.rssi = rssi;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, (const char *)&packet, 17);
#endif
}

#endif

// MESSAGE RC_CHANNELS_SCALED UNPACKING


/**
 * @brief Get field chan1_scaled from rc_channels_scaled message
 *
 * @return RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan1_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  0);
}

/**
 * @brief Get field chan2_scaled from rc_channels_scaled message
 *
 * @return RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan2_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  2);
}

/**
 * @brief Get field chan3_scaled from rc_channels_scaled message
 *
 * @return RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan3_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  4);
}

/**
 * @brief Get field chan4_scaled from rc_channels_scaled message
 *
 * @return RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan4_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  6);
}

/**
 * @brief Get field chan5_scaled from rc_channels_scaled message
 *
 * @return RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan5_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  8);
}

/**
 * @brief Get field chan6_scaled from rc_channels_scaled message
 *
 * @return RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan6_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  10);
}

/**
 * @brief Get field chan7_scaled from rc_channels_scaled message
 *
 * @return RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan7_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  12);
}

/**
 * @brief Get field chan8_scaled from rc_channels_scaled message
 *
 * @return RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
static inline Int16 mavlink_msg_rc_channels_scaled_get_chan8_scaled(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  14);
}

/**
 * @brief Get field rssi from rc_channels_scaled message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline byte mavlink_msg_rc_channels_scaled_get_rssi(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  16);
}

/**
 * @brief Decode a rc_channels_scaled message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_scaled C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_scaled_decode(const mavlink_message_t* msg, mavlink_rc_channels_scaled_t* rc_channels_scaled)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_scaled->chan1_scaled = mavlink_msg_rc_channels_scaled_get_chan1_scaled(msg);
	rc_channels_scaled->chan2_scaled = mavlink_msg_rc_channels_scaled_get_chan2_scaled(msg);
	rc_channels_scaled->chan3_scaled = mavlink_msg_rc_channels_scaled_get_chan3_scaled(msg);
	rc_channels_scaled->chan4_scaled = mavlink_msg_rc_channels_scaled_get_chan4_scaled(msg);
	rc_channels_scaled->chan5_scaled = mavlink_msg_rc_channels_scaled_get_chan5_scaled(msg);
	rc_channels_scaled->chan6_scaled = mavlink_msg_rc_channels_scaled_get_chan6_scaled(msg);
	rc_channels_scaled->chan7_scaled = mavlink_msg_rc_channels_scaled_get_chan7_scaled(msg);
	rc_channels_scaled->chan8_scaled = mavlink_msg_rc_channels_scaled_get_chan8_scaled(msg);
	rc_channels_scaled->rssi = mavlink_msg_rc_channels_scaled_get_rssi(msg);
#else
	memcpy(rc_channels_scaled, _MAV_PAYLOAD(msg), 17);
#endif
}
