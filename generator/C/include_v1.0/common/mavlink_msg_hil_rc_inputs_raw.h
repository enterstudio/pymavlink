// MESSAGE HIL_RC_INPUTS_RAW PACKING

#define MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW 92

typedef struct __mavlink_hil_rc_inputs_raw_t
{
 uint64_t time_us; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 uint16_t chan1_raw; ///< RC channel 1 value, in microseconds
 uint16_t chan2_raw; ///< RC channel 2 value, in microseconds
 uint16_t chan3_raw; ///< RC channel 3 value, in microseconds
 uint16_t chan4_raw; ///< RC channel 4 value, in microseconds
 uint16_t chan5_raw; ///< RC channel 5 value, in microseconds
 uint16_t chan6_raw; ///< RC channel 6 value, in microseconds
 uint16_t chan7_raw; ///< RC channel 7 value, in microseconds
 uint16_t chan8_raw; ///< RC channel 8 value, in microseconds
 uint16_t chan9_raw; ///< RC channel 9 value, in microseconds
 uint16_t chan10_raw; ///< RC channel 10 value, in microseconds
 uint16_t chan11_raw; ///< RC channel 11 value, in microseconds
 uint16_t chan12_raw; ///< RC channel 12 value, in microseconds
 uint8_t rssi; ///< Receive signal strength indicator, 0: 0%, 255: 100%
} mavlink_hil_rc_inputs_raw_t;

/**
 * @brief Pack a hil_rc_inputs_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_us, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
	msg->msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;

	put_uint64_t_by_index(time_us, 0,  MAVLINK_PAYLOAD(msg)); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	put_uint16_t_by_index(chan1_raw, 8,  MAVLINK_PAYLOAD(msg)); // RC channel 1 value, in microseconds
	put_uint16_t_by_index(chan2_raw, 10,  MAVLINK_PAYLOAD(msg)); // RC channel 2 value, in microseconds
	put_uint16_t_by_index(chan3_raw, 12,  MAVLINK_PAYLOAD(msg)); // RC channel 3 value, in microseconds
	put_uint16_t_by_index(chan4_raw, 14,  MAVLINK_PAYLOAD(msg)); // RC channel 4 value, in microseconds
	put_uint16_t_by_index(chan5_raw, 16,  MAVLINK_PAYLOAD(msg)); // RC channel 5 value, in microseconds
	put_uint16_t_by_index(chan6_raw, 18,  MAVLINK_PAYLOAD(msg)); // RC channel 6 value, in microseconds
	put_uint16_t_by_index(chan7_raw, 20,  MAVLINK_PAYLOAD(msg)); // RC channel 7 value, in microseconds
	put_uint16_t_by_index(chan8_raw, 22,  MAVLINK_PAYLOAD(msg)); // RC channel 8 value, in microseconds
	put_uint16_t_by_index(chan9_raw, 24,  MAVLINK_PAYLOAD(msg)); // RC channel 9 value, in microseconds
	put_uint16_t_by_index(chan10_raw, 26,  MAVLINK_PAYLOAD(msg)); // RC channel 10 value, in microseconds
	put_uint16_t_by_index(chan11_raw, 28,  MAVLINK_PAYLOAD(msg)); // RC channel 11 value, in microseconds
	put_uint16_t_by_index(chan12_raw, 30,  MAVLINK_PAYLOAD(msg)); // RC channel 12 value, in microseconds
	put_uint8_t_by_index(rssi, 32,  MAVLINK_PAYLOAD(msg)); // Receive signal strength indicator, 0: 0%, 255: 100%

	return mavlink_finalize_message(msg, system_id, component_id, 33, 62);
}

/**
 * @brief Pack a hil_rc_inputs_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_us,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint16_t chan9_raw,uint16_t chan10_raw,uint16_t chan11_raw,uint16_t chan12_raw,uint8_t rssi)
{
	msg->msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;

	put_uint64_t_by_index(time_us, 0,  MAVLINK_PAYLOAD(msg)); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	put_uint16_t_by_index(chan1_raw, 8,  MAVLINK_PAYLOAD(msg)); // RC channel 1 value, in microseconds
	put_uint16_t_by_index(chan2_raw, 10,  MAVLINK_PAYLOAD(msg)); // RC channel 2 value, in microseconds
	put_uint16_t_by_index(chan3_raw, 12,  MAVLINK_PAYLOAD(msg)); // RC channel 3 value, in microseconds
	put_uint16_t_by_index(chan4_raw, 14,  MAVLINK_PAYLOAD(msg)); // RC channel 4 value, in microseconds
	put_uint16_t_by_index(chan5_raw, 16,  MAVLINK_PAYLOAD(msg)); // RC channel 5 value, in microseconds
	put_uint16_t_by_index(chan6_raw, 18,  MAVLINK_PAYLOAD(msg)); // RC channel 6 value, in microseconds
	put_uint16_t_by_index(chan7_raw, 20,  MAVLINK_PAYLOAD(msg)); // RC channel 7 value, in microseconds
	put_uint16_t_by_index(chan8_raw, 22,  MAVLINK_PAYLOAD(msg)); // RC channel 8 value, in microseconds
	put_uint16_t_by_index(chan9_raw, 24,  MAVLINK_PAYLOAD(msg)); // RC channel 9 value, in microseconds
	put_uint16_t_by_index(chan10_raw, 26,  MAVLINK_PAYLOAD(msg)); // RC channel 10 value, in microseconds
	put_uint16_t_by_index(chan11_raw, 28,  MAVLINK_PAYLOAD(msg)); // RC channel 11 value, in microseconds
	put_uint16_t_by_index(chan12_raw, 30,  MAVLINK_PAYLOAD(msg)); // RC channel 12 value, in microseconds
	put_uint8_t_by_index(rssi, 32,  MAVLINK_PAYLOAD(msg)); // Receive signal strength indicator, 0: 0%, 255: 100%

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 33, 62);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a hil_rc_inputs_raw message on a channel and send
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline void mavlink_msg_hil_rc_inputs_raw_pack_chan_send(mavlink_channel_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_us,uint16_t chan1_raw,uint16_t chan2_raw,uint16_t chan3_raw,uint16_t chan4_raw,uint16_t chan5_raw,uint16_t chan6_raw,uint16_t chan7_raw,uint16_t chan8_raw,uint16_t chan9_raw,uint16_t chan10_raw,uint16_t chan11_raw,uint16_t chan12_raw,uint8_t rssi)
{
	msg->msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;

	put_uint64_t_by_index(time_us, 0,  MAVLINK_PAYLOAD(msg)); // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	put_uint16_t_by_index(chan1_raw, 8,  MAVLINK_PAYLOAD(msg)); // RC channel 1 value, in microseconds
	put_uint16_t_by_index(chan2_raw, 10,  MAVLINK_PAYLOAD(msg)); // RC channel 2 value, in microseconds
	put_uint16_t_by_index(chan3_raw, 12,  MAVLINK_PAYLOAD(msg)); // RC channel 3 value, in microseconds
	put_uint16_t_by_index(chan4_raw, 14,  MAVLINK_PAYLOAD(msg)); // RC channel 4 value, in microseconds
	put_uint16_t_by_index(chan5_raw, 16,  MAVLINK_PAYLOAD(msg)); // RC channel 5 value, in microseconds
	put_uint16_t_by_index(chan6_raw, 18,  MAVLINK_PAYLOAD(msg)); // RC channel 6 value, in microseconds
	put_uint16_t_by_index(chan7_raw, 20,  MAVLINK_PAYLOAD(msg)); // RC channel 7 value, in microseconds
	put_uint16_t_by_index(chan8_raw, 22,  MAVLINK_PAYLOAD(msg)); // RC channel 8 value, in microseconds
	put_uint16_t_by_index(chan9_raw, 24,  MAVLINK_PAYLOAD(msg)); // RC channel 9 value, in microseconds
	put_uint16_t_by_index(chan10_raw, 26,  MAVLINK_PAYLOAD(msg)); // RC channel 10 value, in microseconds
	put_uint16_t_by_index(chan11_raw, 28,  MAVLINK_PAYLOAD(msg)); // RC channel 11 value, in microseconds
	put_uint16_t_by_index(chan12_raw, 30,  MAVLINK_PAYLOAD(msg)); // RC channel 12 value, in microseconds
	put_uint8_t_by_index(rssi, 32,  MAVLINK_PAYLOAD(msg)); // Receive signal strength indicator, 0: 0%, 255: 100%

	mavlink_finalize_message_chan_send(msg, chan, 33, 62);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS


/**
 * @brief Encode a hil_rc_inputs_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_rc_inputs_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_rc_inputs_raw_t* hil_rc_inputs_raw)
{
	return mavlink_msg_hil_rc_inputs_raw_pack(system_id, component_id, msg, hil_rc_inputs_raw->time_us, hil_rc_inputs_raw->chan1_raw, hil_rc_inputs_raw->chan2_raw, hil_rc_inputs_raw->chan3_raw, hil_rc_inputs_raw->chan4_raw, hil_rc_inputs_raw->chan5_raw, hil_rc_inputs_raw->chan6_raw, hil_rc_inputs_raw->chan7_raw, hil_rc_inputs_raw->chan8_raw, hil_rc_inputs_raw->chan9_raw, hil_rc_inputs_raw->chan10_raw, hil_rc_inputs_raw->chan11_raw, hil_rc_inputs_raw->chan12_raw, hil_rc_inputs_raw->rssi);
}

/**
 * @brief Send a hil_rc_inputs_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_rc_inputs_raw_send(mavlink_channel_t chan, uint64_t time_us, uint16_t chan1_raw, uint16_t chan2_raw, uint16_t chan3_raw, uint16_t chan4_raw, uint16_t chan5_raw, uint16_t chan6_raw, uint16_t chan7_raw, uint16_t chan8_raw, uint16_t chan9_raw, uint16_t chan10_raw, uint16_t chan11_raw, uint16_t chan12_raw, uint8_t rssi)
{
	MAVLINK_ALIGNED_MESSAGE(msg, 33);
	mavlink_msg_hil_rc_inputs_raw_pack_chan_send(chan, msg, time_us, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw, chan9_raw, chan10_raw, chan11_raw, chan12_raw, rssi);
}

#endif

// MESSAGE HIL_RC_INPUTS_RAW UNPACKING


/**
 * @brief Get field time_us from hil_rc_inputs_raw message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline uint64_t mavlink_msg_hil_rc_inputs_raw_get_time_us(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field chan1_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 1 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan1_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field chan2_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 2 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan2_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field chan3_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan3_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field chan4_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 4 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan4_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field chan5_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 5 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan5_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field chan6_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 6 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan6_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field chan7_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 7 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan7_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field chan8_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 8 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan8_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field chan9_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 9 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan9_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field chan10_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 10 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan10_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field chan11_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 11 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan11_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field chan12_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 12 value, in microseconds
 */
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_get_chan12_raw(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field rssi from hil_rc_inputs_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
static inline uint8_t mavlink_msg_hil_rc_inputs_raw_get_rssi(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Decode a hil_rc_inputs_raw message into a struct
 *
 * @param msg The message to decode
 * @param hil_rc_inputs_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_rc_inputs_raw_decode(const mavlink_message_t* msg, mavlink_hil_rc_inputs_raw_t* hil_rc_inputs_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	hil_rc_inputs_raw->time_us = mavlink_msg_hil_rc_inputs_raw_get_time_us(msg);
	hil_rc_inputs_raw->chan1_raw = mavlink_msg_hil_rc_inputs_raw_get_chan1_raw(msg);
	hil_rc_inputs_raw->chan2_raw = mavlink_msg_hil_rc_inputs_raw_get_chan2_raw(msg);
	hil_rc_inputs_raw->chan3_raw = mavlink_msg_hil_rc_inputs_raw_get_chan3_raw(msg);
	hil_rc_inputs_raw->chan4_raw = mavlink_msg_hil_rc_inputs_raw_get_chan4_raw(msg);
	hil_rc_inputs_raw->chan5_raw = mavlink_msg_hil_rc_inputs_raw_get_chan5_raw(msg);
	hil_rc_inputs_raw->chan6_raw = mavlink_msg_hil_rc_inputs_raw_get_chan6_raw(msg);
	hil_rc_inputs_raw->chan7_raw = mavlink_msg_hil_rc_inputs_raw_get_chan7_raw(msg);
	hil_rc_inputs_raw->chan8_raw = mavlink_msg_hil_rc_inputs_raw_get_chan8_raw(msg);
	hil_rc_inputs_raw->chan9_raw = mavlink_msg_hil_rc_inputs_raw_get_chan9_raw(msg);
	hil_rc_inputs_raw->chan10_raw = mavlink_msg_hil_rc_inputs_raw_get_chan10_raw(msg);
	hil_rc_inputs_raw->chan11_raw = mavlink_msg_hil_rc_inputs_raw_get_chan11_raw(msg);
	hil_rc_inputs_raw->chan12_raw = mavlink_msg_hil_rc_inputs_raw_get_chan12_raw(msg);
	hil_rc_inputs_raw->rssi = mavlink_msg_hil_rc_inputs_raw_get_rssi(msg);
#else
	memcpy(hil_rc_inputs_raw, MAVLINK_PAYLOAD(msg), 33);
#endif
}