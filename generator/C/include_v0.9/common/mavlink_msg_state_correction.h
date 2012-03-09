// MESSAGE STATE_CORRECTION PACKING

#define MAVLINK_MSG_ID_STATE_CORRECTION 64

typedef struct __mavlink_state_correction_t
{
 Single xErr; ///< x position error
 Single yErr; ///< y position error
 Single zErr; ///< z position error
 Single rollErr; ///< roll error (radians)
 Single pitchErr; ///< pitch error (radians)
 Single yawErr; ///< yaw error (radians)
 Single vxErr; ///< x velocity
 Single vyErr; ///< y velocity
 Single vzErr; ///< z velocity
} mavlink_state_correction_t;

#define MAVLINK_MSG_ID_STATE_CORRECTION_LEN 36
#define MAVLINK_MSG_ID_64_LEN 36



#define MAVLINK_MESSAGE_INFO_STATE_CORRECTION { \
	"STATE_CORRECTION", \
	9, \
	{  { "xErr", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_state_correction_t, xErr) }, \
         { "yErr", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_state_correction_t, yErr) }, \
         { "zErr", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_state_correction_t, zErr) }, \
         { "rollErr", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_state_correction_t, rollErr) }, \
         { "pitchErr", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_state_correction_t, pitchErr) }, \
         { "yawErr", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_state_correction_t, yawErr) }, \
         { "vxErr", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_state_correction_t, vxErr) }, \
         { "vyErr", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_state_correction_t, vyErr) }, \
         { "vzErr", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_state_correction_t, vzErr) }, \
         } \
}


/**
 * @brief Pack a state_correction message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param xErr x position error
 * @param yErr y position error
 * @param zErr z position error
 * @param rollErr roll error (radians)
 * @param pitchErr pitch error (radians)
 * @param yawErr yaw error (radians)
 * @param vxErr x velocity
 * @param vyErr y velocity
 * @param vzErr z velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_correction_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       Single xErr, Single yErr, Single zErr, Single rollErr, Single pitchErr, Single yawErr, Single vxErr, Single vyErr, Single vzErr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_Single(buf, 0, xErr);
	_mav_put_Single(buf, 4, yErr);
	_mav_put_Single(buf, 8, zErr);
	_mav_put_Single(buf, 12, rollErr);
	_mav_put_Single(buf, 16, pitchErr);
	_mav_put_Single(buf, 20, yawErr);
	_mav_put_Single(buf, 24, vxErr);
	_mav_put_Single(buf, 28, vyErr);
	_mav_put_Single(buf, 32, vzErr);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_state_correction_t packet;
	packet.xErr = xErr;
	packet.yErr = yErr;
	packet.zErr = zErr;
	packet.rollErr = rollErr;
	packet.pitchErr = pitchErr;
	packet.yawErr = yawErr;
	packet.vxErr = vxErr;
	packet.vyErr = vyErr;
	packet.vzErr = vzErr;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_CORRECTION;
	return mavlink_finalize_message(msg, system_id, component_id, 36);
}

/**
 * @brief Pack a state_correction message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param xErr x position error
 * @param yErr y position error
 * @param zErr z position error
 * @param rollErr roll error (radians)
 * @param pitchErr pitch error (radians)
 * @param yawErr yaw error (radians)
 * @param vxErr x velocity
 * @param vyErr y velocity
 * @param vzErr z velocity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_state_correction_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           Single xErr,Single yErr,Single zErr,Single rollErr,Single pitchErr,Single yawErr,Single vxErr,Single vyErr,Single vzErr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_Single(buf, 0, xErr);
	_mav_put_Single(buf, 4, yErr);
	_mav_put_Single(buf, 8, zErr);
	_mav_put_Single(buf, 12, rollErr);
	_mav_put_Single(buf, 16, pitchErr);
	_mav_put_Single(buf, 20, yawErr);
	_mav_put_Single(buf, 24, vxErr);
	_mav_put_Single(buf, 28, vyErr);
	_mav_put_Single(buf, 32, vzErr);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_state_correction_t packet;
	packet.xErr = xErr;
	packet.yErr = yErr;
	packet.zErr = zErr;
	packet.rollErr = rollErr;
	packet.pitchErr = pitchErr;
	packet.yawErr = yawErr;
	packet.vxErr = vxErr;
	packet.vyErr = vyErr;
	packet.vzErr = vzErr;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATE_CORRECTION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36);
}

/**
 * @brief Encode a state_correction struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param state_correction C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_state_correction_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_state_correction_t* state_correction)
{
	return mavlink_msg_state_correction_pack(system_id, component_id, msg, state_correction->xErr, state_correction->yErr, state_correction->zErr, state_correction->rollErr, state_correction->pitchErr, state_correction->yawErr, state_correction->vxErr, state_correction->vyErr, state_correction->vzErr);
}

/**
 * @brief Send a state_correction message
 * @param chan MAVLink channel to send the message
 *
 * @param xErr x position error
 * @param yErr y position error
 * @param zErr z position error
 * @param rollErr roll error (radians)
 * @param pitchErr pitch error (radians)
 * @param yawErr yaw error (radians)
 * @param vxErr x velocity
 * @param vyErr y velocity
 * @param vzErr z velocity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_state_correction_send(mavlink_channel_t chan, Single xErr, Single yErr, Single zErr, Single rollErr, Single pitchErr, Single yawErr, Single vxErr, Single vyErr, Single vzErr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_Single(buf, 0, xErr);
	_mav_put_Single(buf, 4, yErr);
	_mav_put_Single(buf, 8, zErr);
	_mav_put_Single(buf, 12, rollErr);
	_mav_put_Single(buf, 16, pitchErr);
	_mav_put_Single(buf, 20, yawErr);
	_mav_put_Single(buf, 24, vxErr);
	_mav_put_Single(buf, 28, vyErr);
	_mav_put_Single(buf, 32, vzErr);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_CORRECTION, buf, 36);
#else
	mavlink_state_correction_t packet;
	packet.xErr = xErr;
	packet.yErr = yErr;
	packet.zErr = zErr;
	packet.rollErr = rollErr;
	packet.pitchErr = pitchErr;
	packet.yawErr = yawErr;
	packet.vxErr = vxErr;
	packet.vyErr = vyErr;
	packet.vzErr = vzErr;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATE_CORRECTION, (const char *)&packet, 36);
#endif
}

#endif

// MESSAGE STATE_CORRECTION UNPACKING


/**
 * @brief Get field xErr from state_correction message
 *
 * @return x position error
 */
static inline Single mavlink_msg_state_correction_get_xErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  0);
}

/**
 * @brief Get field yErr from state_correction message
 *
 * @return y position error
 */
static inline Single mavlink_msg_state_correction_get_yErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  4);
}

/**
 * @brief Get field zErr from state_correction message
 *
 * @return z position error
 */
static inline Single mavlink_msg_state_correction_get_zErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  8);
}

/**
 * @brief Get field rollErr from state_correction message
 *
 * @return roll error (radians)
 */
static inline Single mavlink_msg_state_correction_get_rollErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  12);
}

/**
 * @brief Get field pitchErr from state_correction message
 *
 * @return pitch error (radians)
 */
static inline Single mavlink_msg_state_correction_get_pitchErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  16);
}

/**
 * @brief Get field yawErr from state_correction message
 *
 * @return yaw error (radians)
 */
static inline Single mavlink_msg_state_correction_get_yawErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  20);
}

/**
 * @brief Get field vxErr from state_correction message
 *
 * @return x velocity
 */
static inline Single mavlink_msg_state_correction_get_vxErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  24);
}

/**
 * @brief Get field vyErr from state_correction message
 *
 * @return y velocity
 */
static inline Single mavlink_msg_state_correction_get_vyErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  28);
}

/**
 * @brief Get field vzErr from state_correction message
 *
 * @return z velocity
 */
static inline Single mavlink_msg_state_correction_get_vzErr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  32);
}

/**
 * @brief Decode a state_correction message into a struct
 *
 * @param msg The message to decode
 * @param state_correction C-struct to decode the message contents into
 */
static inline void mavlink_msg_state_correction_decode(const mavlink_message_t* msg, mavlink_state_correction_t* state_correction)
{
#if MAVLINK_NEED_BYTE_SWAP
	state_correction->xErr = mavlink_msg_state_correction_get_xErr(msg);
	state_correction->yErr = mavlink_msg_state_correction_get_yErr(msg);
	state_correction->zErr = mavlink_msg_state_correction_get_zErr(msg);
	state_correction->rollErr = mavlink_msg_state_correction_get_rollErr(msg);
	state_correction->pitchErr = mavlink_msg_state_correction_get_pitchErr(msg);
	state_correction->yawErr = mavlink_msg_state_correction_get_yawErr(msg);
	state_correction->vxErr = mavlink_msg_state_correction_get_vxErr(msg);
	state_correction->vyErr = mavlink_msg_state_correction_get_vyErr(msg);
	state_correction->vzErr = mavlink_msg_state_correction_get_vzErr(msg);
#else
	memcpy(state_correction, _MAV_PAYLOAD(msg), 36);
#endif
}
