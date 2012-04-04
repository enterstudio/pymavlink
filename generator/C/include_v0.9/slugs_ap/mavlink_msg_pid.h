// MESSAGE PID PACKING

#define MAVLINK_MSG_ID_PID 182

typedef struct __mavlink_pid_t
{
 byte target; ///< The system setting the PID values
 Single pVal; ///< Proportional gain
 Single iVal; ///< Integral gain
 Single dVal; ///< Derivative gain
 byte idx; ///< PID loop index
} mavlink_pid_t;

#define MAVLINK_MSG_ID_PID_LEN 14
#define MAVLINK_MSG_ID_182_LEN 14



#define MAVLINK_MESSAGE_INFO_PID { \
	"PID", \
	5, \
	{  { "target", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_pid_t, target) }, \
         { "pVal", NULL, MAVLINK_TYPE_FLOAT, 0, 1, offsetof(mavlink_pid_t, pVal) }, \
         { "iVal", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_pid_t, iVal) }, \
         { "dVal", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_pid_t, dVal) }, \
         { "idx", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_pid_t, idx) }, \
         } \
}


/**
 * @brief Pack a pid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the PID values
 * @param pVal Proportional gain
 * @param iVal Integral gain
 * @param dVal Derivative gain
 * @param idx PID loop index
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target, Single pVal, Single iVal, Single dVal, byte idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_byte(buf, 0, target);
	_mav_put_Single(buf, 1, pVal);
	_mav_put_Single(buf, 5, iVal);
	_mav_put_Single(buf, 9, dVal);
	_mav_put_byte(buf, 13, idx);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
	mavlink_pid_t packet;
	packet.target = target;
	packet.pVal = pVal;
	packet.iVal = iVal;
	packet.dVal = dVal;
	packet.idx = idx;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID;
	return mavlink_finalize_message(msg, system_id, component_id, 14);
}

/**
 * @brief Pack a pid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the PID values
 * @param pVal Proportional gain
 * @param iVal Integral gain
 * @param dVal Derivative gain
 * @param idx PID loop index
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target,Single pVal,Single iVal,Single dVal,byte idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_byte(buf, 0, target);
	_mav_put_Single(buf, 1, pVal);
	_mav_put_Single(buf, 5, iVal);
	_mav_put_Single(buf, 9, dVal);
	_mav_put_byte(buf, 13, idx);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
	mavlink_pid_t packet;
	packet.target = target;
	packet.pVal = pVal;
	packet.iVal = iVal;
	packet.dVal = dVal;
	packet.idx = idx;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14);
}

/**
 * @brief Encode a pid struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pid C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_t* pid)
{
	return mavlink_msg_pid_pack(system_id, component_id, msg, pid->target, pid->pVal, pid->iVal, pid->dVal, pid->idx);
}

/**
 * @brief Send a pid message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the PID values
 * @param pVal Proportional gain
 * @param iVal Integral gain
 * @param dVal Derivative gain
 * @param idx PID loop index
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_send(mavlink_channel_t chan, byte target, Single pVal, Single iVal, Single dVal, byte idx)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[14];
	_mav_put_byte(buf, 0, target);
	_mav_put_Single(buf, 1, pVal);
	_mav_put_Single(buf, 5, iVal);
	_mav_put_Single(buf, 9, dVal);
	_mav_put_byte(buf, 13, idx);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID, buf, 14);
#else
	mavlink_pid_t packet;
	packet.target = target;
	packet.pVal = pVal;
	packet.iVal = iVal;
	packet.dVal = dVal;
	packet.idx = idx;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID, (const char *)&packet, 14);
#endif
}

#endif

// MESSAGE PID UNPACKING


/**
 * @brief Get field target from pid message
 *
 * @return The system setting the PID values
 */
static inline byte mavlink_msg_pid_get_target(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field pVal from pid message
 *
 * @return Proportional gain
 */
static inline Single mavlink_msg_pid_get_pVal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  1);
}

/**
 * @brief Get field iVal from pid message
 *
 * @return Integral gain
 */
static inline Single mavlink_msg_pid_get_iVal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  5);
}

/**
 * @brief Get field dVal from pid message
 *
 * @return Derivative gain
 */
static inline Single mavlink_msg_pid_get_dVal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  9);
}

/**
 * @brief Get field idx from pid message
 *
 * @return PID loop index
 */
static inline byte mavlink_msg_pid_get_idx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  13);
}

/**
 * @brief Decode a pid message into a struct
 *
 * @param msg The message to decode
 * @param pid C-struct to decode the message contents into
 */
static inline void mavlink_msg_pid_decode(const mavlink_message_t* msg, mavlink_pid_t* pid)
{
#if MAVLINK_NEED_BYTE_SWAP
	pid->target = mavlink_msg_pid_get_target(msg);
	pid->pVal = mavlink_msg_pid_get_pVal(msg);
	pid->iVal = mavlink_msg_pid_get_iVal(msg);
	pid->dVal = mavlink_msg_pid_get_dVal(msg);
	pid->idx = mavlink_msg_pid_get_idx(msg);
#else
	memcpy(pid, _MAV_PAYLOAD(msg), 14);
#endif
}
