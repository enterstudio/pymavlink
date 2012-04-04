// MESSAGE PID PACKING

#define MAVLINK_MSG_ID_PID 34

typedef struct __mavlink_pid_t
{
 byte pid_id; ///< PID ID
 Single k_p; ///< P
 Single k_i; ///< I
 Single k_d; ///< D
} mavlink_pid_t;

#define MAVLINK_MSG_ID_PID_LEN 13
#define MAVLINK_MSG_ID_34_LEN 13



#define MAVLINK_MESSAGE_INFO_PID { \
	"PID", \
	4, \
	{  { "pid_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_pid_t, pid_id) }, \
         { "k_p", NULL, MAVLINK_TYPE_FLOAT, 0, 1, offsetof(mavlink_pid_t, k_p) }, \
         { "k_i", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_pid_t, k_i) }, \
         { "k_d", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_pid_t, k_d) }, \
         } \
}


/**
 * @brief Pack a pid message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pid_id PID ID
 * @param k_p P
 * @param k_i I
 * @param k_d D
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte pid_id, Single k_p, Single k_i, Single k_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_byte(buf, 0, pid_id);
	_mav_put_Single(buf, 1, k_p);
	_mav_put_Single(buf, 5, k_i);
	_mav_put_Single(buf, 9, k_d);

        memcpy(_MAV_PAYLOAD(msg), buf, 13);
#else
	mavlink_pid_t packet;
	packet.pid_id = pid_id;
	packet.k_p = k_p;
	packet.k_i = k_i;
	packet.k_d = k_d;

        memcpy(_MAV_PAYLOAD(msg), &packet, 13);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID;
	return mavlink_finalize_message(msg, system_id, component_id, 13);
}

/**
 * @brief Pack a pid message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param pid_id PID ID
 * @param k_p P
 * @param k_i I
 * @param k_d D
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte pid_id,Single k_p,Single k_i,Single k_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_byte(buf, 0, pid_id);
	_mav_put_Single(buf, 1, k_p);
	_mav_put_Single(buf, 5, k_i);
	_mav_put_Single(buf, 9, k_d);

        memcpy(_MAV_PAYLOAD(msg), buf, 13);
#else
	mavlink_pid_t packet;
	packet.pid_id = pid_id;
	packet.k_p = k_p;
	packet.k_i = k_i;
	packet.k_d = k_d;

        memcpy(_MAV_PAYLOAD(msg), &packet, 13);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 13);
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
	return mavlink_msg_pid_pack(system_id, component_id, msg, pid->pid_id, pid->k_p, pid->k_i, pid->k_d);
}

/**
 * @brief Send a pid message
 * @param chan MAVLink channel to send the message
 *
 * @param pid_id PID ID
 * @param k_p P
 * @param k_i I
 * @param k_d D
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_send(mavlink_channel_t chan, byte pid_id, Single k_p, Single k_i, Single k_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[13];
	_mav_put_byte(buf, 0, pid_id);
	_mav_put_Single(buf, 1, k_p);
	_mav_put_Single(buf, 5, k_i);
	_mav_put_Single(buf, 9, k_d);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID, buf, 13);
#else
	mavlink_pid_t packet;
	packet.pid_id = pid_id;
	packet.k_p = k_p;
	packet.k_i = k_i;
	packet.k_d = k_d;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID, (const char *)&packet, 13);
#endif
}

#endif

// MESSAGE PID UNPACKING


/**
 * @brief Get field pid_id from pid message
 *
 * @return PID ID
 */
static inline byte mavlink_msg_pid_get_pid_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field k_p from pid message
 *
 * @return P
 */
static inline Single mavlink_msg_pid_get_k_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  1);
}

/**
 * @brief Get field k_i from pid message
 *
 * @return I
 */
static inline Single mavlink_msg_pid_get_k_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  5);
}

/**
 * @brief Get field k_d from pid message
 *
 * @return D
 */
static inline Single mavlink_msg_pid_get_k_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  9);
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
	pid->pid_id = mavlink_msg_pid_get_pid_id(msg);
	pid->k_p = mavlink_msg_pid_get_k_p(msg);
	pid->k_i = mavlink_msg_pid_get_k_i(msg);
	pid->k_d = mavlink_msg_pid_get_k_d(msg);
#else
	memcpy(pid, _MAV_PAYLOAD(msg), 13);
#endif
}
