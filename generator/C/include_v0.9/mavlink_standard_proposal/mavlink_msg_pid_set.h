// MESSAGE PID_SET PACKING

#define MAVLINK_MSG_ID_PID_SET 13

typedef struct __mavlink_pid_set_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 byte pid_id; ///< PID ID
 Single k_p; ///< P
 Single k_i; ///< I
 Single k_d; ///< D
} mavlink_pid_set_t;

#define MAVLINK_MSG_ID_PID_SET_LEN 15
#define MAVLINK_MSG_ID_13_LEN 15



#define MAVLINK_MESSAGE_INFO_PID_SET { \
	"PID_SET", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_pid_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_pid_set_t, target_component) }, \
         { "pid_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_pid_set_t, pid_id) }, \
         { "k_p", NULL, MAVLINK_TYPE_FLOAT, 0, 3, offsetof(mavlink_pid_set_t, k_p) }, \
         { "k_i", NULL, MAVLINK_TYPE_FLOAT, 0, 7, offsetof(mavlink_pid_set_t, k_i) }, \
         { "k_d", NULL, MAVLINK_TYPE_FLOAT, 0, 11, offsetof(mavlink_pid_set_t, k_d) }, \
         } \
}


/**
 * @brief Pack a pid_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param pid_id PID ID
 * @param k_p P
 * @param k_i I
 * @param k_d D
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, byte pid_id, Single k_p, Single k_i, Single k_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[15];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, pid_id);
	_mav_put_Single(buf, 3, k_p);
	_mav_put_Single(buf, 7, k_i);
	_mav_put_Single(buf, 11, k_d);

        memcpy(_MAV_PAYLOAD(msg), buf, 15);
#else
	mavlink_pid_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.pid_id = pid_id;
	packet.k_p = k_p;
	packet.k_i = k_i;
	packet.k_d = k_d;

        memcpy(_MAV_PAYLOAD(msg), &packet, 15);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_SET;
	return mavlink_finalize_message(msg, system_id, component_id, 15);
}

/**
 * @brief Pack a pid_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param pid_id PID ID
 * @param k_p P
 * @param k_i I
 * @param k_d D
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,byte pid_id,Single k_p,Single k_i,Single k_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[15];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, pid_id);
	_mav_put_Single(buf, 3, k_p);
	_mav_put_Single(buf, 7, k_i);
	_mav_put_Single(buf, 11, k_d);

        memcpy(_MAV_PAYLOAD(msg), buf, 15);
#else
	mavlink_pid_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.pid_id = pid_id;
	packet.k_p = k_p;
	packet.k_i = k_i;
	packet.k_d = k_d;

        memcpy(_MAV_PAYLOAD(msg), &packet, 15);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_SET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 15);
}

/**
 * @brief Encode a pid_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pid_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_set_t* pid_set)
{
	return mavlink_msg_pid_set_pack(system_id, component_id, msg, pid_set->target_system, pid_set->target_component, pid_set->pid_id, pid_set->k_p, pid_set->k_i, pid_set->k_d);
}

/**
 * @brief Send a pid_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param pid_id PID ID
 * @param k_p P
 * @param k_i I
 * @param k_d D
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_set_send(mavlink_channel_t chan, byte target_system, byte target_component, byte pid_id, Single k_p, Single k_i, Single k_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[15];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, pid_id);
	_mav_put_Single(buf, 3, k_p);
	_mav_put_Single(buf, 7, k_i);
	_mav_put_Single(buf, 11, k_d);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SET, buf, 15);
#else
	mavlink_pid_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.pid_id = pid_id;
	packet.k_p = k_p;
	packet.k_i = k_i;
	packet.k_d = k_d;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SET, (const char *)&packet, 15);
#endif
}

#endif

// MESSAGE PID_SET UNPACKING


/**
 * @brief Get field target_system from pid_set message
 *
 * @return System ID
 */
static inline byte mavlink_msg_pid_set_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from pid_set message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_pid_set_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field pid_id from pid_set message
 *
 * @return PID ID
 */
static inline byte mavlink_msg_pid_set_get_pid_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Get field k_p from pid_set message
 *
 * @return P
 */
static inline Single mavlink_msg_pid_set_get_k_p(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  3);
}

/**
 * @brief Get field k_i from pid_set message
 *
 * @return I
 */
static inline Single mavlink_msg_pid_set_get_k_i(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  7);
}

/**
 * @brief Get field k_d from pid_set message
 *
 * @return D
 */
static inline Single mavlink_msg_pid_set_get_k_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  11);
}

/**
 * @brief Decode a pid_set message into a struct
 *
 * @param msg The message to decode
 * @param pid_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_pid_set_decode(const mavlink_message_t* msg, mavlink_pid_set_t* pid_set)
{
#if MAVLINK_NEED_BYTE_SWAP
	pid_set->target_system = mavlink_msg_pid_set_get_target_system(msg);
	pid_set->target_component = mavlink_msg_pid_set_get_target_component(msg);
	pid_set->pid_id = mavlink_msg_pid_set_get_pid_id(msg);
	pid_set->k_p = mavlink_msg_pid_set_get_k_p(msg);
	pid_set->k_i = mavlink_msg_pid_set_get_k_i(msg);
	pid_set->k_d = mavlink_msg_pid_set_get_k_d(msg);
#else
	memcpy(pid_set, _MAV_PAYLOAD(msg), 15);
#endif
}
