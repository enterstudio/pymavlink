// MESSAGE WAYPOINT_SET_GLOBAL_REFERENCE PACKING

#define MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE 48

typedef struct __mavlink_waypoint_set_global_reference_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 Single global_x; ///< global x position
 Single global_y; ///< global y position
 Single global_z; ///< global z position
 Single global_yaw; ///< global yaw orientation in radians, 0 = NORTH
 Single local_x; ///< local x position that matches the global x position
 Single local_y; ///< local y position that matches the global y position
 Single local_z; ///< local z position that matches the global z position
 Single local_yaw; ///< local yaw that matches the global yaw orientation
} mavlink_waypoint_set_global_reference_t;

#define MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE_LEN 34
#define MAVLINK_MSG_ID_48_LEN 34



#define MAVLINK_MESSAGE_INFO_WAYPOINT_SET_GLOBAL_REFERENCE { \
	"WAYPOINT_SET_GLOBAL_REFERENCE", \
	10, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_waypoint_set_global_reference_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_waypoint_set_global_reference_t, target_component) }, \
         { "global_x", NULL, MAVLINK_TYPE_FLOAT, 0, 2, offsetof(mavlink_waypoint_set_global_reference_t, global_x) }, \
         { "global_y", NULL, MAVLINK_TYPE_FLOAT, 0, 6, offsetof(mavlink_waypoint_set_global_reference_t, global_y) }, \
         { "global_z", NULL, MAVLINK_TYPE_FLOAT, 0, 10, offsetof(mavlink_waypoint_set_global_reference_t, global_z) }, \
         { "global_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 14, offsetof(mavlink_waypoint_set_global_reference_t, global_yaw) }, \
         { "local_x", NULL, MAVLINK_TYPE_FLOAT, 0, 18, offsetof(mavlink_waypoint_set_global_reference_t, local_x) }, \
         { "local_y", NULL, MAVLINK_TYPE_FLOAT, 0, 22, offsetof(mavlink_waypoint_set_global_reference_t, local_y) }, \
         { "local_z", NULL, MAVLINK_TYPE_FLOAT, 0, 26, offsetof(mavlink_waypoint_set_global_reference_t, local_z) }, \
         { "local_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 30, offsetof(mavlink_waypoint_set_global_reference_t, local_yaw) }, \
         } \
}


/**
 * @brief Pack a waypoint_set_global_reference message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position
 * @param global_y global y position
 * @param global_z global z position
 * @param global_yaw global yaw orientation in radians, 0 = NORTH
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @param local_yaw local yaw that matches the global yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_global_reference_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, Single global_x, Single global_y, Single global_z, Single global_yaw, Single local_x, Single local_y, Single local_z, Single local_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[34];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, global_x);
	_mav_put_Single(buf, 6, global_y);
	_mav_put_Single(buf, 10, global_z);
	_mav_put_Single(buf, 14, global_yaw);
	_mav_put_Single(buf, 18, local_x);
	_mav_put_Single(buf, 22, local_y);
	_mav_put_Single(buf, 26, local_z);
	_mav_put_Single(buf, 30, local_yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 34);
#else
	mavlink_waypoint_set_global_reference_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.global_x = global_x;
	packet.global_y = global_y;
	packet.global_z = global_z;
	packet.global_yaw = global_yaw;
	packet.local_x = local_x;
	packet.local_y = local_y;
	packet.local_z = local_z;
	packet.local_yaw = local_yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 34);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE;
	return mavlink_finalize_message(msg, system_id, component_id, 34);
}

/**
 * @brief Pack a waypoint_set_global_reference message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position
 * @param global_y global y position
 * @param global_z global z position
 * @param global_yaw global yaw orientation in radians, 0 = NORTH
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @param local_yaw local yaw that matches the global yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_global_reference_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,Single global_x,Single global_y,Single global_z,Single global_yaw,Single local_x,Single local_y,Single local_z,Single local_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[34];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, global_x);
	_mav_put_Single(buf, 6, global_y);
	_mav_put_Single(buf, 10, global_z);
	_mav_put_Single(buf, 14, global_yaw);
	_mav_put_Single(buf, 18, local_x);
	_mav_put_Single(buf, 22, local_y);
	_mav_put_Single(buf, 26, local_z);
	_mav_put_Single(buf, 30, local_yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 34);
#else
	mavlink_waypoint_set_global_reference_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.global_x = global_x;
	packet.global_y = global_y;
	packet.global_z = global_z;
	packet.global_yaw = global_yaw;
	packet.local_x = local_x;
	packet.local_y = local_y;
	packet.local_z = local_z;
	packet.local_yaw = local_yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 34);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 34);
}

/**
 * @brief Encode a waypoint_set_global_reference struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_set_global_reference C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_set_global_reference_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_set_global_reference_t* waypoint_set_global_reference)
{
	return mavlink_msg_waypoint_set_global_reference_pack(system_id, component_id, msg, waypoint_set_global_reference->target_system, waypoint_set_global_reference->target_component, waypoint_set_global_reference->global_x, waypoint_set_global_reference->global_y, waypoint_set_global_reference->global_z, waypoint_set_global_reference->global_yaw, waypoint_set_global_reference->local_x, waypoint_set_global_reference->local_y, waypoint_set_global_reference->local_z, waypoint_set_global_reference->local_yaw);
}

/**
 * @brief Send a waypoint_set_global_reference message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param global_x global x position
 * @param global_y global y position
 * @param global_z global z position
 * @param global_yaw global yaw orientation in radians, 0 = NORTH
 * @param local_x local x position that matches the global x position
 * @param local_y local y position that matches the global y position
 * @param local_z local z position that matches the global z position
 * @param local_yaw local yaw that matches the global yaw orientation
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_set_global_reference_send(mavlink_channel_t chan, byte target_system, byte target_component, Single global_x, Single global_y, Single global_z, Single global_yaw, Single local_x, Single local_y, Single local_z, Single local_yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[34];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, global_x);
	_mav_put_Single(buf, 6, global_y);
	_mav_put_Single(buf, 10, global_z);
	_mav_put_Single(buf, 14, global_yaw);
	_mav_put_Single(buf, 18, local_x);
	_mav_put_Single(buf, 22, local_y);
	_mav_put_Single(buf, 26, local_z);
	_mav_put_Single(buf, 30, local_yaw);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE, buf, 34);
#else
	mavlink_waypoint_set_global_reference_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.global_x = global_x;
	packet.global_y = global_y;
	packet.global_z = global_z;
	packet.global_yaw = global_yaw;
	packet.local_x = local_x;
	packet.local_y = local_y;
	packet.local_z = local_z;
	packet.local_yaw = local_yaw;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_SET_GLOBAL_REFERENCE, (const char *)&packet, 34);
#endif
}

#endif

// MESSAGE WAYPOINT_SET_GLOBAL_REFERENCE UNPACKING


/**
 * @brief Get field target_system from waypoint_set_global_reference message
 *
 * @return System ID
 */
static inline byte mavlink_msg_waypoint_set_global_reference_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from waypoint_set_global_reference message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_waypoint_set_global_reference_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field global_x from waypoint_set_global_reference message
 *
 * @return global x position
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_global_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  2);
}

/**
 * @brief Get field global_y from waypoint_set_global_reference message
 *
 * @return global y position
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_global_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  6);
}

/**
 * @brief Get field global_z from waypoint_set_global_reference message
 *
 * @return global z position
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_global_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  10);
}

/**
 * @brief Get field global_yaw from waypoint_set_global_reference message
 *
 * @return global yaw orientation in radians, 0 = NORTH
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_global_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  14);
}

/**
 * @brief Get field local_x from waypoint_set_global_reference message
 *
 * @return local x position that matches the global x position
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_local_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  18);
}

/**
 * @brief Get field local_y from waypoint_set_global_reference message
 *
 * @return local y position that matches the global y position
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_local_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  22);
}

/**
 * @brief Get field local_z from waypoint_set_global_reference message
 *
 * @return local z position that matches the global z position
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_local_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  26);
}

/**
 * @brief Get field local_yaw from waypoint_set_global_reference message
 *
 * @return local yaw that matches the global yaw orientation
 */
static inline Single mavlink_msg_waypoint_set_global_reference_get_local_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  30);
}

/**
 * @brief Decode a waypoint_set_global_reference message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_set_global_reference C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_set_global_reference_decode(const mavlink_message_t* msg, mavlink_waypoint_set_global_reference_t* waypoint_set_global_reference)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_set_global_reference->target_system = mavlink_msg_waypoint_set_global_reference_get_target_system(msg);
	waypoint_set_global_reference->target_component = mavlink_msg_waypoint_set_global_reference_get_target_component(msg);
	waypoint_set_global_reference->global_x = mavlink_msg_waypoint_set_global_reference_get_global_x(msg);
	waypoint_set_global_reference->global_y = mavlink_msg_waypoint_set_global_reference_get_global_y(msg);
	waypoint_set_global_reference->global_z = mavlink_msg_waypoint_set_global_reference_get_global_z(msg);
	waypoint_set_global_reference->global_yaw = mavlink_msg_waypoint_set_global_reference_get_global_yaw(msg);
	waypoint_set_global_reference->local_x = mavlink_msg_waypoint_set_global_reference_get_local_x(msg);
	waypoint_set_global_reference->local_y = mavlink_msg_waypoint_set_global_reference_get_local_y(msg);
	waypoint_set_global_reference->local_z = mavlink_msg_waypoint_set_global_reference_get_local_z(msg);
	waypoint_set_global_reference->local_yaw = mavlink_msg_waypoint_set_global_reference_get_local_yaw(msg);
#else
	memcpy(waypoint_set_global_reference, _MAV_PAYLOAD(msg), 34);
#endif
}
