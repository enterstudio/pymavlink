// MESSAGE WAYPOINT_SET PACKING

#define MAVLINK_MSG_ID_WAYPOINT_SET 16

typedef struct __mavlink_waypoint_set_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 UInt16 wp_id; ///< Waypoint ID
 byte type; ///< 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 Single param1; ///< Orbit to circle around the waypoint, in meters
 Single param2; ///< Time that the MAV should stay inside the orbit before advancing, in milliseconds
 byte orbit; ///< 0: No orbit, 1: Right Orbit, 2: Left Orbit
 byte current; ///< false:0, true:1
 Single x; ///< local: x position, global: longitude
 Single y; ///< y position: global: latitude
 Single z; ///< z position: global: altitude
 UInt16 autocontinue; ///< autocontinue to next wp
} mavlink_waypoint_set_t;

#define MAVLINK_MSG_ID_WAYPOINT_SET_LEN 29
#define MAVLINK_MSG_ID_16_LEN 29



#define MAVLINK_MESSAGE_INFO_WAYPOINT_SET { \
	"WAYPOINT_SET", \
	12, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_waypoint_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_waypoint_set_t, target_component) }, \
         { "wp_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_waypoint_set_t, wp_id) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_waypoint_set_t, type) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_waypoint_set_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_waypoint_set_t, param2) }, \
         { "orbit", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_waypoint_set_t, orbit) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_waypoint_set_t, current) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 15, offsetof(mavlink_waypoint_set_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 19, offsetof(mavlink_waypoint_set_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 23, offsetof(mavlink_waypoint_set_t, z) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT16_T, 0, 27, offsetof(mavlink_waypoint_set_t, autocontinue) }, \
         } \
}


/**
 * @brief Pack a waypoint_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param wp_id Waypoint ID
 * @param type 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 * @param param1 Orbit to circle around the waypoint, in meters
 * @param param2 Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param orbit 0: No orbit, 1: Right Orbit, 2: Left Orbit
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param autocontinue autocontinue to next wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, UInt16 wp_id, byte type, Single param1, Single param2, byte orbit, byte current, Single x, Single y, Single z, UInt16 autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[29];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, wp_id);
	_mav_put_byte(buf, 4, type);
	_mav_put_Single(buf, 5, param1);
	_mav_put_Single(buf, 9, param2);
	_mav_put_byte(buf, 13, orbit);
	_mav_put_byte(buf, 14, current);
	_mav_put_Single(buf, 15, x);
	_mav_put_Single(buf, 19, y);
	_mav_put_Single(buf, 23, z);
	_mav_put_UInt16(buf, 27, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 29);
#else
	mavlink_waypoint_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.wp_id = wp_id;
	packet.type = type;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.orbit = orbit;
	packet.current = current;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 29);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET;
	return mavlink_finalize_message(msg, system_id, component_id, 29);
}

/**
 * @brief Pack a waypoint_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param wp_id Waypoint ID
 * @param type 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 * @param param1 Orbit to circle around the waypoint, in meters
 * @param param2 Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param orbit 0: No orbit, 1: Right Orbit, 2: Left Orbit
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param autocontinue autocontinue to next wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,UInt16 wp_id,byte type,Single param1,Single param2,byte orbit,byte current,Single x,Single y,Single z,UInt16 autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[29];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, wp_id);
	_mav_put_byte(buf, 4, type);
	_mav_put_Single(buf, 5, param1);
	_mav_put_Single(buf, 9, param2);
	_mav_put_byte(buf, 13, orbit);
	_mav_put_byte(buf, 14, current);
	_mav_put_Single(buf, 15, x);
	_mav_put_Single(buf, 19, y);
	_mav_put_Single(buf, 23, z);
	_mav_put_UInt16(buf, 27, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 29);
#else
	mavlink_waypoint_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.wp_id = wp_id;
	packet.type = type;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.orbit = orbit;
	packet.current = current;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 29);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 29);
}

/**
 * @brief Encode a waypoint_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_set_t* waypoint_set)
{
	return mavlink_msg_waypoint_set_pack(system_id, component_id, msg, waypoint_set->target_system, waypoint_set->target_component, waypoint_set->wp_id, waypoint_set->type, waypoint_set->param1, waypoint_set->param2, waypoint_set->orbit, waypoint_set->current, waypoint_set->x, waypoint_set->y, waypoint_set->z, waypoint_set->autocontinue);
}

/**
 * @brief Send a waypoint_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param wp_id Waypoint ID
 * @param type 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 * @param param1 Orbit to circle around the waypoint, in meters
 * @param param2 Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param orbit 0: No orbit, 1: Right Orbit, 2: Left Orbit
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param autocontinue autocontinue to next wp
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_set_send(mavlink_channel_t chan, byte target_system, byte target_component, UInt16 wp_id, byte type, Single param1, Single param2, byte orbit, byte current, Single x, Single y, Single z, UInt16 autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[29];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, wp_id);
	_mav_put_byte(buf, 4, type);
	_mav_put_Single(buf, 5, param1);
	_mav_put_Single(buf, 9, param2);
	_mav_put_byte(buf, 13, orbit);
	_mav_put_byte(buf, 14, current);
	_mav_put_Single(buf, 15, x);
	_mav_put_Single(buf, 19, y);
	_mav_put_Single(buf, 23, z);
	_mav_put_UInt16(buf, 27, autocontinue);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_SET, buf, 29);
#else
	mavlink_waypoint_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.wp_id = wp_id;
	packet.type = type;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.orbit = orbit;
	packet.current = current;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.autocontinue = autocontinue;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_SET, (const char *)&packet, 29);
#endif
}

#endif

// MESSAGE WAYPOINT_SET UNPACKING


/**
 * @brief Get field target_system from waypoint_set message
 *
 * @return System ID
 */
static inline byte mavlink_msg_waypoint_set_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from waypoint_set message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_waypoint_set_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field wp_id from waypoint_set message
 *
 * @return Waypoint ID
 */
static inline UInt16 mavlink_msg_waypoint_set_get_wp_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  2);
}

/**
 * @brief Get field type from waypoint_set message
 *
 * @return 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 */
static inline byte mavlink_msg_waypoint_set_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  4);
}

/**
 * @brief Get field param1 from waypoint_set message
 *
 * @return Orbit to circle around the waypoint, in meters
 */
static inline Single mavlink_msg_waypoint_set_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  5);
}

/**
 * @brief Get field param2 from waypoint_set message
 *
 * @return Time that the MAV should stay inside the orbit before advancing, in milliseconds
 */
static inline Single mavlink_msg_waypoint_set_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  9);
}

/**
 * @brief Get field orbit from waypoint_set message
 *
 * @return 0: No orbit, 1: Right Orbit, 2: Left Orbit
 */
static inline byte mavlink_msg_waypoint_set_get_orbit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  13);
}

/**
 * @brief Get field current from waypoint_set message
 *
 * @return false:0, true:1
 */
static inline byte mavlink_msg_waypoint_set_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  14);
}

/**
 * @brief Get field x from waypoint_set message
 *
 * @return local: x position, global: longitude
 */
static inline Single mavlink_msg_waypoint_set_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  15);
}

/**
 * @brief Get field y from waypoint_set message
 *
 * @return y position: global: latitude
 */
static inline Single mavlink_msg_waypoint_set_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  19);
}

/**
 * @brief Get field z from waypoint_set message
 *
 * @return z position: global: altitude
 */
static inline Single mavlink_msg_waypoint_set_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  23);
}

/**
 * @brief Get field autocontinue from waypoint_set message
 *
 * @return autocontinue to next wp
 */
static inline UInt16 mavlink_msg_waypoint_set_get_autocontinue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  27);
}

/**
 * @brief Decode a waypoint_set message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_set_decode(const mavlink_message_t* msg, mavlink_waypoint_set_t* waypoint_set)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_set->target_system = mavlink_msg_waypoint_set_get_target_system(msg);
	waypoint_set->target_component = mavlink_msg_waypoint_set_get_target_component(msg);
	waypoint_set->wp_id = mavlink_msg_waypoint_set_get_wp_id(msg);
	waypoint_set->type = mavlink_msg_waypoint_set_get_type(msg);
	waypoint_set->param1 = mavlink_msg_waypoint_set_get_param1(msg);
	waypoint_set->param2 = mavlink_msg_waypoint_set_get_param2(msg);
	waypoint_set->orbit = mavlink_msg_waypoint_set_get_orbit(msg);
	waypoint_set->current = mavlink_msg_waypoint_set_get_current(msg);
	waypoint_set->x = mavlink_msg_waypoint_set_get_x(msg);
	waypoint_set->y = mavlink_msg_waypoint_set_get_y(msg);
	waypoint_set->z = mavlink_msg_waypoint_set_get_z(msg);
	waypoint_set->autocontinue = mavlink_msg_waypoint_set_get_autocontinue(msg);
#else
	memcpy(waypoint_set, _MAV_PAYLOAD(msg), 29);
#endif
}
