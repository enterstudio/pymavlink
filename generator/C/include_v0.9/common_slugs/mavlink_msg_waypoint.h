// MESSAGE WAYPOINT PACKING

#define MAVLINK_MSG_ID_WAYPOINT 39

typedef struct __mavlink_waypoint_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 UInt16 seq; ///< Sequence
 byte type; ///< 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 Single orbit; ///< Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 byte orbit_direction; ///< Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 Single param1; ///< For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 Single param2; ///< For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 byte current; ///< false:0, true:1
 Single x; ///< local: x position, global: longitude
 Single y; ///< y position: global: latitude
 Single z; ///< z position: global: altitude
 Single yaw; ///< yaw orientation in radians, 0 = NORTH
 byte autocontinue; ///< autocontinue to next wp
} mavlink_waypoint_t;

#define MAVLINK_MSG_ID_WAYPOINT_LEN 36
#define MAVLINK_MSG_ID_39_LEN 36



#define MAVLINK_MESSAGE_INFO_WAYPOINT { \
	"WAYPOINT", \
	14, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_waypoint_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_waypoint_t, target_component) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_waypoint_t, seq) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_waypoint_t, type) }, \
         { "orbit", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_waypoint_t, orbit) }, \
         { "orbit_direction", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_waypoint_t, orbit_direction) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 10, offsetof(mavlink_waypoint_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 14, offsetof(mavlink_waypoint_t, param2) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_waypoint_t, current) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 19, offsetof(mavlink_waypoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 23, offsetof(mavlink_waypoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 27, offsetof(mavlink_waypoint_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 31, offsetof(mavlink_waypoint_t, yaw) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_waypoint_t, autocontinue) }, \
         } \
}


/**
 * @brief Pack a waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param type 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 * @param orbit Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 * @param orbit_direction Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 * @param param1 For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @param autocontinue autocontinue to next wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, UInt16 seq, byte type, Single orbit, byte orbit_direction, Single param1, Single param2, byte current, Single x, Single y, Single z, Single yaw, byte autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, seq);
	_mav_put_byte(buf, 4, type);
	_mav_put_Single(buf, 5, orbit);
	_mav_put_byte(buf, 9, orbit_direction);
	_mav_put_Single(buf, 10, param1);
	_mav_put_Single(buf, 14, param2);
	_mav_put_byte(buf, 18, current);
	_mav_put_Single(buf, 19, x);
	_mav_put_Single(buf, 23, y);
	_mav_put_Single(buf, 27, z);
	_mav_put_Single(buf, 31, yaw);
	_mav_put_byte(buf, 35, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_waypoint_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;
	packet.type = type;
	packet.orbit = orbit;
	packet.orbit_direction = orbit_direction;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.current = current;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 36);
}

/**
 * @brief Pack a waypoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param type 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 * @param orbit Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 * @param orbit_direction Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 * @param param1 For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @param autocontinue autocontinue to next wp
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,UInt16 seq,byte type,Single orbit,byte orbit_direction,Single param1,Single param2,byte current,Single x,Single y,Single z,Single yaw,byte autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, seq);
	_mav_put_byte(buf, 4, type);
	_mav_put_Single(buf, 5, orbit);
	_mav_put_byte(buf, 9, orbit_direction);
	_mav_put_Single(buf, 10, param1);
	_mav_put_Single(buf, 14, param2);
	_mav_put_byte(buf, 18, current);
	_mav_put_Single(buf, 19, x);
	_mav_put_Single(buf, 23, y);
	_mav_put_Single(buf, 27, z);
	_mav_put_Single(buf, 31, yaw);
	_mav_put_byte(buf, 35, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_waypoint_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;
	packet.type = type;
	packet.orbit = orbit;
	packet.orbit_direction = orbit_direction;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.current = current;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36);
}

/**
 * @brief Encode a waypoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_t* waypoint)
{
	return mavlink_msg_waypoint_pack(system_id, component_id, msg, waypoint->target_system, waypoint->target_component, waypoint->seq, waypoint->type, waypoint->orbit, waypoint->orbit_direction, waypoint->param1, waypoint->param2, waypoint->current, waypoint->x, waypoint->y, waypoint->z, waypoint->yaw, waypoint->autocontinue);
}

/**
 * @brief Send a waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param type 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 * @param orbit Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 * @param orbit_direction Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 * @param param1 For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 * @param current false:0, true:1
 * @param x local: x position, global: longitude
 * @param y y position: global: latitude
 * @param z z position: global: altitude
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @param autocontinue autocontinue to next wp
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_send(mavlink_channel_t chan, byte target_system, byte target_component, UInt16 seq, byte type, Single orbit, byte orbit_direction, Single param1, Single param2, byte current, Single x, Single y, Single z, Single yaw, byte autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, seq);
	_mav_put_byte(buf, 4, type);
	_mav_put_Single(buf, 5, orbit);
	_mav_put_byte(buf, 9, orbit_direction);
	_mav_put_Single(buf, 10, param1);
	_mav_put_Single(buf, 14, param2);
	_mav_put_byte(buf, 18, current);
	_mav_put_Single(buf, 19, x);
	_mav_put_Single(buf, 23, y);
	_mav_put_Single(buf, 27, z);
	_mav_put_Single(buf, 31, yaw);
	_mav_put_byte(buf, 35, autocontinue);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, buf, 36);
#else
	mavlink_waypoint_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;
	packet.type = type;
	packet.orbit = orbit;
	packet.orbit_direction = orbit_direction;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.current = current;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.autocontinue = autocontinue;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, (const char *)&packet, 36);
#endif
}

#endif

// MESSAGE WAYPOINT UNPACKING


/**
 * @brief Get field target_system from waypoint message
 *
 * @return System ID
 */
static inline byte mavlink_msg_waypoint_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from waypoint message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_waypoint_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field seq from waypoint message
 *
 * @return Sequence
 */
static inline UInt16 mavlink_msg_waypoint_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  2);
}

/**
 * @brief Get field type from waypoint message
 *
 * @return 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 */
static inline byte mavlink_msg_waypoint_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  4);
}

/**
 * @brief Get field orbit from waypoint message
 *
 * @return Orbit to circle around the waypoint, in meters. Set to 0 to fly straight through the waypoint
 */
static inline Single mavlink_msg_waypoint_get_orbit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  5);
}

/**
 * @brief Get field orbit_direction from waypoint message
 *
 * @return Direction of the orbit circling: 0: clockwise, 1: counter-clockwise
 */
static inline byte mavlink_msg_waypoint_get_orbit_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  9);
}

/**
 * @brief Get field param1 from waypoint message
 *
 * @return For waypoints of type 0 and 1: Radius in which the waypoint is accepted as reached, in meters
 */
static inline Single mavlink_msg_waypoint_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  10);
}

/**
 * @brief Get field param2 from waypoint message
 *
 * @return For waypoints of type 0 and 1: Time that the MAV should stay inside the orbit before advancing, in milliseconds
 */
static inline Single mavlink_msg_waypoint_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  14);
}

/**
 * @brief Get field current from waypoint message
 *
 * @return false:0, true:1
 */
static inline byte mavlink_msg_waypoint_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  18);
}

/**
 * @brief Get field x from waypoint message
 *
 * @return local: x position, global: longitude
 */
static inline Single mavlink_msg_waypoint_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  19);
}

/**
 * @brief Get field y from waypoint message
 *
 * @return y position: global: latitude
 */
static inline Single mavlink_msg_waypoint_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  23);
}

/**
 * @brief Get field z from waypoint message
 *
 * @return z position: global: altitude
 */
static inline Single mavlink_msg_waypoint_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  27);
}

/**
 * @brief Get field yaw from waypoint message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
static inline Single mavlink_msg_waypoint_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  31);
}

/**
 * @brief Get field autocontinue from waypoint message
 *
 * @return autocontinue to next wp
 */
static inline byte mavlink_msg_waypoint_get_autocontinue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  35);
}

/**
 * @brief Decode a waypoint message into a struct
 *
 * @param msg The message to decode
 * @param waypoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_decode(const mavlink_message_t* msg, mavlink_waypoint_t* waypoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint->target_system = mavlink_msg_waypoint_get_target_system(msg);
	waypoint->target_component = mavlink_msg_waypoint_get_target_component(msg);
	waypoint->seq = mavlink_msg_waypoint_get_seq(msg);
	waypoint->type = mavlink_msg_waypoint_get_type(msg);
	waypoint->orbit = mavlink_msg_waypoint_get_orbit(msg);
	waypoint->orbit_direction = mavlink_msg_waypoint_get_orbit_direction(msg);
	waypoint->param1 = mavlink_msg_waypoint_get_param1(msg);
	waypoint->param2 = mavlink_msg_waypoint_get_param2(msg);
	waypoint->current = mavlink_msg_waypoint_get_current(msg);
	waypoint->x = mavlink_msg_waypoint_get_x(msg);
	waypoint->y = mavlink_msg_waypoint_get_y(msg);
	waypoint->z = mavlink_msg_waypoint_get_z(msg);
	waypoint->yaw = mavlink_msg_waypoint_get_yaw(msg);
	waypoint->autocontinue = mavlink_msg_waypoint_get_autocontinue(msg);
#else
	memcpy(waypoint, _MAV_PAYLOAD(msg), 36);
#endif
}
