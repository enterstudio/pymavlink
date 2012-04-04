// MESSAGE WAYPOINT PACKING

#define MAVLINK_MSG_ID_WAYPOINT 37

typedef struct __mavlink_waypoint_t
{
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
} mavlink_waypoint_t;

#define MAVLINK_MSG_ID_WAYPOINT_LEN 27
#define MAVLINK_MSG_ID_37_LEN 27



#define MAVLINK_MESSAGE_INFO_WAYPOINT { \
	"WAYPOINT", \
	10, \
	{  { "wp_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_waypoint_t, wp_id) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_waypoint_t, type) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 3, offsetof(mavlink_waypoint_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 7, offsetof(mavlink_waypoint_t, param2) }, \
         { "orbit", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_waypoint_t, orbit) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_waypoint_t, current) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_waypoint_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 17, offsetof(mavlink_waypoint_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 21, offsetof(mavlink_waypoint_t, z) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT16_T, 0, 25, offsetof(mavlink_waypoint_t, autocontinue) }, \
         } \
}


/**
 * @brief Pack a waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
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
static inline uint16_t mavlink_msg_waypoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt16 wp_id, byte type, Single param1, Single param2, byte orbit, byte current, Single x, Single y, Single z, UInt16 autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[27];
	_mav_put_UInt16(buf, 0, wp_id);
	_mav_put_byte(buf, 2, type);
	_mav_put_Single(buf, 3, param1);
	_mav_put_Single(buf, 7, param2);
	_mav_put_byte(buf, 11, orbit);
	_mav_put_byte(buf, 12, current);
	_mav_put_Single(buf, 13, x);
	_mav_put_Single(buf, 17, y);
	_mav_put_Single(buf, 21, z);
	_mav_put_UInt16(buf, 25, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 27);
#else
	mavlink_waypoint_t packet;
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

        memcpy(_MAV_PAYLOAD(msg), &packet, 27);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 27);
}

/**
 * @brief Pack a waypoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
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
static inline uint16_t mavlink_msg_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt16 wp_id,byte type,Single param1,Single param2,byte orbit,byte current,Single x,Single y,Single z,UInt16 autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[27];
	_mav_put_UInt16(buf, 0, wp_id);
	_mav_put_byte(buf, 2, type);
	_mav_put_Single(buf, 3, param1);
	_mav_put_Single(buf, 7, param2);
	_mav_put_byte(buf, 11, orbit);
	_mav_put_byte(buf, 12, current);
	_mav_put_Single(buf, 13, x);
	_mav_put_Single(buf, 17, y);
	_mav_put_Single(buf, 21, z);
	_mav_put_UInt16(buf, 25, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 27);
#else
	mavlink_waypoint_t packet;
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

        memcpy(_MAV_PAYLOAD(msg), &packet, 27);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 27);
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
	return mavlink_msg_waypoint_pack(system_id, component_id, msg, waypoint->wp_id, waypoint->type, waypoint->param1, waypoint->param2, waypoint->orbit, waypoint->current, waypoint->x, waypoint->y, waypoint->z, waypoint->autocontinue);
}

/**
 * @brief Send a waypoint message
 * @param chan MAVLink channel to send the message
 *
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

static inline void mavlink_msg_waypoint_send(mavlink_channel_t chan, UInt16 wp_id, byte type, Single param1, Single param2, byte orbit, byte current, Single x, Single y, Single z, UInt16 autocontinue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[27];
	_mav_put_UInt16(buf, 0, wp_id);
	_mav_put_byte(buf, 2, type);
	_mav_put_Single(buf, 3, param1);
	_mav_put_Single(buf, 7, param2);
	_mav_put_byte(buf, 11, orbit);
	_mav_put_byte(buf, 12, current);
	_mav_put_Single(buf, 13, x);
	_mav_put_Single(buf, 17, y);
	_mav_put_Single(buf, 21, z);
	_mav_put_UInt16(buf, 25, autocontinue);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, buf, 27);
#else
	mavlink_waypoint_t packet;
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

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, (const char *)&packet, 27);
#endif
}

#endif

// MESSAGE WAYPOINT UNPACKING


/**
 * @brief Get field wp_id from waypoint message
 *
 * @return Waypoint ID
 */
static inline UInt16 mavlink_msg_waypoint_get_wp_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  0);
}

/**
 * @brief Get field type from waypoint message
 *
 * @return 0: global (GPS), 1: local, 2: global orbit, 3: local orbit
 */
static inline byte mavlink_msg_waypoint_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Get field param1 from waypoint message
 *
 * @return Orbit to circle around the waypoint, in meters
 */
static inline Single mavlink_msg_waypoint_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  3);
}

/**
 * @brief Get field param2 from waypoint message
 *
 * @return Time that the MAV should stay inside the orbit before advancing, in milliseconds
 */
static inline Single mavlink_msg_waypoint_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  7);
}

/**
 * @brief Get field orbit from waypoint message
 *
 * @return 0: No orbit, 1: Right Orbit, 2: Left Orbit
 */
static inline byte mavlink_msg_waypoint_get_orbit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  11);
}

/**
 * @brief Get field current from waypoint message
 *
 * @return false:0, true:1
 */
static inline byte mavlink_msg_waypoint_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  12);
}

/**
 * @brief Get field x from waypoint message
 *
 * @return local: x position, global: longitude
 */
static inline Single mavlink_msg_waypoint_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  13);
}

/**
 * @brief Get field y from waypoint message
 *
 * @return y position: global: latitude
 */
static inline Single mavlink_msg_waypoint_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  17);
}

/**
 * @brief Get field z from waypoint message
 *
 * @return z position: global: altitude
 */
static inline Single mavlink_msg_waypoint_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  21);
}

/**
 * @brief Get field autocontinue from waypoint message
 *
 * @return autocontinue to next wp
 */
static inline UInt16 mavlink_msg_waypoint_get_autocontinue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  25);
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
	waypoint->wp_id = mavlink_msg_waypoint_get_wp_id(msg);
	waypoint->type = mavlink_msg_waypoint_get_type(msg);
	waypoint->param1 = mavlink_msg_waypoint_get_param1(msg);
	waypoint->param2 = mavlink_msg_waypoint_get_param2(msg);
	waypoint->orbit = mavlink_msg_waypoint_get_orbit(msg);
	waypoint->current = mavlink_msg_waypoint_get_current(msg);
	waypoint->x = mavlink_msg_waypoint_get_x(msg);
	waypoint->y = mavlink_msg_waypoint_get_y(msg);
	waypoint->z = mavlink_msg_waypoint_get_z(msg);
	waypoint->autocontinue = mavlink_msg_waypoint_get_autocontinue(msg);
#else
	memcpy(waypoint, _MAV_PAYLOAD(msg), 27);
#endif
}
