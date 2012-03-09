// MESSAGE MISSION_ITEM PACKING

#define MAVLINK_MSG_ID_MISSION_ITEM 39

typedef struct __mavlink_mission_item_t
{
 Single param1; ///< PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 Single param2; ///< PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 Single param3; ///< PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 Single param4; ///< PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 Single x; ///< PARAM5 / local: x position, global: latitude
 Single y; ///< PARAM6 / y position: global: longitude
 Single z; ///< PARAM7 / z position: global: altitude
 UInt16 seq; ///< Sequence
 UInt16 command; ///< The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 byte frame; ///< The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 byte current; ///< false:0, true:1
 byte autocontinue; ///< autocontinue to next wp
} mavlink_mission_item_t;

#define MAVLINK_MSG_ID_MISSION_ITEM_LEN 37
#define MAVLINK_MSG_ID_39_LEN 37



#define MAVLINK_MESSAGE_INFO_MISSION_ITEM { \
	"MISSION_ITEM", \
	14, \
	{  { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_mission_item_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_mission_item_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_item_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_item_t, param4) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_mission_item_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_mission_item_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_mission_item_t, z) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_mission_item_t, seq) }, \
         { "command", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_mission_item_t, command) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_mission_item_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_mission_item_t, target_component) }, \
         { "frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_mission_item_t, frame) }, \
         { "current", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_mission_item_t, current) }, \
         { "autocontinue", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_mission_item_t, autocontinue) }, \
         } \
}


/**
 * @brief Pack a mission_item message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, UInt16 seq, byte frame, UInt16 command, byte current, byte autocontinue, Single param1, Single param2, Single param3, Single param4, Single x, Single y, Single z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[37];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, x);
	_mav_put_Single(buf, 20, y);
	_mav_put_Single(buf, 24, z);
	_mav_put_UInt16(buf, 28, seq);
	_mav_put_UInt16(buf, 30, command);
	_mav_put_byte(buf, 32, target_system);
	_mav_put_byte(buf, 33, target_component);
	_mav_put_byte(buf, 34, frame);
	_mav_put_byte(buf, 35, current);
	_mav_put_byte(buf, 36, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 37);
#else
	mavlink_mission_item_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.command = command;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.current = current;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 37);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;
	return mavlink_finalize_message(msg, system_id, component_id, 37, 254);
}

/**
 * @brief Pack a mission_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,UInt16 seq,byte frame,UInt16 command,byte current,byte autocontinue,Single param1,Single param2,Single param3,Single param4,Single x,Single y,Single z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[37];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, x);
	_mav_put_Single(buf, 20, y);
	_mav_put_Single(buf, 24, z);
	_mav_put_UInt16(buf, 28, seq);
	_mav_put_UInt16(buf, 30, command);
	_mav_put_byte(buf, 32, target_system);
	_mav_put_byte(buf, 33, target_component);
	_mav_put_byte(buf, 34, frame);
	_mav_put_byte(buf, 35, current);
	_mav_put_byte(buf, 36, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 37);
#else
	mavlink_mission_item_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.command = command;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.current = current;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 37);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 37, 254);
}

/**
 * @brief Encode a mission_item struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_item C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_item_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_item_t* mission_item)
{
	return mavlink_msg_mission_item_pack(system_id, component_id, msg, mission_item->target_system, mission_item->target_component, mission_item->seq, mission_item->frame, mission_item->command, mission_item->current, mission_item->autocontinue, mission_item->param1, mission_item->param2, mission_item->param3, mission_item->param4, mission_item->x, mission_item->y, mission_item->z);
}

/**
 * @brief Send a mission_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_item_send(mavlink_channel_t chan, byte target_system, byte target_component, UInt16 seq, byte frame, UInt16 command, byte current, byte autocontinue, Single param1, Single param2, Single param3, Single param4, Single x, Single y, Single z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[37];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, x);
	_mav_put_Single(buf, 20, y);
	_mav_put_Single(buf, 24, z);
	_mav_put_UInt16(buf, 28, seq);
	_mav_put_UInt16(buf, 30, command);
	_mav_put_byte(buf, 32, target_system);
	_mav_put_byte(buf, 33, target_component);
	_mav_put_byte(buf, 34, frame);
	_mav_put_byte(buf, 35, current);
	_mav_put_byte(buf, 36, autocontinue);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM, buf, 37, 254);
#else
	mavlink_mission_item_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.command = command;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.current = current;
	packet.autocontinue = autocontinue;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM, (const char *)&packet, 37, 254);
#endif
}

#endif

// MESSAGE MISSION_ITEM UNPACKING


/**
 * @brief Get field target_system from mission_item message
 *
 * @return System ID
 */
static inline byte mavlink_msg_mission_item_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  32);
}

/**
 * @brief Get field target_component from mission_item message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_mission_item_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  33);
}

/**
 * @brief Get field seq from mission_item message
 *
 * @return Sequence
 */
static inline UInt16 mavlink_msg_mission_item_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  28);
}

/**
 * @brief Get field frame from mission_item message
 *
 * @return The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 */
static inline byte mavlink_msg_mission_item_get_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  34);
}

/**
 * @brief Get field command from mission_item message
 *
 * @return The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 */
static inline UInt16 mavlink_msg_mission_item_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  30);
}

/**
 * @brief Get field current from mission_item message
 *
 * @return false:0, true:1
 */
static inline byte mavlink_msg_mission_item_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  35);
}

/**
 * @brief Get field autocontinue from mission_item message
 *
 * @return autocontinue to next wp
 */
static inline byte mavlink_msg_mission_item_get_autocontinue(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  36);
}

/**
 * @brief Get field param1 from mission_item message
 *
 * @return PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 */
static inline Single mavlink_msg_mission_item_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  0);
}

/**
 * @brief Get field param2 from mission_item message
 *
 * @return PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 */
static inline Single mavlink_msg_mission_item_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  4);
}

/**
 * @brief Get field param3 from mission_item message
 *
 * @return PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 */
static inline Single mavlink_msg_mission_item_get_param3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  8);
}

/**
 * @brief Get field param4 from mission_item message
 *
 * @return PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 */
static inline Single mavlink_msg_mission_item_get_param4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  12);
}

/**
 * @brief Get field x from mission_item message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
static inline Single mavlink_msg_mission_item_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  16);
}

/**
 * @brief Get field y from mission_item message
 *
 * @return PARAM6 / y position: global: longitude
 */
static inline Single mavlink_msg_mission_item_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  20);
}

/**
 * @brief Get field z from mission_item message
 *
 * @return PARAM7 / z position: global: altitude
 */
static inline Single mavlink_msg_mission_item_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  24);
}

/**
 * @brief Decode a mission_item message into a struct
 *
 * @param msg The message to decode
 * @param mission_item C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_item_decode(const mavlink_message_t* msg, mavlink_mission_item_t* mission_item)
{
#if MAVLINK_NEED_BYTE_SWAP
	mission_item->param1 = mavlink_msg_mission_item_get_param1(msg);
	mission_item->param2 = mavlink_msg_mission_item_get_param2(msg);
	mission_item->param3 = mavlink_msg_mission_item_get_param3(msg);
	mission_item->param4 = mavlink_msg_mission_item_get_param4(msg);
	mission_item->x = mavlink_msg_mission_item_get_x(msg);
	mission_item->y = mavlink_msg_mission_item_get_y(msg);
	mission_item->z = mavlink_msg_mission_item_get_z(msg);
	mission_item->seq = mavlink_msg_mission_item_get_seq(msg);
	mission_item->command = mavlink_msg_mission_item_get_command(msg);
	mission_item->target_system = mavlink_msg_mission_item_get_target_system(msg);
	mission_item->target_component = mavlink_msg_mission_item_get_target_component(msg);
	mission_item->frame = mavlink_msg_mission_item_get_frame(msg);
	mission_item->current = mavlink_msg_mission_item_get_current(msg);
	mission_item->autocontinue = mavlink_msg_mission_item_get_autocontinue(msg);
#else
	memcpy(mission_item, _MAV_PAYLOAD(msg), 37);
#endif
}
