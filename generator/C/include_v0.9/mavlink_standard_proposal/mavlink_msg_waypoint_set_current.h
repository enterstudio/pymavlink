// MESSAGE WAYPOINT_SET_CURRENT PACKING

#define MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT 17

typedef struct __mavlink_waypoint_set_current_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 UInt16 wp_id; ///< Waypoint ID
} mavlink_waypoint_set_current_t;

#define MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT_LEN 4
#define MAVLINK_MSG_ID_17_LEN 4



#define MAVLINK_MESSAGE_INFO_WAYPOINT_SET_CURRENT { \
	"WAYPOINT_SET_CURRENT", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_waypoint_set_current_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_waypoint_set_current_t, target_component) }, \
         { "wp_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_waypoint_set_current_t, wp_id) }, \
         } \
}


/**
 * @brief Pack a waypoint_set_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param wp_id Waypoint ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, UInt16 wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, wp_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_waypoint_set_current_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.wp_id = wp_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT;
	return mavlink_finalize_message(msg, system_id, component_id, 4);
}

/**
 * @brief Pack a waypoint_set_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param wp_id Waypoint ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_set_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,UInt16 wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, wp_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_waypoint_set_current_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.wp_id = wp_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}

/**
 * @brief Encode a waypoint_set_current struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_set_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_set_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_set_current_t* waypoint_set_current)
{
	return mavlink_msg_waypoint_set_current_pack(system_id, component_id, msg, waypoint_set_current->target_system, waypoint_set_current->target_component, waypoint_set_current->wp_id);
}

/**
 * @brief Send a waypoint_set_current message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param wp_id Waypoint ID
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_set_current_send(mavlink_channel_t chan, byte target_system, byte target_component, UInt16 wp_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, wp_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, buf, 4);
#else
	mavlink_waypoint_set_current_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.wp_id = wp_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, (const char *)&packet, 4);
#endif
}

#endif

// MESSAGE WAYPOINT_SET_CURRENT UNPACKING


/**
 * @brief Get field target_system from waypoint_set_current message
 *
 * @return System ID
 */
static inline byte mavlink_msg_waypoint_set_current_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from waypoint_set_current message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_waypoint_set_current_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field wp_id from waypoint_set_current message
 *
 * @return Waypoint ID
 */
static inline UInt16 mavlink_msg_waypoint_set_current_get_wp_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  2);
}

/**
 * @brief Decode a waypoint_set_current message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_set_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_set_current_decode(const mavlink_message_t* msg, mavlink_waypoint_set_current_t* waypoint_set_current)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_set_current->target_system = mavlink_msg_waypoint_set_current_get_target_system(msg);
	waypoint_set_current->target_component = mavlink_msg_waypoint_set_current_get_target_component(msg);
	waypoint_set_current->wp_id = mavlink_msg_waypoint_set_current_get_wp_id(msg);
#else
	memcpy(waypoint_set_current, _MAV_PAYLOAD(msg), 4);
#endif
}
