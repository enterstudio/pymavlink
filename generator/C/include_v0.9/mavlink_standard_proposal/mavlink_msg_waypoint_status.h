// MESSAGE WAYPOINT_STATUS PACKING

#define MAVLINK_MSG_ID_WAYPOINT_STATUS 38

typedef struct __mavlink_waypoint_status_t
{
 UInt16 wp_id; ///< Waypoint ID
 byte wp_status; ///< Waypoint status: 0: Ok, 1: Reached, 2: Orbit time expired, 254: Error
} mavlink_waypoint_status_t;

#define MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN 3
#define MAVLINK_MSG_ID_38_LEN 3



#define MAVLINK_MESSAGE_INFO_WAYPOINT_STATUS { \
	"WAYPOINT_STATUS", \
	2, \
	{  { "wp_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_waypoint_status_t, wp_id) }, \
         { "wp_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_waypoint_status_t, wp_status) }, \
         } \
}


/**
 * @brief Pack a waypoint_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param wp_id Waypoint ID
 * @param wp_status Waypoint status: 0: Ok, 1: Reached, 2: Orbit time expired, 254: Error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt16 wp_id, byte wp_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_UInt16(buf, 0, wp_id);
	_mav_put_byte(buf, 2, wp_status);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_waypoint_status_t packet;
	packet.wp_id = wp_id;
	packet.wp_status = wp_status;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 3);
}

/**
 * @brief Pack a waypoint_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param wp_id Waypoint ID
 * @param wp_status Waypoint status: 0: Ok, 1: Reached, 2: Orbit time expired, 254: Error
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt16 wp_id,byte wp_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_UInt16(buf, 0, wp_id);
	_mav_put_byte(buf, 2, wp_status);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_waypoint_status_t packet;
	packet.wp_id = wp_id;
	packet.wp_status = wp_status;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3);
}

/**
 * @brief Encode a waypoint_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_status_t* waypoint_status)
{
	return mavlink_msg_waypoint_status_pack(system_id, component_id, msg, waypoint_status->wp_id, waypoint_status->wp_status);
}

/**
 * @brief Send a waypoint_status message
 * @param chan MAVLink channel to send the message
 *
 * @param wp_id Waypoint ID
 * @param wp_status Waypoint status: 0: Ok, 1: Reached, 2: Orbit time expired, 254: Error
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_status_send(mavlink_channel_t chan, UInt16 wp_id, byte wp_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_UInt16(buf, 0, wp_id);
	_mav_put_byte(buf, 2, wp_status);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_STATUS, buf, 3);
#else
	mavlink_waypoint_status_t packet;
	packet.wp_id = wp_id;
	packet.wp_status = wp_status;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_STATUS, (const char *)&packet, 3);
#endif
}

#endif

// MESSAGE WAYPOINT_STATUS UNPACKING


/**
 * @brief Get field wp_id from waypoint_status message
 *
 * @return Waypoint ID
 */
static inline UInt16 mavlink_msg_waypoint_status_get_wp_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  0);
}

/**
 * @brief Get field wp_status from waypoint_status message
 *
 * @return Waypoint status: 0: Ok, 1: Reached, 2: Orbit time expired, 254: Error
 */
static inline byte mavlink_msg_waypoint_status_get_wp_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Decode a waypoint_status message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_status_decode(const mavlink_message_t* msg, mavlink_waypoint_status_t* waypoint_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_status->wp_id = mavlink_msg_waypoint_status_get_wp_id(msg);
	waypoint_status->wp_status = mavlink_msg_waypoint_status_get_wp_status(msg);
#else
	memcpy(waypoint_status, _MAV_PAYLOAD(msg), 3);
#endif
}
