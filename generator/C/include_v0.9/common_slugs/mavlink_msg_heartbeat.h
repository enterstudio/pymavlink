// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 0

typedef struct __mavlink_heartbeat_t
{
 byte type; ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 byte autopilot; ///< Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 2
#define MAVLINK_MSG_ID_0_LEN 2



#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
	"HEARTBEAT", \
	2, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_heartbeat_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_heartbeat_t, autopilot) }, \
         } \
}


/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte type, byte autopilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message(msg, system_id, component_id, 2);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte type,byte autopilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2);
}

/**
 * @brief Encode a heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
	return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, byte type, byte autopilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 2);
#else
	mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 2);
#endif
}

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
static inline byte mavlink_msg_heartbeat_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 */
static inline byte mavlink_msg_heartbeat_get_autopilot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP
	heartbeat->type = mavlink_msg_heartbeat_get_type(msg);
	heartbeat->autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
#else
	memcpy(heartbeat, _MAV_PAYLOAD(msg), 2);
#endif
}
