// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 0

typedef struct __mavlink_heartbeat_t
{
 UInt32 custom_mode; ///< Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 byte type; ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 byte autopilot; ///< Autopilot type / class. defined in MAV_CLASS ENUM
 byte base_mode; ///< System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 byte system_status; ///< System status flag, see MAV_STATUS ENUM
 byte mavlink_version; ///< MAVLink version
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_0_LEN 9



#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
	"HEARTBEAT", \
	6, \
	{  { "custom_mode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_heartbeat_t, custom_mode) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_heartbeat_t, type) }, \
         { "autopilot", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_heartbeat_t, autopilot) }, \
         { "base_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_heartbeat_t, base_mode) }, \
         { "system_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_heartbeat_t, system_status) }, \
         { "mavlink_version", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heartbeat_t, mavlink_version) }, \
         } \
}


/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte type, byte autopilot, byte base_mode, UInt32 custom_mode, byte system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_UInt32(buf, 0, custom_mode);
	_mav_put_byte(buf, 4, type);
	_mav_put_byte(buf, 5, autopilot);
	_mav_put_byte(buf, 6, base_mode);
	_mav_put_byte(buf, 7, system_status);
	_mav_put_byte(buf, 8, 3);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_heartbeat_t packet;
	packet.custom_mode = custom_mode;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message(msg, system_id, component_id, 9, 50);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte type,byte autopilot,byte base_mode,UInt32 custom_mode,byte system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_UInt32(buf, 0, custom_mode);
	_mav_put_byte(buf, 4, type);
	_mav_put_byte(buf, 5, autopilot);
	_mav_put_byte(buf, 6, base_mode);
	_mav_put_byte(buf, 7, system_status);
	_mav_put_byte(buf, 8, 3);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
	mavlink_heartbeat_t packet;
	packet.custom_mode = custom_mode;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9, 50);
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
	return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot, heartbeat->base_mode, heartbeat->custom_mode, heartbeat->system_status);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 * @param system_status System status flag, see MAV_STATUS ENUM
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, byte type, byte autopilot, byte base_mode, UInt32 custom_mode, byte system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[9];
	_mav_put_UInt32(buf, 0, custom_mode);
	_mav_put_byte(buf, 4, type);
	_mav_put_byte(buf, 5, autopilot);
	_mav_put_byte(buf, 6, base_mode);
	_mav_put_byte(buf, 7, system_status);
	_mav_put_byte(buf, 8, 3);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 9, 50);
#else
	mavlink_heartbeat_t packet;
	packet.custom_mode = custom_mode;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 3;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 9, 50);
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
	return _MAV_RETURN_byte(msg,  4);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Autopilot type / class. defined in MAV_CLASS ENUM
 */
static inline byte mavlink_msg_heartbeat_get_autopilot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  5);
}

/**
 * @brief Get field base_mode from heartbeat message
 *
 * @return System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 */
static inline byte mavlink_msg_heartbeat_get_base_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  6);
}

/**
 * @brief Get field custom_mode from heartbeat message
 *
 * @return Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 */
static inline UInt32 mavlink_msg_heartbeat_get_custom_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  0);
}

/**
 * @brief Get field system_status from heartbeat message
 *
 * @return System status flag, see MAV_STATUS ENUM
 */
static inline byte mavlink_msg_heartbeat_get_system_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  7);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
static inline byte mavlink_msg_heartbeat_get_mavlink_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  8);
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
	heartbeat->custom_mode = mavlink_msg_heartbeat_get_custom_mode(msg);
	heartbeat->type = mavlink_msg_heartbeat_get_type(msg);
	heartbeat->autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	heartbeat->base_mode = mavlink_msg_heartbeat_get_base_mode(msg);
	heartbeat->system_status = mavlink_msg_heartbeat_get_system_status(msg);
	heartbeat->mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
#else
	memcpy(heartbeat, _MAV_PAYLOAD(msg), 9);
#endif
}
