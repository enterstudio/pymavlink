// MESSAGE SYS_STATUS PACKING

#define MAVLINK_MSG_ID_SYS_STATUS 2

typedef struct __mavlink_sys_status_t
{
 byte mode; ///< System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 byte nav_mode; ///< Navigation mode, see MAV_NAV_MODE ENUM
 byte status; ///< System status flag, see MAV_STATUS ENUM
 byte failure; ///< Failure code description, see MAV_FAILURE ENUM
 byte motor_block; ///< Motor block status flag
 UInt16 packet_drop; ///< Dropped packets
} mavlink_sys_status_t;

#define MAVLINK_MSG_ID_SYS_STATUS_LEN 7
#define MAVLINK_MSG_ID_2_LEN 7



#define MAVLINK_MESSAGE_INFO_SYS_STATUS { \
	"SYS_STATUS", \
	6, \
	{  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_sys_status_t, mode) }, \
         { "nav_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_sys_status_t, nav_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_sys_status_t, status) }, \
         { "failure", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_sys_status_t, failure) }, \
         { "motor_block", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_sys_status_t, motor_block) }, \
         { "packet_drop", NULL, MAVLINK_TYPE_UINT16_T, 0, 5, offsetof(mavlink_sys_status_t, packet_drop) }, \
         } \
}


/**
 * @brief Pack a sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param failure Failure code description, see MAV_FAILURE ENUM
 * @param motor_block Motor block status flag
 * @param packet_drop Dropped packets
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte mode, byte nav_mode, byte status, byte failure, byte motor_block, UInt16 packet_drop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, status);
	_mav_put_byte(buf, 3, failure);
	_mav_put_byte(buf, 4, motor_block);
	_mav_put_UInt16(buf, 5, packet_drop);

        memcpy(_MAV_PAYLOAD(msg), buf, 7);
#else
	mavlink_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.failure = failure;
	packet.motor_block = motor_block;
	packet.packet_drop = packet_drop;

        memcpy(_MAV_PAYLOAD(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 7);
}

/**
 * @brief Pack a sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param failure Failure code description, see MAV_FAILURE ENUM
 * @param motor_block Motor block status flag
 * @param packet_drop Dropped packets
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte mode,byte nav_mode,byte status,byte failure,byte motor_block,UInt16 packet_drop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, status);
	_mav_put_byte(buf, 3, failure);
	_mav_put_byte(buf, 4, motor_block);
	_mav_put_UInt16(buf, 5, packet_drop);

        memcpy(_MAV_PAYLOAD(msg), buf, 7);
#else
	mavlink_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.failure = failure;
	packet.motor_block = motor_block;
	packet.packet_drop = packet_drop;

        memcpy(_MAV_PAYLOAD(msg), &packet, 7);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 7);
}

/**
 * @brief Encode a sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status)
{
	return mavlink_msg_sys_status_pack(system_id, component_id, msg, sys_status->mode, sys_status->nav_mode, sys_status->status, sys_status->failure, sys_status->motor_block, sys_status->packet_drop);
}

/**
 * @brief Send a sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param failure Failure code description, see MAV_FAILURE ENUM
 * @param motor_block Motor block status flag
 * @param packet_drop Dropped packets
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_status_send(mavlink_channel_t chan, byte mode, byte nav_mode, byte status, byte failure, byte motor_block, UInt16 packet_drop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[7];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, status);
	_mav_put_byte(buf, 3, failure);
	_mav_put_byte(buf, 4, motor_block);
	_mav_put_UInt16(buf, 5, packet_drop);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, buf, 7);
#else
	mavlink_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.failure = failure;
	packet.motor_block = motor_block;
	packet.packet_drop = packet_drop;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)&packet, 7);
#endif
}

#endif

// MESSAGE SYS_STATUS UNPACKING


/**
 * @brief Get field mode from sys_status message
 *
 * @return System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 */
static inline byte mavlink_msg_sys_status_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field nav_mode from sys_status message
 *
 * @return Navigation mode, see MAV_NAV_MODE ENUM
 */
static inline byte mavlink_msg_sys_status_get_nav_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field status from sys_status message
 *
 * @return System status flag, see MAV_STATUS ENUM
 */
static inline byte mavlink_msg_sys_status_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Get field failure from sys_status message
 *
 * @return Failure code description, see MAV_FAILURE ENUM
 */
static inline byte mavlink_msg_sys_status_get_failure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  3);
}

/**
 * @brief Get field motor_block from sys_status message
 *
 * @return Motor block status flag
 */
static inline byte mavlink_msg_sys_status_get_motor_block(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  4);
}

/**
 * @brief Get field packet_drop from sys_status message
 *
 * @return Dropped packets
 */
static inline UInt16 mavlink_msg_sys_status_get_packet_drop(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  5);
}

/**
 * @brief Decode a sys_status message into a struct
 *
 * @param msg The message to decode
 * @param sys_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	sys_status->mode = mavlink_msg_sys_status_get_mode(msg);
	sys_status->nav_mode = mavlink_msg_sys_status_get_nav_mode(msg);
	sys_status->status = mavlink_msg_sys_status_get_status(msg);
	sys_status->failure = mavlink_msg_sys_status_get_failure(msg);
	sys_status->motor_block = mavlink_msg_sys_status_get_motor_block(msg);
	sys_status->packet_drop = mavlink_msg_sys_status_get_packet_drop(msg);
#else
	memcpy(sys_status, _MAV_PAYLOAD(msg), 7);
#endif
}
