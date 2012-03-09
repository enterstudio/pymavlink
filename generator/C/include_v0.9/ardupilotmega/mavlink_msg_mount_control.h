// MESSAGE MOUNT_CONTROL PACKING

#define MAVLINK_MSG_ID_MOUNT_CONTROL 157

typedef struct __mavlink_mount_control_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 Int32 input_a; ///< pitch(deg*100) or lat, depending on mount mode
 Int32 input_b; ///< roll(deg*100) or lon depending on mount mode
 Int32 input_c; ///< yaw(deg*100) or alt (in cm) depending on mount mode
 byte save_position; ///< if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
} mavlink_mount_control_t;

#define MAVLINK_MSG_ID_MOUNT_CONTROL_LEN 15
#define MAVLINK_MSG_ID_157_LEN 15



#define MAVLINK_MESSAGE_INFO_MOUNT_CONTROL { \
	"MOUNT_CONTROL", \
	6, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mount_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mount_control_t, target_component) }, \
         { "input_a", NULL, MAVLINK_TYPE_INT32_T, 0, 2, offsetof(mavlink_mount_control_t, input_a) }, \
         { "input_b", NULL, MAVLINK_TYPE_INT32_T, 0, 6, offsetof(mavlink_mount_control_t, input_b) }, \
         { "input_c", NULL, MAVLINK_TYPE_INT32_T, 0, 10, offsetof(mavlink_mount_control_t, input_c) }, \
         { "save_position", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mount_control_t, save_position) }, \
         } \
}


/**
 * @brief Pack a mount_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param input_a pitch(deg*100) or lat, depending on mount mode
 * @param input_b roll(deg*100) or lon depending on mount mode
 * @param input_c yaw(deg*100) or alt (in cm) depending on mount mode
 * @param save_position if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, Int32 input_a, Int32 input_b, Int32 input_c, byte save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[15];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Int32(buf, 2, input_a);
	_mav_put_Int32(buf, 6, input_b);
	_mav_put_Int32(buf, 10, input_c);
	_mav_put_byte(buf, 14, save_position);

        memcpy(_MAV_PAYLOAD(msg), buf, 15);
#else
	mavlink_mount_control_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.input_a = input_a;
	packet.input_b = input_b;
	packet.input_c = input_c;
	packet.save_position = save_position;

        memcpy(_MAV_PAYLOAD(msg), &packet, 15);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOUNT_CONTROL;
	return mavlink_finalize_message(msg, system_id, component_id, 15);
}

/**
 * @brief Pack a mount_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param input_a pitch(deg*100) or lat, depending on mount mode
 * @param input_b roll(deg*100) or lon depending on mount mode
 * @param input_c yaw(deg*100) or alt (in cm) depending on mount mode
 * @param save_position if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mount_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,Int32 input_a,Int32 input_b,Int32 input_c,byte save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[15];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Int32(buf, 2, input_a);
	_mav_put_Int32(buf, 6, input_b);
	_mav_put_Int32(buf, 10, input_c);
	_mav_put_byte(buf, 14, save_position);

        memcpy(_MAV_PAYLOAD(msg), buf, 15);
#else
	mavlink_mount_control_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.input_a = input_a;
	packet.input_b = input_b;
	packet.input_c = input_c;
	packet.save_position = save_position;

        memcpy(_MAV_PAYLOAD(msg), &packet, 15);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOUNT_CONTROL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 15);
}

/**
 * @brief Encode a mount_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mount_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mount_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mount_control_t* mount_control)
{
	return mavlink_msg_mount_control_pack(system_id, component_id, msg, mount_control->target_system, mount_control->target_component, mount_control->input_a, mount_control->input_b, mount_control->input_c, mount_control->save_position);
}

/**
 * @brief Send a mount_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param input_a pitch(deg*100) or lat, depending on mount mode
 * @param input_b roll(deg*100) or lon depending on mount mode
 * @param input_c yaw(deg*100) or alt (in cm) depending on mount mode
 * @param save_position if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mount_control_send(mavlink_channel_t chan, byte target_system, byte target_component, Int32 input_a, Int32 input_b, Int32 input_c, byte save_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[15];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Int32(buf, 2, input_a);
	_mav_put_Int32(buf, 6, input_b);
	_mav_put_Int32(buf, 10, input_c);
	_mav_put_byte(buf, 14, save_position);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, buf, 15);
#else
	mavlink_mount_control_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.input_a = input_a;
	packet.input_b = input_b;
	packet.input_c = input_c;
	packet.save_position = save_position;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOUNT_CONTROL, (const char *)&packet, 15);
#endif
}

#endif

// MESSAGE MOUNT_CONTROL UNPACKING


/**
 * @brief Get field target_system from mount_control message
 *
 * @return System ID
 */
static inline byte mavlink_msg_mount_control_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from mount_control message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_mount_control_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field input_a from mount_control message
 *
 * @return pitch(deg*100) or lat, depending on mount mode
 */
static inline Int32 mavlink_msg_mount_control_get_input_a(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  2);
}

/**
 * @brief Get field input_b from mount_control message
 *
 * @return roll(deg*100) or lon depending on mount mode
 */
static inline Int32 mavlink_msg_mount_control_get_input_b(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  6);
}

/**
 * @brief Get field input_c from mount_control message
 *
 * @return yaw(deg*100) or alt (in cm) depending on mount mode
 */
static inline Int32 mavlink_msg_mount_control_get_input_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  10);
}

/**
 * @brief Get field save_position from mount_control message
 *
 * @return if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
 */
static inline byte mavlink_msg_mount_control_get_save_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  14);
}

/**
 * @brief Decode a mount_control message into a struct
 *
 * @param msg The message to decode
 * @param mount_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_mount_control_decode(const mavlink_message_t* msg, mavlink_mount_control_t* mount_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	mount_control->target_system = mavlink_msg_mount_control_get_target_system(msg);
	mount_control->target_component = mavlink_msg_mount_control_get_target_component(msg);
	mount_control->input_a = mavlink_msg_mount_control_get_input_a(msg);
	mount_control->input_b = mavlink_msg_mount_control_get_input_b(msg);
	mount_control->input_c = mavlink_msg_mount_control_get_input_c(msg);
	mount_control->save_position = mavlink_msg_mount_control_get_save_position(msg);
#else
	memcpy(mount_control, _MAV_PAYLOAD(msg), 15);
#endif
}
