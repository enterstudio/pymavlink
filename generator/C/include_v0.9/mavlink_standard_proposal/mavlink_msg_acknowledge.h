// MESSAGE ACKNOWLEDGE PACKING

#define MAVLINK_MSG_ID_ACKNOWLEDGE 30

typedef struct __mavlink_acknowledge_t
{
 byte target_system; ///< The system executing the action
 byte target_component; ///< Component ID
 byte akn_id; ///< The id of the action being successfully executed and acknowledged
} mavlink_acknowledge_t;

#define MAVLINK_MSG_ID_ACKNOWLEDGE_LEN 3
#define MAVLINK_MSG_ID_30_LEN 3



#define MAVLINK_MESSAGE_INFO_ACKNOWLEDGE { \
	"ACKNOWLEDGE", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_acknowledge_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_acknowledge_t, target_component) }, \
         { "akn_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_acknowledge_t, akn_id) }, \
         } \
}


/**
 * @brief Pack a acknowledge message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The system executing the action
 * @param target_component Component ID
 * @param akn_id The id of the action being successfully executed and acknowledged
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acknowledge_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, byte akn_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, akn_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_acknowledge_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.akn_id = akn_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACKNOWLEDGE;
	return mavlink_finalize_message(msg, system_id, component_id, 3);
}

/**
 * @brief Pack a acknowledge message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The system executing the action
 * @param target_component Component ID
 * @param akn_id The id of the action being successfully executed and acknowledged
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acknowledge_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,byte akn_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, akn_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_acknowledge_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.akn_id = akn_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACKNOWLEDGE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3);
}

/**
 * @brief Encode a acknowledge struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acknowledge C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acknowledge_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acknowledge_t* acknowledge)
{
	return mavlink_msg_acknowledge_pack(system_id, component_id, msg, acknowledge->target_system, acknowledge->target_component, acknowledge->akn_id);
}

/**
 * @brief Send a acknowledge message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The system executing the action
 * @param target_component Component ID
 * @param akn_id The id of the action being successfully executed and acknowledged
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acknowledge_send(mavlink_channel_t chan, byte target_system, byte target_component, byte akn_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, akn_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACKNOWLEDGE, buf, 3);
#else
	mavlink_acknowledge_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.akn_id = akn_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACKNOWLEDGE, (const char *)&packet, 3);
#endif
}

#endif

// MESSAGE ACKNOWLEDGE UNPACKING


/**
 * @brief Get field target_system from acknowledge message
 *
 * @return The system executing the action
 */
static inline byte mavlink_msg_acknowledge_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from acknowledge message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_acknowledge_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field akn_id from acknowledge message
 *
 * @return The id of the action being successfully executed and acknowledged
 */
static inline byte mavlink_msg_acknowledge_get_akn_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Decode a acknowledge message into a struct
 *
 * @param msg The message to decode
 * @param acknowledge C-struct to decode the message contents into
 */
static inline void mavlink_msg_acknowledge_decode(const mavlink_message_t* msg, mavlink_acknowledge_t* acknowledge)
{
#if MAVLINK_NEED_BYTE_SWAP
	acknowledge->target_system = mavlink_msg_acknowledge_get_target_system(msg);
	acknowledge->target_component = mavlink_msg_acknowledge_get_target_component(msg);
	acknowledge->akn_id = mavlink_msg_acknowledge_get_akn_id(msg);
#else
	memcpy(acknowledge, _MAV_PAYLOAD(msg), 3);
#endif
}
