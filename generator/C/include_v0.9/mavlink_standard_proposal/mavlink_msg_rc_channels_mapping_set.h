// MESSAGE RC_CHANNELS_MAPPING_SET PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET 15

typedef struct __mavlink_rc_channels_mapping_set_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 byte chan_id; ///< RC channel id
 byte chan_function; ///< RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
} mavlink_rc_channels_mapping_set_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET_LEN 4
#define MAVLINK_MSG_ID_15_LEN 4



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_MAPPING_SET { \
	"RC_CHANNELS_MAPPING_SET", \
	4, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rc_channels_mapping_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rc_channels_mapping_set_t, target_component) }, \
         { "chan_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rc_channels_mapping_set_t, chan_id) }, \
         { "chan_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_rc_channels_mapping_set_t, chan_function) }, \
         } \
}


/**
 * @brief Pack a rc_channels_mapping_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan_id RC channel id
 * @param chan_function RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_mapping_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, byte chan_id, byte chan_function)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, chan_id);
	_mav_put_byte(buf, 3, chan_function);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_rc_channels_mapping_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.chan_id = chan_id;
	packet.chan_function = chan_function;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET;
	return mavlink_finalize_message(msg, system_id, component_id, 4);
}

/**
 * @brief Pack a rc_channels_mapping_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan_id RC channel id
 * @param chan_function RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_mapping_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,byte chan_id,byte chan_function)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, chan_id);
	_mav_put_byte(buf, 3, chan_function);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
	mavlink_rc_channels_mapping_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.chan_id = chan_id;
	packet.chan_function = chan_function;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}

/**
 * @brief Encode a rc_channels_mapping_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_mapping_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_mapping_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_mapping_set_t* rc_channels_mapping_set)
{
	return mavlink_msg_rc_channels_mapping_set_pack(system_id, component_id, msg, rc_channels_mapping_set->target_system, rc_channels_mapping_set->target_component, rc_channels_mapping_set->chan_id, rc_channels_mapping_set->chan_function);
}

/**
 * @brief Send a rc_channels_mapping_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan_id RC channel id
 * @param chan_function RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_mapping_set_send(mavlink_channel_t chan, byte target_system, byte target_component, byte chan_id, byte chan_function)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, chan_id);
	_mav_put_byte(buf, 3, chan_function);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET, buf, 4);
#else
	mavlink_rc_channels_mapping_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.chan_id = chan_id;
	packet.chan_function = chan_function;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_SET, (const char *)&packet, 4);
#endif
}

#endif

// MESSAGE RC_CHANNELS_MAPPING_SET UNPACKING


/**
 * @brief Get field target_system from rc_channels_mapping_set message
 *
 * @return System ID
 */
static inline byte mavlink_msg_rc_channels_mapping_set_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from rc_channels_mapping_set message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_rc_channels_mapping_set_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field chan_id from rc_channels_mapping_set message
 *
 * @return RC channel id
 */
static inline byte mavlink_msg_rc_channels_mapping_set_get_chan_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Get field chan_function from rc_channels_mapping_set message
 *
 * @return RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 */
static inline byte mavlink_msg_rc_channels_mapping_set_get_chan_function(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  3);
}

/**
 * @brief Decode a rc_channels_mapping_set message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_mapping_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_mapping_set_decode(const mavlink_message_t* msg, mavlink_rc_channels_mapping_set_t* rc_channels_mapping_set)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_mapping_set->target_system = mavlink_msg_rc_channels_mapping_set_get_target_system(msg);
	rc_channels_mapping_set->target_component = mavlink_msg_rc_channels_mapping_set_get_target_component(msg);
	rc_channels_mapping_set->chan_id = mavlink_msg_rc_channels_mapping_set_get_chan_id(msg);
	rc_channels_mapping_set->chan_function = mavlink_msg_rc_channels_mapping_set_get_chan_function(msg);
#else
	memcpy(rc_channels_mapping_set, _MAV_PAYLOAD(msg), 4);
#endif
}
