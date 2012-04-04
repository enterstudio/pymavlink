// MESSAGE RC_CHANNELS_MAPPING PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_MAPPING 36

typedef struct __mavlink_rc_channels_mapping_t
{
 byte chan_id; ///< RC channel id
 byte chan_function; ///< RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
} mavlink_rc_channels_mapping_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_MAPPING_LEN 2
#define MAVLINK_MSG_ID_36_LEN 2



#define MAVLINK_MESSAGE_INFO_RC_CHANNELS_MAPPING { \
	"RC_CHANNELS_MAPPING", \
	2, \
	{  { "chan_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_rc_channels_mapping_t, chan_id) }, \
         { "chan_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_rc_channels_mapping_t, chan_function) }, \
         } \
}


/**
 * @brief Pack a rc_channels_mapping message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan_id RC channel id
 * @param chan_function RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_mapping_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte chan_id, byte chan_function)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_byte(buf, 0, chan_id);
	_mav_put_byte(buf, 1, chan_function);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
	mavlink_rc_channels_mapping_t packet;
	packet.chan_id = chan_id;
	packet.chan_function = chan_function;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_MAPPING;
	return mavlink_finalize_message(msg, system_id, component_id, 2);
}

/**
 * @brief Pack a rc_channels_mapping message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param chan_id RC channel id
 * @param chan_function RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_mapping_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte chan_id,byte chan_function)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_byte(buf, 0, chan_id);
	_mav_put_byte(buf, 1, chan_function);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
	mavlink_rc_channels_mapping_t packet;
	packet.chan_id = chan_id;
	packet.chan_function = chan_function;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_MAPPING;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2);
}

/**
 * @brief Encode a rc_channels_mapping struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_mapping C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_mapping_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_mapping_t* rc_channels_mapping)
{
	return mavlink_msg_rc_channels_mapping_pack(system_id, component_id, msg, rc_channels_mapping->chan_id, rc_channels_mapping->chan_function);
}

/**
 * @brief Send a rc_channels_mapping message
 * @param chan MAVLink channel to send the message
 *
 * @param chan_id RC channel id
 * @param chan_function RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_mapping_send(mavlink_channel_t chan, byte chan_id, byte chan_function)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_byte(buf, 0, chan_id);
	_mav_put_byte(buf, 1, chan_function);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_MAPPING, buf, 2);
#else
	mavlink_rc_channels_mapping_t packet;
	packet.chan_id = chan_id;
	packet.chan_function = chan_function;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_MAPPING, (const char *)&packet, 2);
#endif
}

#endif

// MESSAGE RC_CHANNELS_MAPPING UNPACKING


/**
 * @brief Get field chan_id from rc_channels_mapping message
 *
 * @return RC channel id
 */
static inline byte mavlink_msg_rc_channels_mapping_get_chan_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field chan_function from rc_channels_mapping message
 *
 * @return RC channel function, as defined in ENUM MAVLINK_RC_CHAN_MAPPING in mavlink/include/mavlink_types.h
 */
static inline byte mavlink_msg_rc_channels_mapping_get_chan_function(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Decode a rc_channels_mapping message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_mapping C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_mapping_decode(const mavlink_message_t* msg, mavlink_rc_channels_mapping_t* rc_channels_mapping)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_channels_mapping->chan_id = mavlink_msg_rc_channels_mapping_get_chan_id(msg);
	rc_channels_mapping->chan_function = mavlink_msg_rc_channels_mapping_get_chan_function(msg);
#else
	memcpy(rc_channels_mapping, _MAV_PAYLOAD(msg), 2);
#endif
}
