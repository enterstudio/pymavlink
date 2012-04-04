// MESSAGE PARAM_REQUEST_READ PACKING

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ 31

typedef struct __mavlink_param_request_read_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 byte[] param_id[15]; ///< Onboard parameter id
} mavlink_param_request_read_t;

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN 17
#define MAVLINK_MSG_ID_31_LEN 17

#define MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN 15

#define MAVLINK_MESSAGE_INFO_PARAM_REQUEST_READ { \
	"PARAM_REQUEST_READ", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_param_request_read_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_param_request_read_t, target_component) }, \
         { "param_id", NULL, MAVLINK_TYPE_INT8_T, 15, 2, offsetof(mavlink_param_request_read_t, param_id) }, \
         } \
}


/**
 * @brief Pack a param_request_read message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_request_read_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, const byte[] *param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[17];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte[]_array(buf, 2, param_id, 15);
        memcpy(_MAV_PAYLOAD(msg), buf, 17);
#else
	mavlink_param_request_read_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.param_id, param_id, sizeof(byte[])*15);
        memcpy(_MAV_PAYLOAD(msg), &packet, 17);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;
	return mavlink_finalize_message(msg, system_id, component_id, 17);
}

/**
 * @brief Pack a param_request_read message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_request_read_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,const byte[] *param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[17];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte[]_array(buf, 2, param_id, 15);
        memcpy(_MAV_PAYLOAD(msg), buf, 17);
#else
	mavlink_param_request_read_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.param_id, param_id, sizeof(byte[])*15);
        memcpy(_MAV_PAYLOAD(msg), &packet, 17);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 17);
}

/**
 * @brief Encode a param_request_read struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_read C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_request_read_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_request_read_t* param_request_read)
{
	return mavlink_msg_param_request_read_pack(system_id, component_id, msg, param_request_read->target_system, param_request_read->target_component, param_request_read->param_id);
}

/**
 * @brief Send a param_request_read message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_request_read_send(mavlink_channel_t chan, byte target_system, byte target_component, const byte[] *param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[17];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte[]_array(buf, 2, param_id, 15);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_READ, buf, 17);
#else
	mavlink_param_request_read_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	mav_array_memcpy(packet.param_id, param_id, sizeof(byte[])*15);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_READ, (const char *)&packet, 17);
#endif
}

#endif

// MESSAGE PARAM_REQUEST_READ UNPACKING


/**
 * @brief Get field target_system from param_request_read message
 *
 * @return System ID
 */
static inline byte mavlink_msg_param_request_read_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from param_request_read message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_param_request_read_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field param_id from param_request_read message
 *
 * @return Onboard parameter id
 */
static inline uint16_t mavlink_msg_param_request_read_get_param_id(const mavlink_message_t* msg, byte[] *param_id)
{
	return _MAV_RETURN_byte[]_array(msg, param_id, 15,  2);
}

/**
 * @brief Decode a param_request_read message into a struct
 *
 * @param msg The message to decode
 * @param param_request_read C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_request_read_decode(const mavlink_message_t* msg, mavlink_param_request_read_t* param_request_read)
{
#if MAVLINK_NEED_BYTE_SWAP
	param_request_read->target_system = mavlink_msg_param_request_read_get_target_system(msg);
	param_request_read->target_component = mavlink_msg_param_request_read_get_target_component(msg);
	mavlink_msg_param_request_read_get_param_id(msg, param_request_read->param_id);
#else
	memcpy(param_request_read, _MAV_PAYLOAD(msg), 17);
#endif
}
