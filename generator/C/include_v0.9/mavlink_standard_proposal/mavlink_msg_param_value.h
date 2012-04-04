// MESSAGE PARAM_VALUE PACKING

#define MAVLINK_MSG_ID_PARAM_VALUE 32

typedef struct __mavlink_param_value_t
{
 byte[] param_id[15]; ///< Onboard parameter id
 Single param_value; ///< Onboard parameter value
} mavlink_param_value_t;

#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 19
#define MAVLINK_MSG_ID_32_LEN 19

#define MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN 15

#define MAVLINK_MESSAGE_INFO_PARAM_VALUE { \
	"PARAM_VALUE", \
	2, \
	{  { "param_id", NULL, MAVLINK_TYPE_INT8_T, 15, 0, offsetof(mavlink_param_value_t, param_id) }, \
         { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 15, offsetof(mavlink_param_value_t, param_value) }, \
         } \
}


/**
 * @brief Pack a param_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const byte[] *param_id, Single param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[19];
	_mav_put_Single(buf, 15, param_value);
	_mav_put_byte[]_array(buf, 0, param_id, 15);
        memcpy(_MAV_PAYLOAD(msg), buf, 19);
#else
	mavlink_param_value_t packet;
	packet.param_value = param_value;
	mav_array_memcpy(packet.param_id, param_id, sizeof(byte[])*15);
        memcpy(_MAV_PAYLOAD(msg), &packet, 19);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
	return mavlink_finalize_message(msg, system_id, component_id, 19);
}

/**
 * @brief Pack a param_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const byte[] *param_id,Single param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[19];
	_mav_put_Single(buf, 15, param_value);
	_mav_put_byte[]_array(buf, 0, param_id, 15);
        memcpy(_MAV_PAYLOAD(msg), buf, 19);
#else
	mavlink_param_value_t packet;
	packet.param_value = param_value;
	mav_array_memcpy(packet.param_id, param_id, sizeof(byte[])*15);
        memcpy(_MAV_PAYLOAD(msg), &packet, 19);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 19);
}

/**
 * @brief Encode a param_value struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_value C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_value_t* param_value)
{
	return mavlink_msg_param_value_pack(system_id, component_id, msg, param_value->param_id, param_value->param_value);
}

/**
 * @brief Send a param_value message
 * @param chan MAVLink channel to send the message
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_value_send(mavlink_channel_t chan, const byte[] *param_id, Single param_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[19];
	_mav_put_Single(buf, 15, param_value);
	_mav_put_byte[]_array(buf, 0, param_id, 15);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, buf, 19);
#else
	mavlink_param_value_t packet;
	packet.param_value = param_value;
	mav_array_memcpy(packet.param_id, param_id, sizeof(byte[])*15);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, (const char *)&packet, 19);
#endif
}

#endif

// MESSAGE PARAM_VALUE UNPACKING


/**
 * @brief Get field param_id from param_value message
 *
 * @return Onboard parameter id
 */
static inline uint16_t mavlink_msg_param_value_get_param_id(const mavlink_message_t* msg, byte[] *param_id)
{
	return _MAV_RETURN_byte[]_array(msg, param_id, 15,  0);
}

/**
 * @brief Get field param_value from param_value message
 *
 * @return Onboard parameter value
 */
static inline Single mavlink_msg_param_value_get_param_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  15);
}

/**
 * @brief Decode a param_value message into a struct
 *
 * @param msg The message to decode
 * @param param_value C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_param_value_get_param_id(msg, param_value->param_id);
	param_value->param_value = mavlink_msg_param_value_get_param_value(msg);
#else
	memcpy(param_value, _MAV_PAYLOAD(msg), 19);
#endif
}
