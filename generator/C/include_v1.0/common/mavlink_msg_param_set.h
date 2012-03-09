// MESSAGE PARAM_SET PACKING

#define MAVLINK_MSG_ID_PARAM_SET 23

typedef struct __mavlink_param_set_t
{
 Single param_value; ///< Onboard parameter value
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 string param_id[16]; ///< Onboard parameter id
 byte param_type; ///< Onboard parameter type: see MAV_VAR enum
} mavlink_param_set_t;

#define MAVLINK_MSG_ID_PARAM_SET_LEN 23
#define MAVLINK_MSG_ID_23_LEN 23

#define MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN 16

#define MAVLINK_MESSAGE_INFO_PARAM_SET { \
	"PARAM_SET", \
	5, \
	{  { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_param_set_t, param_value) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_param_set_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_param_set_t, target_component) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 6, offsetof(mavlink_param_set_t, param_id) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_param_set_t, param_type) }, \
         } \
}


/**
 * @brief Pack a param_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, const string *param_id, Single param_value, byte param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[23];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_byte(buf, 22, param_type);
	_mav_put_string_array(buf, 6, param_id, 16);
        memcpy(_MAV_PAYLOAD(msg), buf, 23);
#else
	mavlink_param_set_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_type = param_type;
	mav_array_memcpy(packet.param_id, param_id, sizeof(string)*16);
        memcpy(_MAV_PAYLOAD(msg), &packet, 23);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_SET;
	return mavlink_finalize_message(msg, system_id, component_id, 23, 168);
}

/**
 * @brief Pack a param_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,const string *param_id,Single param_value,byte param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[23];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_byte(buf, 22, param_type);
	_mav_put_string_array(buf, 6, param_id, 16);
        memcpy(_MAV_PAYLOAD(msg), buf, 23);
#else
	mavlink_param_set_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_type = param_type;
	mav_array_memcpy(packet.param_id, param_id, sizeof(string)*16);
        memcpy(_MAV_PAYLOAD(msg), &packet, 23);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_SET;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 23, 168);
}

/**
 * @brief Encode a param_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_set_t* param_set)
{
	return mavlink_msg_param_set_pack(system_id, component_id, msg, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}

/**
 * @brief Send a param_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_set_send(mavlink_channel_t chan, byte target_system, byte target_component, const string *param_id, Single param_value, byte param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[23];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_byte(buf, 22, param_type);
	_mav_put_string_array(buf, 6, param_id, 16);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, buf, 23, 168);
#else
	mavlink_param_set_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_type = param_type;
	mav_array_memcpy(packet.param_id, param_id, sizeof(string)*16);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, (const char *)&packet, 23, 168);
#endif
}

#endif

// MESSAGE PARAM_SET UNPACKING


/**
 * @brief Get field target_system from param_set message
 *
 * @return System ID
 */
static inline byte mavlink_msg_param_set_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  4);
}

/**
 * @brief Get field target_component from param_set message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_param_set_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  5);
}

/**
 * @brief Get field param_id from param_set message
 *
 * @return Onboard parameter id
 */
static inline uint16_t mavlink_msg_param_set_get_param_id(const mavlink_message_t* msg, string *param_id)
{
	return _MAV_RETURN_string_array(msg, param_id, 16,  6);
}

/**
 * @brief Get field param_value from param_set message
 *
 * @return Onboard parameter value
 */
static inline Single mavlink_msg_param_set_get_param_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  0);
}

/**
 * @brief Get field param_type from param_set message
 *
 * @return Onboard parameter type: see MAV_VAR enum
 */
static inline byte mavlink_msg_param_set_get_param_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  22);
}

/**
 * @brief Decode a param_set message into a struct
 *
 * @param msg The message to decode
 * @param param_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_set_decode(const mavlink_message_t* msg, mavlink_param_set_t* param_set)
{
#if MAVLINK_NEED_BYTE_SWAP
	param_set->param_value = mavlink_msg_param_set_get_param_value(msg);
	param_set->target_system = mavlink_msg_param_set_get_target_system(msg);
	param_set->target_component = mavlink_msg_param_set_get_target_component(msg);
	mavlink_msg_param_set_get_param_id(msg, param_set->param_id);
	param_set->param_type = mavlink_msg_param_set_get_param_type(msg);
#else
	memcpy(param_set, _MAV_PAYLOAD(msg), 23);
#endif
}
