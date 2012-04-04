// MESSAGE PARAM_REQUEST_LIST PACKING

#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST 33

typedef struct __mavlink_param_request_list_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 byte param_subset_id; ///< 0: All parameters, else report a subset of parameters as defined in MAVLIN_SUBSET_PARAM enum
} mavlink_param_request_list_t;

#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN 3
#define MAVLINK_MSG_ID_33_LEN 3



#define MAVLINK_MESSAGE_INFO_PARAM_REQUEST_LIST { \
	"PARAM_REQUEST_LIST", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_param_request_list_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_param_request_list_t, target_component) }, \
         { "param_subset_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_param_request_list_t, param_subset_id) }, \
         } \
}


/**
 * @brief Pack a param_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_subset_id 0: All parameters, else report a subset of parameters as defined in MAVLIN_SUBSET_PARAM enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_request_list_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, byte param_subset_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, param_subset_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_param_request_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_subset_id = param_subset_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_LIST;
	return mavlink_finalize_message(msg, system_id, component_id, 3);
}

/**
 * @brief Pack a param_request_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_subset_id 0: All parameters, else report a subset of parameters as defined in MAVLIN_SUBSET_PARAM enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_request_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,byte param_subset_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, param_subset_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
	mavlink_param_request_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_subset_id = param_subset_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_LIST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3);
}

/**
 * @brief Encode a param_request_list struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_list C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_request_list_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_request_list_t* param_request_list)
{
	return mavlink_msg_param_request_list_pack(system_id, component_id, msg, param_request_list->target_system, param_request_list->target_component, param_request_list->param_subset_id);
}

/**
 * @brief Send a param_request_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_subset_id 0: All parameters, else report a subset of parameters as defined in MAVLIN_SUBSET_PARAM enum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_request_list_send(mavlink_channel_t chan, byte target_system, byte target_component, byte param_subset_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, param_subset_id);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_LIST, buf, 3);
#else
	mavlink_param_request_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_subset_id = param_subset_id;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_LIST, (const char *)&packet, 3);
#endif
}

#endif

// MESSAGE PARAM_REQUEST_LIST UNPACKING


/**
 * @brief Get field target_system from param_request_list message
 *
 * @return System ID
 */
static inline byte mavlink_msg_param_request_list_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from param_request_list message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_param_request_list_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field param_subset_id from param_request_list message
 *
 * @return 0: All parameters, else report a subset of parameters as defined in MAVLIN_SUBSET_PARAM enum
 */
static inline byte mavlink_msg_param_request_list_get_param_subset_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Decode a param_request_list message into a struct
 *
 * @param msg The message to decode
 * @param param_request_list C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_request_list_decode(const mavlink_message_t* msg, mavlink_param_request_list_t* param_request_list)
{
#if MAVLINK_NEED_BYTE_SWAP
	param_request_list->target_system = mavlink_msg_param_request_list_get_target_system(msg);
	param_request_list->target_component = mavlink_msg_param_request_list_get_target_component(msg);
	param_request_list->param_subset_id = mavlink_msg_param_request_list_get_param_subset_id(msg);
#else
	memcpy(param_request_list, _MAV_PAYLOAD(msg), 3);
#endif
}
