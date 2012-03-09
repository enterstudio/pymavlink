// MESSAGE WATCHDOG_PROCESS_INFO PACKING

#define MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO 181

typedef struct __mavlink_watchdog_process_info_t
{
 Int32 timeout; ///< Timeout (seconds)
 UInt16 watchdog_id; ///< Watchdog ID
 UInt16 process_id; ///< Process ID
 string name[100]; ///< Process name
 string arguments[147]; ///< Process arguments
} mavlink_watchdog_process_info_t;

#define MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO_LEN 255
#define MAVLINK_MSG_ID_181_LEN 255

#define MAVLINK_MSG_WATCHDOG_PROCESS_INFO_FIELD_NAME_LEN 100
#define MAVLINK_MSG_WATCHDOG_PROCESS_INFO_FIELD_ARGUMENTS_LEN 147

#define MAVLINK_MESSAGE_INFO_WATCHDOG_PROCESS_INFO { \
	"WATCHDOG_PROCESS_INFO", \
	5, \
	{  { "timeout", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_watchdog_process_info_t, timeout) }, \
         { "watchdog_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_watchdog_process_info_t, watchdog_id) }, \
         { "process_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_watchdog_process_info_t, process_id) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 100, 8, offsetof(mavlink_watchdog_process_info_t, name) }, \
         { "arguments", NULL, MAVLINK_TYPE_CHAR, 147, 108, offsetof(mavlink_watchdog_process_info_t, arguments) }, \
         } \
}


/**
 * @brief Pack a watchdog_process_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param name Process name
 * @param arguments Process arguments
 * @param timeout Timeout (seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_watchdog_process_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt16 watchdog_id, UInt16 process_id, const string *name, const string *arguments, Int32 timeout)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[255];
	_mav_put_Int32(buf, 0, timeout);
	_mav_put_UInt16(buf, 4, watchdog_id);
	_mav_put_UInt16(buf, 6, process_id);
	_mav_put_string_array(buf, 8, name, 100);
	_mav_put_string_array(buf, 108, arguments, 147);
        memcpy(_MAV_PAYLOAD(msg), buf, 255);
#else
	mavlink_watchdog_process_info_t packet;
	packet.timeout = timeout;
	packet.watchdog_id = watchdog_id;
	packet.process_id = process_id;
	mav_array_memcpy(packet.name, name, sizeof(string)*100);
	mav_array_memcpy(packet.arguments, arguments, sizeof(string)*147);
        memcpy(_MAV_PAYLOAD(msg), &packet, 255);
#endif

	msg->msgid = MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO;
	return mavlink_finalize_message(msg, system_id, component_id, 255, 16);
}

/**
 * @brief Pack a watchdog_process_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param name Process name
 * @param arguments Process arguments
 * @param timeout Timeout (seconds)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_watchdog_process_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt16 watchdog_id,UInt16 process_id,const string *name,const string *arguments,Int32 timeout)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[255];
	_mav_put_Int32(buf, 0, timeout);
	_mav_put_UInt16(buf, 4, watchdog_id);
	_mav_put_UInt16(buf, 6, process_id);
	_mav_put_string_array(buf, 8, name, 100);
	_mav_put_string_array(buf, 108, arguments, 147);
        memcpy(_MAV_PAYLOAD(msg), buf, 255);
#else
	mavlink_watchdog_process_info_t packet;
	packet.timeout = timeout;
	packet.watchdog_id = watchdog_id;
	packet.process_id = process_id;
	mav_array_memcpy(packet.name, name, sizeof(string)*100);
	mav_array_memcpy(packet.arguments, arguments, sizeof(string)*147);
        memcpy(_MAV_PAYLOAD(msg), &packet, 255);
#endif

	msg->msgid = MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 255, 16);
}

/**
 * @brief Encode a watchdog_process_info struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_process_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_watchdog_process_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_watchdog_process_info_t* watchdog_process_info)
{
	return mavlink_msg_watchdog_process_info_pack(system_id, component_id, msg, watchdog_process_info->watchdog_id, watchdog_process_info->process_id, watchdog_process_info->name, watchdog_process_info->arguments, watchdog_process_info->timeout);
}

/**
 * @brief Send a watchdog_process_info message
 * @param chan MAVLink channel to send the message
 *
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param name Process name
 * @param arguments Process arguments
 * @param timeout Timeout (seconds)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_watchdog_process_info_send(mavlink_channel_t chan, UInt16 watchdog_id, UInt16 process_id, const string *name, const string *arguments, Int32 timeout)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[255];
	_mav_put_Int32(buf, 0, timeout);
	_mav_put_UInt16(buf, 4, watchdog_id);
	_mav_put_UInt16(buf, 6, process_id);
	_mav_put_string_array(buf, 8, name, 100);
	_mav_put_string_array(buf, 108, arguments, 147);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO, buf, 255, 16);
#else
	mavlink_watchdog_process_info_t packet;
	packet.timeout = timeout;
	packet.watchdog_id = watchdog_id;
	packet.process_id = process_id;
	mav_array_memcpy(packet.name, name, sizeof(string)*100);
	mav_array_memcpy(packet.arguments, arguments, sizeof(string)*147);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_PROCESS_INFO, (const char *)&packet, 255, 16);
#endif
}

#endif

// MESSAGE WATCHDOG_PROCESS_INFO UNPACKING


/**
 * @brief Get field watchdog_id from watchdog_process_info message
 *
 * @return Watchdog ID
 */
static inline UInt16 mavlink_msg_watchdog_process_info_get_watchdog_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  4);
}

/**
 * @brief Get field process_id from watchdog_process_info message
 *
 * @return Process ID
 */
static inline UInt16 mavlink_msg_watchdog_process_info_get_process_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  6);
}

/**
 * @brief Get field name from watchdog_process_info message
 *
 * @return Process name
 */
static inline uint16_t mavlink_msg_watchdog_process_info_get_name(const mavlink_message_t* msg, string *name)
{
	return _MAV_RETURN_string_array(msg, name, 100,  8);
}

/**
 * @brief Get field arguments from watchdog_process_info message
 *
 * @return Process arguments
 */
static inline uint16_t mavlink_msg_watchdog_process_info_get_arguments(const mavlink_message_t* msg, string *arguments)
{
	return _MAV_RETURN_string_array(msg, arguments, 147,  108);
}

/**
 * @brief Get field timeout from watchdog_process_info message
 *
 * @return Timeout (seconds)
 */
static inline Int32 mavlink_msg_watchdog_process_info_get_timeout(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  0);
}

/**
 * @brief Decode a watchdog_process_info message into a struct
 *
 * @param msg The message to decode
 * @param watchdog_process_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_watchdog_process_info_decode(const mavlink_message_t* msg, mavlink_watchdog_process_info_t* watchdog_process_info)
{
#if MAVLINK_NEED_BYTE_SWAP
	watchdog_process_info->timeout = mavlink_msg_watchdog_process_info_get_timeout(msg);
	watchdog_process_info->watchdog_id = mavlink_msg_watchdog_process_info_get_watchdog_id(msg);
	watchdog_process_info->process_id = mavlink_msg_watchdog_process_info_get_process_id(msg);
	mavlink_msg_watchdog_process_info_get_name(msg, watchdog_process_info->name);
	mavlink_msg_watchdog_process_info_get_arguments(msg, watchdog_process_info->arguments);
#else
	memcpy(watchdog_process_info, _MAV_PAYLOAD(msg), 255);
#endif
}
