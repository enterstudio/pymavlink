// MESSAGE PING PACKING

#define MAVLINK_MSG_ID_PING 1

typedef struct __mavlink_ping_t
{
 UInt32 seq; ///< PING sequence
 byte target_system; ///< 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 byte target_component; ///< 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 UInt64 time; ///< Unix timestamp in microseconds
 UInt32 version; ///< The onboard software version
} mavlink_ping_t;

#define MAVLINK_MSG_ID_PING_LEN 18
#define MAVLINK_MSG_ID_1_LEN 18



#define MAVLINK_MESSAGE_INFO_PING { \
	"PING", \
	5, \
	{  { "seq", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_ping_t, seq) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_ping_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_ping_t, target_component) }, \
         { "time", NULL, MAVLINK_TYPE_UINT64_T, 0, 6, offsetof(mavlink_ping_t, time) }, \
         { "version", NULL, MAVLINK_TYPE_UINT32_T, 0, 14, offsetof(mavlink_ping_t, version) }, \
         } \
}


/**
 * @brief Pack a ping message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ping_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt32 seq, byte target_system, byte target_component, UInt64 time, UInt32 version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_UInt32(buf, 0, seq);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_UInt64(buf, 6, time);
	_mav_put_UInt32(buf, 14, version);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_ping_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.time = time;
	packet.version = version;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_PING;
	return mavlink_finalize_message(msg, system_id, component_id, 18);
}

/**
 * @brief Pack a ping message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ping_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt32 seq,byte target_system,byte target_component,UInt64 time,UInt32 version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_UInt32(buf, 0, seq);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_UInt64(buf, 6, time);
	_mav_put_UInt32(buf, 14, version);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
	mavlink_ping_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.time = time;
	packet.version = version;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_PING;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18);
}

/**
 * @brief Encode a ping struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ping C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ping_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ping_t* ping)
{
	return mavlink_msg_ping_pack(system_id, component_id, msg, ping->seq, ping->target_system, ping->target_component, ping->time, ping->version);
}

/**
 * @brief Send a ping message
 * @param chan MAVLink channel to send the message
 *
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 * @param version The onboard software version
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ping_send(mavlink_channel_t chan, UInt32 seq, byte target_system, byte target_component, UInt64 time, UInt32 version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_UInt32(buf, 0, seq);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_UInt64(buf, 6, time);
	_mav_put_UInt32(buf, 14, version);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING, buf, 18);
#else
	mavlink_ping_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.time = time;
	packet.version = version;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING, (const char *)&packet, 18);
#endif
}

#endif

// MESSAGE PING UNPACKING


/**
 * @brief Get field seq from ping message
 *
 * @return PING sequence
 */
static inline UInt32 mavlink_msg_ping_get_seq(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  0);
}

/**
 * @brief Get field target_system from ping message
 *
 * @return 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
static inline byte mavlink_msg_ping_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  4);
}

/**
 * @brief Get field target_component from ping message
 *
 * @return 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
static inline byte mavlink_msg_ping_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  5);
}

/**
 * @brief Get field time from ping message
 *
 * @return Unix timestamp in microseconds
 */
static inline UInt64 mavlink_msg_ping_get_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt64(msg,  6);
}

/**
 * @brief Get field version from ping message
 *
 * @return The onboard software version
 */
static inline UInt32 mavlink_msg_ping_get_version(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  14);
}

/**
 * @brief Decode a ping message into a struct
 *
 * @param msg The message to decode
 * @param ping C-struct to decode the message contents into
 */
static inline void mavlink_msg_ping_decode(const mavlink_message_t* msg, mavlink_ping_t* ping)
{
#if MAVLINK_NEED_BYTE_SWAP
	ping->seq = mavlink_msg_ping_get_seq(msg);
	ping->target_system = mavlink_msg_ping_get_target_system(msg);
	ping->target_component = mavlink_msg_ping_get_target_component(msg);
	ping->time = mavlink_msg_ping_get_time(msg);
	ping->version = mavlink_msg_ping_get_version(msg);
#else
	memcpy(ping, _MAV_PAYLOAD(msg), 18);
#endif
}
