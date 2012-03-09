// MESSAGE SET_MAG_OFFSETS PACKING

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS 151

typedef struct __mavlink_set_mag_offsets_t
{
 byte target_system; ///< System ID
 byte target_component; ///< Component ID
 Int16 mag_ofs_x; ///< magnetometer X offset
 Int16 mag_ofs_y; ///< magnetometer Y offset
 Int16 mag_ofs_z; ///< magnetometer Z offset
} mavlink_set_mag_offsets_t;

#define MAVLINK_MSG_ID_SET_MAG_OFFSETS_LEN 8
#define MAVLINK_MSG_ID_151_LEN 8



#define MAVLINK_MESSAGE_INFO_SET_MAG_OFFSETS { \
	"SET_MAG_OFFSETS", \
	5, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_mag_offsets_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_mag_offsets_t, target_component) }, \
         { "mag_ofs_x", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_set_mag_offsets_t, mag_ofs_x) }, \
         { "mag_ofs_y", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_set_mag_offsets_t, mag_ofs_y) }, \
         { "mag_ofs_z", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_set_mag_offsets_t, mag_ofs_z) }, \
         } \
}


/**
 * @brief Pack a set_mag_offsets message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mag_offsets_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, Int16 mag_ofs_x, Int16 mag_ofs_y, Int16 mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Int16(buf, 2, mag_ofs_x);
	_mav_put_Int16(buf, 4, mag_ofs_y);
	_mav_put_Int16(buf, 6, mag_ofs_z);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_set_mag_offsets_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MAG_OFFSETS;
	return mavlink_finalize_message(msg, system_id, component_id, 8);
}

/**
 * @brief Pack a set_mag_offsets message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_mag_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,Int16 mag_ofs_x,Int16 mag_ofs_y,Int16 mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Int16(buf, 2, mag_ofs_x);
	_mav_put_Int16(buf, 4, mag_ofs_y);
	_mav_put_Int16(buf, 6, mag_ofs_z);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
	mavlink_set_mag_offsets_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_MAG_OFFSETS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8);
}

/**
 * @brief Encode a set_mag_offsets struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_mag_offsets C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_mag_offsets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_mag_offsets_t* set_mag_offsets)
{
	return mavlink_msg_set_mag_offsets_pack(system_id, component_id, msg, set_mag_offsets->target_system, set_mag_offsets->target_component, set_mag_offsets->mag_ofs_x, set_mag_offsets->mag_ofs_y, set_mag_offsets->mag_ofs_z);
}

/**
 * @brief Send a set_mag_offsets message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mag_offsets_send(mavlink_channel_t chan, byte target_system, byte target_component, Int16 mag_ofs_x, Int16 mag_ofs_y, Int16 mag_ofs_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Int16(buf, 2, mag_ofs_x);
	_mav_put_Int16(buf, 4, mag_ofs_y);
	_mav_put_Int16(buf, 6, mag_ofs_z);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, buf, 8);
#else
	mavlink_set_mag_offsets_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MAG_OFFSETS, (const char *)&packet, 8);
#endif
}

#endif

// MESSAGE SET_MAG_OFFSETS UNPACKING


/**
 * @brief Get field target_system from set_mag_offsets message
 *
 * @return System ID
 */
static inline byte mavlink_msg_set_mag_offsets_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from set_mag_offsets message
 *
 * @return Component ID
 */
static inline byte mavlink_msg_set_mag_offsets_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field mag_ofs_x from set_mag_offsets message
 *
 * @return magnetometer X offset
 */
static inline Int16 mavlink_msg_set_mag_offsets_get_mag_ofs_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  2);
}

/**
 * @brief Get field mag_ofs_y from set_mag_offsets message
 *
 * @return magnetometer Y offset
 */
static inline Int16 mavlink_msg_set_mag_offsets_get_mag_ofs_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  4);
}

/**
 * @brief Get field mag_ofs_z from set_mag_offsets message
 *
 * @return magnetometer Z offset
 */
static inline Int16 mavlink_msg_set_mag_offsets_get_mag_ofs_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  6);
}

/**
 * @brief Decode a set_mag_offsets message into a struct
 *
 * @param msg The message to decode
 * @param set_mag_offsets C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_mag_offsets_decode(const mavlink_message_t* msg, mavlink_set_mag_offsets_t* set_mag_offsets)
{
#if MAVLINK_NEED_BYTE_SWAP
	set_mag_offsets->target_system = mavlink_msg_set_mag_offsets_get_target_system(msg);
	set_mag_offsets->target_component = mavlink_msg_set_mag_offsets_get_target_component(msg);
	set_mag_offsets->mag_ofs_x = mavlink_msg_set_mag_offsets_get_mag_ofs_x(msg);
	set_mag_offsets->mag_ofs_y = mavlink_msg_set_mag_offsets_get_mag_ofs_y(msg);
	set_mag_offsets->mag_ofs_z = mavlink_msg_set_mag_offsets_get_mag_ofs_z(msg);
#else
	memcpy(set_mag_offsets, _MAV_PAYLOAD(msg), 8);
#endif
}
