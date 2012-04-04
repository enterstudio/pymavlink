// MESSAGE POSITION PACKING

#define MAVLINK_MSG_ID_POSITION 55

typedef struct __mavlink_position_t
{
 byte type; ///< Position type: 0: Local, 1: Global
 Single x; ///< X (long) Position
 Single y; ///< Y (lat) Position
 Single z; ///< Z (alt) Position
 Single vx; ///< Vx
 Single vy; ///< Vy
 Single vz; ///< Vz
} mavlink_position_t;

#define MAVLINK_MSG_ID_POSITION_LEN 25
#define MAVLINK_MSG_ID_55_LEN 25



#define MAVLINK_MESSAGE_INFO_POSITION { \
	"POSITION", \
	7, \
	{  { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_position_t, type) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 1, offsetof(mavlink_position_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_position_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_position_t, z) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_position_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 17, offsetof(mavlink_position_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 21, offsetof(mavlink_position_t, vz) }, \
         } \
}


/**
 * @brief Pack a position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Position type: 0: Local, 1: Global
 * @param x X (long) Position
 * @param y Y (lat) Position
 * @param z Z (alt) Position
 * @param vx Vx
 * @param vy Vy
 * @param vz Vz
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte type, Single x, Single y, Single z, Single vx, Single vy, Single vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[25];
	_mav_put_byte(buf, 0, type);
	_mav_put_Single(buf, 1, x);
	_mav_put_Single(buf, 5, y);
	_mav_put_Single(buf, 9, z);
	_mav_put_Single(buf, 13, vx);
	_mav_put_Single(buf, 17, vy);
	_mav_put_Single(buf, 21, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 25);
#else
	mavlink_position_t packet;
	packet.type = type;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 25);
#endif

	msg->msgid = MAVLINK_MSG_ID_POSITION;
	return mavlink_finalize_message(msg, system_id, component_id, 25);
}

/**
 * @brief Pack a position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Position type: 0: Local, 1: Global
 * @param x X (long) Position
 * @param y Y (lat) Position
 * @param z Z (alt) Position
 * @param vx Vx
 * @param vy Vy
 * @param vz Vz
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte type,Single x,Single y,Single z,Single vx,Single vy,Single vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[25];
	_mav_put_byte(buf, 0, type);
	_mav_put_Single(buf, 1, x);
	_mav_put_Single(buf, 5, y);
	_mav_put_Single(buf, 9, z);
	_mav_put_Single(buf, 13, vx);
	_mav_put_Single(buf, 17, vy);
	_mav_put_Single(buf, 21, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 25);
#else
	mavlink_position_t packet;
	packet.type = type;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 25);
#endif

	msg->msgid = MAVLINK_MSG_ID_POSITION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 25);
}

/**
 * @brief Encode a position struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_t* position)
{
	return mavlink_msg_position_pack(system_id, component_id, msg, position->type, position->x, position->y, position->z, position->vx, position->vy, position->vz);
}

/**
 * @brief Send a position message
 * @param chan MAVLink channel to send the message
 *
 * @param type Position type: 0: Local, 1: Global
 * @param x X (long) Position
 * @param y Y (lat) Position
 * @param z Z (alt) Position
 * @param vx Vx
 * @param vy Vy
 * @param vz Vz
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_send(mavlink_channel_t chan, byte type, Single x, Single y, Single z, Single vx, Single vy, Single vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[25];
	_mav_put_byte(buf, 0, type);
	_mav_put_Single(buf, 1, x);
	_mav_put_Single(buf, 5, y);
	_mav_put_Single(buf, 9, z);
	_mav_put_Single(buf, 13, vx);
	_mav_put_Single(buf, 17, vy);
	_mav_put_Single(buf, 21, vz);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION, buf, 25);
#else
	mavlink_position_t packet;
	packet.type = type;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION, (const char *)&packet, 25);
#endif
}

#endif

// MESSAGE POSITION UNPACKING


/**
 * @brief Get field type from position message
 *
 * @return Position type: 0: Local, 1: Global
 */
static inline byte mavlink_msg_position_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field x from position message
 *
 * @return X (long) Position
 */
static inline Single mavlink_msg_position_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  1);
}

/**
 * @brief Get field y from position message
 *
 * @return Y (lat) Position
 */
static inline Single mavlink_msg_position_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  5);
}

/**
 * @brief Get field z from position message
 *
 * @return Z (alt) Position
 */
static inline Single mavlink_msg_position_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  9);
}

/**
 * @brief Get field vx from position message
 *
 * @return Vx
 */
static inline Single mavlink_msg_position_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  13);
}

/**
 * @brief Get field vy from position message
 *
 * @return Vy
 */
static inline Single mavlink_msg_position_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  17);
}

/**
 * @brief Get field vz from position message
 *
 * @return Vz
 */
static inline Single mavlink_msg_position_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  21);
}

/**
 * @brief Decode a position message into a struct
 *
 * @param msg The message to decode
 * @param position C-struct to decode the message contents into
 */
static inline void mavlink_msg_position_decode(const mavlink_message_t* msg, mavlink_position_t* position)
{
#if MAVLINK_NEED_BYTE_SWAP
	position->type = mavlink_msg_position_get_type(msg);
	position->x = mavlink_msg_position_get_x(msg);
	position->y = mavlink_msg_position_get_y(msg);
	position->z = mavlink_msg_position_get_z(msg);
	position->vx = mavlink_msg_position_get_vx(msg);
	position->vy = mavlink_msg_position_get_vy(msg);
	position->vz = mavlink_msg_position_get_vz(msg);
#else
	memcpy(position, _MAV_PAYLOAD(msg), 25);
#endif
}
