// MESSAGE FILTERED_DATA PACKING

#define MAVLINK_MSG_ID_FILTERED_DATA 178

typedef struct __mavlink_filtered_data_t
{
 Single aX; ///< Accelerometer X value (m/s^2) 
 Single aY; ///< Accelerometer Y value (m/s^2)
 Single aZ; ///< Accelerometer Z value (m/s^2)
 Single gX; ///< Gyro X value (rad/s) 
 Single gY; ///< Gyro Y value (rad/s)
 Single gZ; ///< Gyro Z value (rad/s)
 Single mX; ///< Magnetometer X (normalized to 1) 
 Single mY; ///< Magnetometer Y (normalized to 1) 
 Single mZ; ///< Magnetometer Z (normalized to 1) 
} mavlink_filtered_data_t;

#define MAVLINK_MSG_ID_FILTERED_DATA_LEN 36
#define MAVLINK_MSG_ID_178_LEN 36



#define MAVLINK_MESSAGE_INFO_FILTERED_DATA { \
	"FILTERED_DATA", \
	9, \
	{  { "aX", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_filtered_data_t, aX) }, \
         { "aY", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_filtered_data_t, aY) }, \
         { "aZ", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_filtered_data_t, aZ) }, \
         { "gX", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_filtered_data_t, gX) }, \
         { "gY", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_filtered_data_t, gY) }, \
         { "gZ", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_filtered_data_t, gZ) }, \
         { "mX", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_filtered_data_t, mX) }, \
         { "mY", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_filtered_data_t, mY) }, \
         { "mZ", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_filtered_data_t, mZ) }, \
         } \
}


/**
 * @brief Pack a filtered_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param aX Accelerometer X value (m/s^2) 
 * @param aY Accelerometer Y value (m/s^2)
 * @param aZ Accelerometer Z value (m/s^2)
 * @param gX Gyro X value (rad/s) 
 * @param gY Gyro Y value (rad/s)
 * @param gZ Gyro Z value (rad/s)
 * @param mX Magnetometer X (normalized to 1) 
 * @param mY Magnetometer Y (normalized to 1) 
 * @param mZ Magnetometer Z (normalized to 1) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_filtered_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       Single aX, Single aY, Single aZ, Single gX, Single gY, Single gZ, Single mX, Single mY, Single mZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_Single(buf, 0, aX);
	_mav_put_Single(buf, 4, aY);
	_mav_put_Single(buf, 8, aZ);
	_mav_put_Single(buf, 12, gX);
	_mav_put_Single(buf, 16, gY);
	_mav_put_Single(buf, 20, gZ);
	_mav_put_Single(buf, 24, mX);
	_mav_put_Single(buf, 28, mY);
	_mav_put_Single(buf, 32, mZ);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_filtered_data_t packet;
	packet.aX = aX;
	packet.aY = aY;
	packet.aZ = aZ;
	packet.gX = gX;
	packet.gY = gY;
	packet.gZ = gZ;
	packet.mX = mX;
	packet.mY = mY;
	packet.mZ = mZ;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILTERED_DATA;
	return mavlink_finalize_message(msg, system_id, component_id, 36);
}

/**
 * @brief Pack a filtered_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param aX Accelerometer X value (m/s^2) 
 * @param aY Accelerometer Y value (m/s^2)
 * @param aZ Accelerometer Z value (m/s^2)
 * @param gX Gyro X value (rad/s) 
 * @param gY Gyro Y value (rad/s)
 * @param gZ Gyro Z value (rad/s)
 * @param mX Magnetometer X (normalized to 1) 
 * @param mY Magnetometer Y (normalized to 1) 
 * @param mZ Magnetometer Z (normalized to 1) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_filtered_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           Single aX,Single aY,Single aZ,Single gX,Single gY,Single gZ,Single mX,Single mY,Single mZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_Single(buf, 0, aX);
	_mav_put_Single(buf, 4, aY);
	_mav_put_Single(buf, 8, aZ);
	_mav_put_Single(buf, 12, gX);
	_mav_put_Single(buf, 16, gY);
	_mav_put_Single(buf, 20, gZ);
	_mav_put_Single(buf, 24, mX);
	_mav_put_Single(buf, 28, mY);
	_mav_put_Single(buf, 32, mZ);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
	mavlink_filtered_data_t packet;
	packet.aX = aX;
	packet.aY = aY;
	packet.aZ = aZ;
	packet.gX = gX;
	packet.gY = gY;
	packet.gZ = gZ;
	packet.mX = mX;
	packet.mY = mY;
	packet.mZ = mZ;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILTERED_DATA;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36);
}

/**
 * @brief Encode a filtered_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param filtered_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_filtered_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_filtered_data_t* filtered_data)
{
	return mavlink_msg_filtered_data_pack(system_id, component_id, msg, filtered_data->aX, filtered_data->aY, filtered_data->aZ, filtered_data->gX, filtered_data->gY, filtered_data->gZ, filtered_data->mX, filtered_data->mY, filtered_data->mZ);
}

/**
 * @brief Send a filtered_data message
 * @param chan MAVLink channel to send the message
 *
 * @param aX Accelerometer X value (m/s^2) 
 * @param aY Accelerometer Y value (m/s^2)
 * @param aZ Accelerometer Z value (m/s^2)
 * @param gX Gyro X value (rad/s) 
 * @param gY Gyro Y value (rad/s)
 * @param gZ Gyro Z value (rad/s)
 * @param mX Magnetometer X (normalized to 1) 
 * @param mY Magnetometer Y (normalized to 1) 
 * @param mZ Magnetometer Z (normalized to 1) 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_filtered_data_send(mavlink_channel_t chan, Single aX, Single aY, Single aZ, Single gX, Single gY, Single gZ, Single mX, Single mY, Single mZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_Single(buf, 0, aX);
	_mav_put_Single(buf, 4, aY);
	_mav_put_Single(buf, 8, aZ);
	_mav_put_Single(buf, 12, gX);
	_mav_put_Single(buf, 16, gY);
	_mav_put_Single(buf, 20, gZ);
	_mav_put_Single(buf, 24, mX);
	_mav_put_Single(buf, 28, mY);
	_mav_put_Single(buf, 32, mZ);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILTERED_DATA, buf, 36);
#else
	mavlink_filtered_data_t packet;
	packet.aX = aX;
	packet.aY = aY;
	packet.aZ = aZ;
	packet.gX = gX;
	packet.gY = gY;
	packet.gZ = gZ;
	packet.mX = mX;
	packet.mY = mY;
	packet.mZ = mZ;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILTERED_DATA, (const char *)&packet, 36);
#endif
}

#endif

// MESSAGE FILTERED_DATA UNPACKING


/**
 * @brief Get field aX from filtered_data message
 *
 * @return Accelerometer X value (m/s^2) 
 */
static inline Single mavlink_msg_filtered_data_get_aX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  0);
}

/**
 * @brief Get field aY from filtered_data message
 *
 * @return Accelerometer Y value (m/s^2)
 */
static inline Single mavlink_msg_filtered_data_get_aY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  4);
}

/**
 * @brief Get field aZ from filtered_data message
 *
 * @return Accelerometer Z value (m/s^2)
 */
static inline Single mavlink_msg_filtered_data_get_aZ(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  8);
}

/**
 * @brief Get field gX from filtered_data message
 *
 * @return Gyro X value (rad/s) 
 */
static inline Single mavlink_msg_filtered_data_get_gX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  12);
}

/**
 * @brief Get field gY from filtered_data message
 *
 * @return Gyro Y value (rad/s)
 */
static inline Single mavlink_msg_filtered_data_get_gY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  16);
}

/**
 * @brief Get field gZ from filtered_data message
 *
 * @return Gyro Z value (rad/s)
 */
static inline Single mavlink_msg_filtered_data_get_gZ(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  20);
}

/**
 * @brief Get field mX from filtered_data message
 *
 * @return Magnetometer X (normalized to 1) 
 */
static inline Single mavlink_msg_filtered_data_get_mX(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  24);
}

/**
 * @brief Get field mY from filtered_data message
 *
 * @return Magnetometer Y (normalized to 1) 
 */
static inline Single mavlink_msg_filtered_data_get_mY(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  28);
}

/**
 * @brief Get field mZ from filtered_data message
 *
 * @return Magnetometer Z (normalized to 1) 
 */
static inline Single mavlink_msg_filtered_data_get_mZ(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  32);
}

/**
 * @brief Decode a filtered_data message into a struct
 *
 * @param msg The message to decode
 * @param filtered_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_filtered_data_decode(const mavlink_message_t* msg, mavlink_filtered_data_t* filtered_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	filtered_data->aX = mavlink_msg_filtered_data_get_aX(msg);
	filtered_data->aY = mavlink_msg_filtered_data_get_aY(msg);
	filtered_data->aZ = mavlink_msg_filtered_data_get_aZ(msg);
	filtered_data->gX = mavlink_msg_filtered_data_get_gX(msg);
	filtered_data->gY = mavlink_msg_filtered_data_get_gY(msg);
	filtered_data->gZ = mavlink_msg_filtered_data_get_gZ(msg);
	filtered_data->mX = mavlink_msg_filtered_data_get_mX(msg);
	filtered_data->mY = mavlink_msg_filtered_data_get_mY(msg);
	filtered_data->mZ = mavlink_msg_filtered_data_get_mZ(msg);
#else
	memcpy(filtered_data, _MAV_PAYLOAD(msg), 36);
#endif
}
