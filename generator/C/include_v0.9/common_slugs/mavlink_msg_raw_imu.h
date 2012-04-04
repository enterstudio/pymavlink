// MESSAGE RAW_IMU PACKING

#define MAVLINK_MSG_ID_RAW_IMU 28

typedef struct __mavlink_raw_imu_t
{
 UInt64 usec; ///< Timestamp (microseconds since UNIX epoch)
 Int16 xacc; ///< X acceleration (mg raw)
 Int16 yacc; ///< Y acceleration (mg raw)
 Int16 zacc; ///< Z acceleration (mg raw)
 Int16 xgyro; ///< Angular speed around X axis (adc units)
 Int16 ygyro; ///< Angular speed around Y axis (adc units)
 Int16 zgyro; ///< Angular speed around Z axis (adc units)
 Int16 xmag; ///< X Magnetic field (milli tesla)
 Int16 ymag; ///< Y Magnetic field (milli tesla)
 Int16 zmag; ///< Z Magnetic field (milli tesla)
} mavlink_raw_imu_t;

#define MAVLINK_MSG_ID_RAW_IMU_LEN 26
#define MAVLINK_MSG_ID_28_LEN 26



#define MAVLINK_MESSAGE_INFO_RAW_IMU { \
	"RAW_IMU", \
	10, \
	{  { "usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_raw_imu_t, usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_raw_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_raw_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_raw_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_raw_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_raw_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_raw_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_raw_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_raw_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_raw_imu_t, zmag) }, \
         } \
}


/**
 * @brief Pack a raw_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc X acceleration (mg raw)
 * @param yacc Y acceleration (mg raw)
 * @param zacc Z acceleration (mg raw)
 * @param xgyro Angular speed around X axis (adc units)
 * @param ygyro Angular speed around Y axis (adc units)
 * @param zgyro Angular speed around Z axis (adc units)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt64 usec, Int16 xacc, Int16 yacc, Int16 zacc, Int16 xgyro, Int16 ygyro, Int16 zgyro, Int16 xmag, Int16 ymag, Int16 zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[26];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Int16(buf, 8, xacc);
	_mav_put_Int16(buf, 10, yacc);
	_mav_put_Int16(buf, 12, zacc);
	_mav_put_Int16(buf, 14, xgyro);
	_mav_put_Int16(buf, 16, ygyro);
	_mav_put_Int16(buf, 18, zgyro);
	_mav_put_Int16(buf, 20, xmag);
	_mav_put_Int16(buf, 22, ymag);
	_mav_put_Int16(buf, 24, zmag);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
	mavlink_raw_imu_t packet;
	packet.usec = usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_IMU;
	return mavlink_finalize_message(msg, system_id, component_id, 26);
}

/**
 * @brief Pack a raw_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc X acceleration (mg raw)
 * @param yacc Y acceleration (mg raw)
 * @param zacc Z acceleration (mg raw)
 * @param xgyro Angular speed around X axis (adc units)
 * @param ygyro Angular speed around Y axis (adc units)
 * @param zgyro Angular speed around Z axis (adc units)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_raw_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt64 usec,Int16 xacc,Int16 yacc,Int16 zacc,Int16 xgyro,Int16 ygyro,Int16 zgyro,Int16 xmag,Int16 ymag,Int16 zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[26];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Int16(buf, 8, xacc);
	_mav_put_Int16(buf, 10, yacc);
	_mav_put_Int16(buf, 12, zacc);
	_mav_put_Int16(buf, 14, xgyro);
	_mav_put_Int16(buf, 16, ygyro);
	_mav_put_Int16(buf, 18, zgyro);
	_mav_put_Int16(buf, 20, xmag);
	_mav_put_Int16(buf, 22, ymag);
	_mav_put_Int16(buf, 24, zmag);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
	mavlink_raw_imu_t packet;
	packet.usec = usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

	msg->msgid = MAVLINK_MSG_ID_RAW_IMU;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26);
}

/**
 * @brief Encode a raw_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_raw_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_imu_t* raw_imu)
{
	return mavlink_msg_raw_imu_pack(system_id, component_id, msg, raw_imu->usec, raw_imu->xacc, raw_imu->yacc, raw_imu->zacc, raw_imu->xgyro, raw_imu->ygyro, raw_imu->zgyro, raw_imu->xmag, raw_imu->ymag, raw_imu->zmag);
}

/**
 * @brief Send a raw_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch)
 * @param xacc X acceleration (mg raw)
 * @param yacc Y acceleration (mg raw)
 * @param zacc Z acceleration (mg raw)
 * @param xgyro Angular speed around X axis (adc units)
 * @param ygyro Angular speed around Y axis (adc units)
 * @param zgyro Angular speed around Z axis (adc units)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_imu_send(mavlink_channel_t chan, UInt64 usec, Int16 xacc, Int16 yacc, Int16 zacc, Int16 xgyro, Int16 ygyro, Int16 zgyro, Int16 xmag, Int16 ymag, Int16 zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[26];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Int16(buf, 8, xacc);
	_mav_put_Int16(buf, 10, yacc);
	_mav_put_Int16(buf, 12, zacc);
	_mav_put_Int16(buf, 14, xgyro);
	_mav_put_Int16(buf, 16, ygyro);
	_mav_put_Int16(buf, 18, zgyro);
	_mav_put_Int16(buf, 20, xmag);
	_mav_put_Int16(buf, 22, ymag);
	_mav_put_Int16(buf, 24, zmag);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU, buf, 26);
#else
	mavlink_raw_imu_t packet;
	packet.usec = usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU, (const char *)&packet, 26);
#endif
}

#endif

// MESSAGE RAW_IMU UNPACKING


/**
 * @brief Get field usec from raw_imu message
 *
 * @return Timestamp (microseconds since UNIX epoch)
 */
static inline UInt64 mavlink_msg_raw_imu_get_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt64(msg,  0);
}

/**
 * @brief Get field xacc from raw_imu message
 *
 * @return X acceleration (mg raw)
 */
static inline Int16 mavlink_msg_raw_imu_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  8);
}

/**
 * @brief Get field yacc from raw_imu message
 *
 * @return Y acceleration (mg raw)
 */
static inline Int16 mavlink_msg_raw_imu_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  10);
}

/**
 * @brief Get field zacc from raw_imu message
 *
 * @return Z acceleration (mg raw)
 */
static inline Int16 mavlink_msg_raw_imu_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  12);
}

/**
 * @brief Get field xgyro from raw_imu message
 *
 * @return Angular speed around X axis (adc units)
 */
static inline Int16 mavlink_msg_raw_imu_get_xgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  14);
}

/**
 * @brief Get field ygyro from raw_imu message
 *
 * @return Angular speed around Y axis (adc units)
 */
static inline Int16 mavlink_msg_raw_imu_get_ygyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  16);
}

/**
 * @brief Get field zgyro from raw_imu message
 *
 * @return Angular speed around Z axis (adc units)
 */
static inline Int16 mavlink_msg_raw_imu_get_zgyro(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  18);
}

/**
 * @brief Get field xmag from raw_imu message
 *
 * @return X Magnetic field (milli tesla)
 */
static inline Int16 mavlink_msg_raw_imu_get_xmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  20);
}

/**
 * @brief Get field ymag from raw_imu message
 *
 * @return Y Magnetic field (milli tesla)
 */
static inline Int16 mavlink_msg_raw_imu_get_ymag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  22);
}

/**
 * @brief Get field zmag from raw_imu message
 *
 * @return Z Magnetic field (milli tesla)
 */
static inline Int16 mavlink_msg_raw_imu_get_zmag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  24);
}

/**
 * @brief Decode a raw_imu message into a struct
 *
 * @param msg The message to decode
 * @param raw_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
{
#if MAVLINK_NEED_BYTE_SWAP
	raw_imu->usec = mavlink_msg_raw_imu_get_usec(msg);
	raw_imu->xacc = mavlink_msg_raw_imu_get_xacc(msg);
	raw_imu->yacc = mavlink_msg_raw_imu_get_yacc(msg);
	raw_imu->zacc = mavlink_msg_raw_imu_get_zacc(msg);
	raw_imu->xgyro = mavlink_msg_raw_imu_get_xgyro(msg);
	raw_imu->ygyro = mavlink_msg_raw_imu_get_ygyro(msg);
	raw_imu->zgyro = mavlink_msg_raw_imu_get_zgyro(msg);
	raw_imu->xmag = mavlink_msg_raw_imu_get_xmag(msg);
	raw_imu->ymag = mavlink_msg_raw_imu_get_ymag(msg);
	raw_imu->zmag = mavlink_msg_raw_imu_get_zmag(msg);
#else
	memcpy(raw_imu, _MAV_PAYLOAD(msg), 26);
#endif
}
