// MESSAGE GLOBAL_POSITION_INT PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33

typedef struct __mavlink_global_position_int_t
{
 UInt32 time_boot_ms; ///< Timestamp (milliseconds since system boot)
 Int32 lat; ///< Latitude, expressed as * 1E7
 Int32 lon; ///< Longitude, expressed as * 1E7
 Int32 alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 Int32 relative_alt; ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
 Int16 vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
 Int16 vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
 Int16 vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
 UInt16 hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
} mavlink_global_position_int_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28
#define MAVLINK_MSG_ID_33_LEN 28



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT { \
	"GLOBAL_POSITION_INT", \
	9, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_global_position_int_t, time_boot_ms) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_global_position_int_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_global_position_int_t, relative_alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_global_position_int_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_global_position_int_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_global_position_int_t, vz) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_global_position_int_t, hdg) }, \
         } \
}


/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt32 time_boot_ms, Int32 lat, Int32 lon, Int32 alt, Int32 relative_alt, Int16 vx, Int16 vy, Int16 vz, UInt16 hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, lat);
	_mav_put_Int32(buf, 8, lon);
	_mav_put_Int32(buf, 12, alt);
	_mav_put_Int32(buf, 16, relative_alt);
	_mav_put_Int16(buf, 20, vx);
	_mav_put_Int16(buf, 22, vy);
	_mav_put_Int16(buf, 24, vz);
	_mav_put_UInt16(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	return mavlink_finalize_message(msg, system_id, component_id, 28, 104);
}

/**
 * @brief Pack a global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt32 time_boot_ms,Int32 lat,Int32 lon,Int32 alt,Int32 relative_alt,Int16 vx,Int16 vy,Int16 vz,UInt16 hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, lat);
	_mav_put_Int32(buf, 8, lon);
	_mav_put_Int32(buf, 12, alt);
	_mav_put_Int32(buf, 16, relative_alt);
	_mav_put_Int16(buf, 20, vx);
	_mav_put_Int16(buf, 22, vy);
	_mav_put_Int16(buf, 24, vz);
	_mav_put_UInt16(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 104);
}

/**
 * @brief Encode a global_position_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int)
{
	return mavlink_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->vx, global_position_int->vy, global_position_int->vz, global_position_int->hdg);
}

/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int_send(mavlink_channel_t chan, UInt32 time_boot_ms, Int32 lat, Int32 lon, Int32 alt, Int32 relative_alt, Int16 vx, Int16 vy, Int16 vz, UInt16 hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, lat);
	_mav_put_Int32(buf, 8, lon);
	_mav_put_Int32(buf, 12, alt);
	_mav_put_Int32(buf, 16, relative_alt);
	_mav_put_Int16(buf, 20, vx);
	_mav_put_Int16(buf, 22, vy);
	_mav_put_Int16(buf, 24, vz);
	_mav_put_UInt16(buf, 26, hdg);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, 28, 104);
#else
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, 28, 104);
#endif
}

#endif

// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline UInt32 mavlink_msg_global_position_int_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt32(msg,  0);
}

/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline Int32 mavlink_msg_global_position_int_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  4);
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline Int32 mavlink_msg_global_position_int_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  8);
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
static inline Int32 mavlink_msg_global_position_int_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  12);
}

/**
 * @brief Get field relative_alt from global_position_int message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline Int32 mavlink_msg_global_position_int_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  16);
}

/**
 * @brief Get field vx from global_position_int message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline Int16 mavlink_msg_global_position_int_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  20);
}

/**
 * @brief Get field vy from global_position_int message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline Int16 mavlink_msg_global_position_int_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  22);
}

/**
 * @brief Get field vz from global_position_int message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline Int16 mavlink_msg_global_position_int_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  24);
}

/**
 * @brief Get field hdg from global_position_int message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
static inline UInt16 mavlink_msg_global_position_int_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  26);
}

/**
 * @brief Decode a global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_int_decode(const mavlink_message_t* msg, mavlink_global_position_int_t* global_position_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position_int->time_boot_ms = mavlink_msg_global_position_int_get_time_boot_ms(msg);
	global_position_int->lat = mavlink_msg_global_position_int_get_lat(msg);
	global_position_int->lon = mavlink_msg_global_position_int_get_lon(msg);
	global_position_int->alt = mavlink_msg_global_position_int_get_alt(msg);
	global_position_int->relative_alt = mavlink_msg_global_position_int_get_relative_alt(msg);
	global_position_int->vx = mavlink_msg_global_position_int_get_vx(msg);
	global_position_int->vy = mavlink_msg_global_position_int_get_vy(msg);
	global_position_int->vz = mavlink_msg_global_position_int_get_vz(msg);
	global_position_int->hdg = mavlink_msg_global_position_int_get_hdg(msg);
#else
	memcpy(global_position_int, _MAV_PAYLOAD(msg), 28);
#endif
}
