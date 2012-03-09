// MESSAGE HIL_STATE PACKING

#define MAVLINK_MSG_ID_HIL_STATE 90

typedef struct __mavlink_hil_state_t
{
 UInt64 time_usec; ///< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 Single roll; ///< Roll angle (rad)
 Single pitch; ///< Pitch angle (rad)
 Single yaw; ///< Yaw angle (rad)
 Single rollspeed; ///< Roll angular speed (rad/s)
 Single pitchspeed; ///< Pitch angular speed (rad/s)
 Single yawspeed; ///< Yaw angular speed (rad/s)
 Int32 lat; ///< Latitude, expressed as * 1E7
 Int32 lon; ///< Longitude, expressed as * 1E7
 Int32 alt; ///< Altitude in meters, expressed as * 1000 (millimeters)
 Int16 vx; ///< Ground X Speed (Latitude), expressed as m/s * 100
 Int16 vy; ///< Ground Y Speed (Longitude), expressed as m/s * 100
 Int16 vz; ///< Ground Z Speed (Altitude), expressed as m/s * 100
 Int16 xacc; ///< X acceleration (mg)
 Int16 yacc; ///< Y acceleration (mg)
 Int16 zacc; ///< Z acceleration (mg)
} mavlink_hil_state_t;

#define MAVLINK_MSG_ID_HIL_STATE_LEN 56
#define MAVLINK_MSG_ID_90_LEN 56



#define MAVLINK_MESSAGE_INFO_HIL_STATE { \
	"HIL_STATE", \
	16, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hil_state_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hil_state_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hil_state_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hil_state_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hil_state_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hil_state_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hil_state_t, yawspeed) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_hil_state_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_hil_state_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 40, offsetof(mavlink_hil_state_t, alt) }, \
         { "vx", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_hil_state_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_hil_state_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_hil_state_t, vz) }, \
         { "xacc", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_hil_state_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_hil_state_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_INT16_T, 0, 54, offsetof(mavlink_hil_state_t, zacc) }, \
         } \
}


/**
 * @brief Pack a hil_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt64 time_usec, Single roll, Single pitch, Single yaw, Single rollspeed, Single pitchspeed, Single yawspeed, Int32 lat, Int32 lon, Int32 alt, Int16 vx, Int16 vy, Int16 vz, Int16 xacc, Int16 yacc, Int16 zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[56];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Single(buf, 8, roll);
	_mav_put_Single(buf, 12, pitch);
	_mav_put_Single(buf, 16, yaw);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);
	_mav_put_Int32(buf, 32, lat);
	_mav_put_Int32(buf, 36, lon);
	_mav_put_Int32(buf, 40, alt);
	_mav_put_Int16(buf, 44, vx);
	_mav_put_Int16(buf, 46, vy);
	_mav_put_Int16(buf, 48, vz);
	_mav_put_Int16(buf, 50, xacc);
	_mav_put_Int16(buf, 52, yacc);
	_mav_put_Int16(buf, 54, zacc);

        memcpy(_MAV_PAYLOAD(msg), buf, 56);
#else
	mavlink_hil_state_t packet;
	packet.time_usec = time_usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;

        memcpy(_MAV_PAYLOAD(msg), &packet, 56);
#endif

	msg->msgid = MAVLINK_MSG_ID_HIL_STATE;
	return mavlink_finalize_message(msg, system_id, component_id, 56, 183);
}

/**
 * @brief Pack a hil_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hil_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt64 time_usec,Single roll,Single pitch,Single yaw,Single rollspeed,Single pitchspeed,Single yawspeed,Int32 lat,Int32 lon,Int32 alt,Int16 vx,Int16 vy,Int16 vz,Int16 xacc,Int16 yacc,Int16 zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[56];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Single(buf, 8, roll);
	_mav_put_Single(buf, 12, pitch);
	_mav_put_Single(buf, 16, yaw);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);
	_mav_put_Int32(buf, 32, lat);
	_mav_put_Int32(buf, 36, lon);
	_mav_put_Int32(buf, 40, alt);
	_mav_put_Int16(buf, 44, vx);
	_mav_put_Int16(buf, 46, vy);
	_mav_put_Int16(buf, 48, vz);
	_mav_put_Int16(buf, 50, xacc);
	_mav_put_Int16(buf, 52, yacc);
	_mav_put_Int16(buf, 54, zacc);

        memcpy(_MAV_PAYLOAD(msg), buf, 56);
#else
	mavlink_hil_state_t packet;
	packet.time_usec = time_usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;

        memcpy(_MAV_PAYLOAD(msg), &packet, 56);
#endif

	msg->msgid = MAVLINK_MSG_ID_HIL_STATE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 56, 183);
}

/**
 * @brief Encode a hil_state struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hil_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_state_t* hil_state)
{
	return mavlink_msg_hil_state_pack(system_id, component_id, msg, hil_state->time_usec, hil_state->roll, hil_state->pitch, hil_state->yaw, hil_state->rollspeed, hil_state->pitchspeed, hil_state->yawspeed, hil_state->lat, hil_state->lon, hil_state->alt, hil_state->vx, hil_state->vy, hil_state->vz, hil_state->xacc, hil_state->yacc, hil_state->zacc);
}

/**
 * @brief Send a hil_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_state_send(mavlink_channel_t chan, UInt64 time_usec, Single roll, Single pitch, Single yaw, Single rollspeed, Single pitchspeed, Single yawspeed, Int32 lat, Int32 lon, Int32 alt, Int16 vx, Int16 vy, Int16 vz, Int16 xacc, Int16 yacc, Int16 zacc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[56];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Single(buf, 8, roll);
	_mav_put_Single(buf, 12, pitch);
	_mav_put_Single(buf, 16, yaw);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);
	_mav_put_Int32(buf, 32, lat);
	_mav_put_Int32(buf, 36, lon);
	_mav_put_Int32(buf, 40, alt);
	_mav_put_Int16(buf, 44, vx);
	_mav_put_Int16(buf, 46, vy);
	_mav_put_Int16(buf, 48, vz);
	_mav_put_Int16(buf, 50, xacc);
	_mav_put_Int16(buf, 52, yacc);
	_mav_put_Int16(buf, 54, zacc);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, buf, 56, 183);
#else
	mavlink_hil_state_t packet;
	packet.time_usec = time_usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_STATE, (const char *)&packet, 56, 183);
#endif
}

#endif

// MESSAGE HIL_STATE UNPACKING


/**
 * @brief Get field time_usec from hil_state message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
static inline UInt64 mavlink_msg_hil_state_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt64(msg,  0);
}

/**
 * @brief Get field roll from hil_state message
 *
 * @return Roll angle (rad)
 */
static inline Single mavlink_msg_hil_state_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  8);
}

/**
 * @brief Get field pitch from hil_state message
 *
 * @return Pitch angle (rad)
 */
static inline Single mavlink_msg_hil_state_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  12);
}

/**
 * @brief Get field yaw from hil_state message
 *
 * @return Yaw angle (rad)
 */
static inline Single mavlink_msg_hil_state_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  16);
}

/**
 * @brief Get field rollspeed from hil_state message
 *
 * @return Roll angular speed (rad/s)
 */
static inline Single mavlink_msg_hil_state_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  20);
}

/**
 * @brief Get field pitchspeed from hil_state message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline Single mavlink_msg_hil_state_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  24);
}

/**
 * @brief Get field yawspeed from hil_state message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline Single mavlink_msg_hil_state_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  28);
}

/**
 * @brief Get field lat from hil_state message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline Int32 mavlink_msg_hil_state_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  32);
}

/**
 * @brief Get field lon from hil_state message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline Int32 mavlink_msg_hil_state_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  36);
}

/**
 * @brief Get field alt from hil_state message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters)
 */
static inline Int32 mavlink_msg_hil_state_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int32(msg,  40);
}

/**
 * @brief Get field vx from hil_state message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
static inline Int16 mavlink_msg_hil_state_get_vx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  44);
}

/**
 * @brief Get field vy from hil_state message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
static inline Int16 mavlink_msg_hil_state_get_vy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  46);
}

/**
 * @brief Get field vz from hil_state message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
static inline Int16 mavlink_msg_hil_state_get_vz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  48);
}

/**
 * @brief Get field xacc from hil_state message
 *
 * @return X acceleration (mg)
 */
static inline Int16 mavlink_msg_hil_state_get_xacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  50);
}

/**
 * @brief Get field yacc from hil_state message
 *
 * @return Y acceleration (mg)
 */
static inline Int16 mavlink_msg_hil_state_get_yacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  52);
}

/**
 * @brief Get field zacc from hil_state message
 *
 * @return Z acceleration (mg)
 */
static inline Int16 mavlink_msg_hil_state_get_zacc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Int16(msg,  54);
}

/**
 * @brief Decode a hil_state message into a struct
 *
 * @param msg The message to decode
 * @param hil_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_hil_state_decode(const mavlink_message_t* msg, mavlink_hil_state_t* hil_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	hil_state->time_usec = mavlink_msg_hil_state_get_time_usec(msg);
	hil_state->roll = mavlink_msg_hil_state_get_roll(msg);
	hil_state->pitch = mavlink_msg_hil_state_get_pitch(msg);
	hil_state->yaw = mavlink_msg_hil_state_get_yaw(msg);
	hil_state->rollspeed = mavlink_msg_hil_state_get_rollspeed(msg);
	hil_state->pitchspeed = mavlink_msg_hil_state_get_pitchspeed(msg);
	hil_state->yawspeed = mavlink_msg_hil_state_get_yawspeed(msg);
	hil_state->lat = mavlink_msg_hil_state_get_lat(msg);
	hil_state->lon = mavlink_msg_hil_state_get_lon(msg);
	hil_state->alt = mavlink_msg_hil_state_get_alt(msg);
	hil_state->vx = mavlink_msg_hil_state_get_vx(msg);
	hil_state->vy = mavlink_msg_hil_state_get_vy(msg);
	hil_state->vz = mavlink_msg_hil_state_get_vz(msg);
	hil_state->xacc = mavlink_msg_hil_state_get_xacc(msg);
	hil_state->yacc = mavlink_msg_hil_state_get_yacc(msg);
	hil_state->zacc = mavlink_msg_hil_state_get_zacc(msg);
#else
	memcpy(hil_state, _MAV_PAYLOAD(msg), 56);
#endif
}
