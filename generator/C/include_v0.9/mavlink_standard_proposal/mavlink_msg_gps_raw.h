// MESSAGE GPS_RAW PACKING

#define MAVLINK_MSG_ID_GPS_RAW 52

typedef struct __mavlink_gps_raw_t
{
 byte fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix
 Single lat; ///< X Position
 Single lon; ///< Y Position
 Single alt; ///< Z Position in meters
 Single eph; ///< Uncertainty in meters of latitude
 Single epv; ///< Uncertainty in meters of longitude
 Single v; ///< Overall speed
 Single hdg; ///< Heading, in FIXME
} mavlink_gps_raw_t;

#define MAVLINK_MSG_ID_GPS_RAW_LEN 29
#define MAVLINK_MSG_ID_52_LEN 29



#define MAVLINK_MESSAGE_INFO_GPS_RAW { \
	"GPS_RAW", \
	8, \
	{  { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_raw_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 1, offsetof(mavlink_gps_raw_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_gps_raw_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_gps_raw_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_gps_raw_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 17, offsetof(mavlink_gps_raw_t, epv) }, \
         { "v", NULL, MAVLINK_TYPE_FLOAT, 0, 21, offsetof(mavlink_gps_raw_t, v) }, \
         { "hdg", NULL, MAVLINK_TYPE_FLOAT, 0, 25, offsetof(mavlink_gps_raw_t, hdg) }, \
         } \
}


/**
 * @brief Pack a gps_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix
 * @param lat X Position
 * @param lon Y Position
 * @param alt Z Position in meters
 * @param eph Uncertainty in meters of latitude
 * @param epv Uncertainty in meters of longitude
 * @param v Overall speed
 * @param hdg Heading, in FIXME
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte fix_type, Single lat, Single lon, Single alt, Single eph, Single epv, Single v, Single hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[29];
	_mav_put_byte(buf, 0, fix_type);
	_mav_put_Single(buf, 1, lat);
	_mav_put_Single(buf, 5, lon);
	_mav_put_Single(buf, 9, alt);
	_mav_put_Single(buf, 13, eph);
	_mav_put_Single(buf, 17, epv);
	_mav_put_Single(buf, 21, v);
	_mav_put_Single(buf, 25, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 29);
#else
	mavlink_gps_raw_t packet;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 29);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 29);
}

/**
 * @brief Pack a gps_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix
 * @param lat X Position
 * @param lon Y Position
 * @param alt Z Position in meters
 * @param eph Uncertainty in meters of latitude
 * @param epv Uncertainty in meters of longitude
 * @param v Overall speed
 * @param hdg Heading, in FIXME
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte fix_type,Single lat,Single lon,Single alt,Single eph,Single epv,Single v,Single hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[29];
	_mav_put_byte(buf, 0, fix_type);
	_mav_put_Single(buf, 1, lat);
	_mav_put_Single(buf, 5, lon);
	_mav_put_Single(buf, 9, alt);
	_mav_put_Single(buf, 13, eph);
	_mav_put_Single(buf, 17, epv);
	_mav_put_Single(buf, 21, v);
	_mav_put_Single(buf, 25, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 29);
#else
	mavlink_gps_raw_t packet;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 29);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 29);
}

/**
 * @brief Encode a gps_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_raw_t* gps_raw)
{
	return mavlink_msg_gps_raw_pack(system_id, component_id, msg, gps_raw->fix_type, gps_raw->lat, gps_raw->lon, gps_raw->alt, gps_raw->eph, gps_raw->epv, gps_raw->v, gps_raw->hdg);
}

/**
 * @brief Send a gps_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix
 * @param lat X Position
 * @param lon Y Position
 * @param alt Z Position in meters
 * @param eph Uncertainty in meters of latitude
 * @param epv Uncertainty in meters of longitude
 * @param v Overall speed
 * @param hdg Heading, in FIXME
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_send(mavlink_channel_t chan, byte fix_type, Single lat, Single lon, Single alt, Single eph, Single epv, Single v, Single hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[29];
	_mav_put_byte(buf, 0, fix_type);
	_mav_put_Single(buf, 1, lat);
	_mav_put_Single(buf, 5, lon);
	_mav_put_Single(buf, 9, alt);
	_mav_put_Single(buf, 13, eph);
	_mav_put_Single(buf, 17, epv);
	_mav_put_Single(buf, 21, v);
	_mav_put_Single(buf, 25, hdg);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW, buf, 29);
#else
	mavlink_gps_raw_t packet;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW, (const char *)&packet, 29);
#endif
}

#endif

// MESSAGE GPS_RAW UNPACKING


/**
 * @brief Get field fix_type from gps_raw message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix
 */
static inline byte mavlink_msg_gps_raw_get_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field lat from gps_raw message
 *
 * @return X Position
 */
static inline Single mavlink_msg_gps_raw_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  1);
}

/**
 * @brief Get field lon from gps_raw message
 *
 * @return Y Position
 */
static inline Single mavlink_msg_gps_raw_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  5);
}

/**
 * @brief Get field alt from gps_raw message
 *
 * @return Z Position in meters
 */
static inline Single mavlink_msg_gps_raw_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  9);
}

/**
 * @brief Get field eph from gps_raw message
 *
 * @return Uncertainty in meters of latitude
 */
static inline Single mavlink_msg_gps_raw_get_eph(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  13);
}

/**
 * @brief Get field epv from gps_raw message
 *
 * @return Uncertainty in meters of longitude
 */
static inline Single mavlink_msg_gps_raw_get_epv(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  17);
}

/**
 * @brief Get field v from gps_raw message
 *
 * @return Overall speed
 */
static inline Single mavlink_msg_gps_raw_get_v(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  21);
}

/**
 * @brief Get field hdg from gps_raw message
 *
 * @return Heading, in FIXME
 */
static inline Single mavlink_msg_gps_raw_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  25);
}

/**
 * @brief Decode a gps_raw message into a struct
 *
 * @param msg The message to decode
 * @param gps_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_raw_decode(const mavlink_message_t* msg, mavlink_gps_raw_t* gps_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps_raw->fix_type = mavlink_msg_gps_raw_get_fix_type(msg);
	gps_raw->lat = mavlink_msg_gps_raw_get_lat(msg);
	gps_raw->lon = mavlink_msg_gps_raw_get_lon(msg);
	gps_raw->alt = mavlink_msg_gps_raw_get_alt(msg);
	gps_raw->eph = mavlink_msg_gps_raw_get_eph(msg);
	gps_raw->epv = mavlink_msg_gps_raw_get_epv(msg);
	gps_raw->v = mavlink_msg_gps_raw_get_v(msg);
	gps_raw->hdg = mavlink_msg_gps_raw_get_hdg(msg);
#else
	memcpy(gps_raw, _MAV_PAYLOAD(msg), 29);
#endif
}
