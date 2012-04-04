// MESSAGE PILOT_CONSOLE PACKING

#define MAVLINK_MSG_ID_PILOT_CONSOLE 174

typedef struct __mavlink_pilot_console_t
{
 UInt16 dt; ///< Pilot's console throttle command 
 UInt16 dla; ///< Pilot's console left aileron command 
 UInt16 dra; ///< Pilot's console right aileron command 
 UInt16 dr; ///< Pilot's console rudder command 
 UInt16 de; ///< Pilot's console elevator command 
} mavlink_pilot_console_t;

#define MAVLINK_MSG_ID_PILOT_CONSOLE_LEN 10
#define MAVLINK_MSG_ID_174_LEN 10



#define MAVLINK_MESSAGE_INFO_PILOT_CONSOLE { \
	"PILOT_CONSOLE", \
	5, \
	{  { "dt", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_pilot_console_t, dt) }, \
         { "dla", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_pilot_console_t, dla) }, \
         { "dra", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_pilot_console_t, dra) }, \
         { "dr", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_pilot_console_t, dr) }, \
         { "de", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_pilot_console_t, de) }, \
         } \
}


/**
 * @brief Pack a pilot_console message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dt Pilot's console throttle command 
 * @param dla Pilot's console left aileron command 
 * @param dra Pilot's console right aileron command 
 * @param dr Pilot's console rudder command 
 * @param de Pilot's console elevator command 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pilot_console_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt16 dt, UInt16 dla, UInt16 dra, UInt16 dr, UInt16 de)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_UInt16(buf, 0, dt);
	_mav_put_UInt16(buf, 2, dla);
	_mav_put_UInt16(buf, 4, dra);
	_mav_put_UInt16(buf, 6, dr);
	_mav_put_UInt16(buf, 8, de);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_pilot_console_t packet;
	packet.dt = dt;
	packet.dla = dla;
	packet.dra = dra;
	packet.dr = dr;
	packet.de = de;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_PILOT_CONSOLE;
	return mavlink_finalize_message(msg, system_id, component_id, 10);
}

/**
 * @brief Pack a pilot_console message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param dt Pilot's console throttle command 
 * @param dla Pilot's console left aileron command 
 * @param dra Pilot's console right aileron command 
 * @param dr Pilot's console rudder command 
 * @param de Pilot's console elevator command 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pilot_console_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt16 dt,UInt16 dla,UInt16 dra,UInt16 dr,UInt16 de)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_UInt16(buf, 0, dt);
	_mav_put_UInt16(buf, 2, dla);
	_mav_put_UInt16(buf, 4, dra);
	_mav_put_UInt16(buf, 6, dr);
	_mav_put_UInt16(buf, 8, de);

        memcpy(_MAV_PAYLOAD(msg), buf, 10);
#else
	mavlink_pilot_console_t packet;
	packet.dt = dt;
	packet.dla = dla;
	packet.dra = dra;
	packet.dr = dr;
	packet.de = de;

        memcpy(_MAV_PAYLOAD(msg), &packet, 10);
#endif

	msg->msgid = MAVLINK_MSG_ID_PILOT_CONSOLE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 10);
}

/**
 * @brief Encode a pilot_console struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pilot_console C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pilot_console_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pilot_console_t* pilot_console)
{
	return mavlink_msg_pilot_console_pack(system_id, component_id, msg, pilot_console->dt, pilot_console->dla, pilot_console->dra, pilot_console->dr, pilot_console->de);
}

/**
 * @brief Send a pilot_console message
 * @param chan MAVLink channel to send the message
 *
 * @param dt Pilot's console throttle command 
 * @param dla Pilot's console left aileron command 
 * @param dra Pilot's console right aileron command 
 * @param dr Pilot's console rudder command 
 * @param de Pilot's console elevator command 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pilot_console_send(mavlink_channel_t chan, UInt16 dt, UInt16 dla, UInt16 dra, UInt16 dr, UInt16 de)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[10];
	_mav_put_UInt16(buf, 0, dt);
	_mav_put_UInt16(buf, 2, dla);
	_mav_put_UInt16(buf, 4, dra);
	_mav_put_UInt16(buf, 6, dr);
	_mav_put_UInt16(buf, 8, de);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PILOT_CONSOLE, buf, 10);
#else
	mavlink_pilot_console_t packet;
	packet.dt = dt;
	packet.dla = dla;
	packet.dra = dra;
	packet.dr = dr;
	packet.de = de;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PILOT_CONSOLE, (const char *)&packet, 10);
#endif
}

#endif

// MESSAGE PILOT_CONSOLE UNPACKING


/**
 * @brief Get field dt from pilot_console message
 *
 * @return Pilot's console throttle command 
 */
static inline UInt16 mavlink_msg_pilot_console_get_dt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  0);
}

/**
 * @brief Get field dla from pilot_console message
 *
 * @return Pilot's console left aileron command 
 */
static inline UInt16 mavlink_msg_pilot_console_get_dla(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  2);
}

/**
 * @brief Get field dra from pilot_console message
 *
 * @return Pilot's console right aileron command 
 */
static inline UInt16 mavlink_msg_pilot_console_get_dra(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  4);
}

/**
 * @brief Get field dr from pilot_console message
 *
 * @return Pilot's console rudder command 
 */
static inline UInt16 mavlink_msg_pilot_console_get_dr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  6);
}

/**
 * @brief Get field de from pilot_console message
 *
 * @return Pilot's console elevator command 
 */
static inline UInt16 mavlink_msg_pilot_console_get_de(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  8);
}

/**
 * @brief Decode a pilot_console message into a struct
 *
 * @param msg The message to decode
 * @param pilot_console C-struct to decode the message contents into
 */
static inline void mavlink_msg_pilot_console_decode(const mavlink_message_t* msg, mavlink_pilot_console_t* pilot_console)
{
#if MAVLINK_NEED_BYTE_SWAP
	pilot_console->dt = mavlink_msg_pilot_console_get_dt(msg);
	pilot_console->dla = mavlink_msg_pilot_console_get_dla(msg);
	pilot_console->dra = mavlink_msg_pilot_console_get_dra(msg);
	pilot_console->dr = mavlink_msg_pilot_console_get_dr(msg);
	pilot_console->de = mavlink_msg_pilot_console_get_de(msg);
#else
	memcpy(pilot_console, _MAV_PAYLOAD(msg), 10);
#endif
}
