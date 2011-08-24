// MESSAGE AIR_DATA PACKING

#define MAVLINK_MSG_ID_AIR_DATA 171

typedef struct __mavlink_air_data_t
{
 float dynamicPressure; ///< Dynamic pressure (Pa)
 float staticPressure; ///< Static pressure (Pa)
 uint16_t temperature; ///< Board temperature
} mavlink_air_data_t;

/**
 * @brief Pack a air_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float dynamicPressure, float staticPressure, uint16_t temperature)
{
	msg->msgid = MAVLINK_MSG_ID_AIR_DATA;

	put_float_by_index(dynamicPressure, 0,  msg->payload); // Dynamic pressure (Pa)
	put_float_by_index(staticPressure, 4,  msg->payload); // Static pressure (Pa)
	put_uint16_t_by_index(temperature, 8,  msg->payload); // Board temperature

	return mavlink_finalize_message(msg, system_id, component_id, 10, 185);
}

/**
 * @brief Pack a air_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float dynamicPressure,float staticPressure,uint16_t temperature)
{
	msg->msgid = MAVLINK_MSG_ID_AIR_DATA;

	put_float_by_index(dynamicPressure, 0,  msg->payload); // Dynamic pressure (Pa)
	put_float_by_index(staticPressure, 4,  msg->payload); // Static pressure (Pa)
	put_uint16_t_by_index(temperature, 8,  msg->payload); // Board temperature

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 10, 185);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

/**
 * @brief Pack a air_data message on a channel and send
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 */
static inline void mavlink_msg_air_data_pack_chan_send(mavlink_channel_t chan,
							   mavlink_message_t* msg,
						           float dynamicPressure,float staticPressure,uint16_t temperature)
{
	msg->msgid = MAVLINK_MSG_ID_AIR_DATA;

	put_float_by_index(dynamicPressure, 0,  msg->payload); // Dynamic pressure (Pa)
	put_float_by_index(staticPressure, 4,  msg->payload); // Static pressure (Pa)
	put_uint16_t_by_index(temperature, 8,  msg->payload); // Board temperature

	mavlink_finalize_message_chan_send(msg, chan, 10, 185);
}
#endif // MAVLINK_USE_CONVENIENCE_FUNCTIONS


/**
 * @brief Encode a air_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param air_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_air_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_air_data_t* air_data)
{
	return mavlink_msg_air_data_pack(system_id, component_id, msg, air_data->dynamicPressure, air_data->staticPressure, air_data->temperature);
}

/**
 * @brief Send a air_data message
 * @param chan MAVLink channel to send the message
 *
 * @param dynamicPressure Dynamic pressure (Pa)
 * @param staticPressure Static pressure (Pa)
 * @param temperature Board temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_air_data_send(mavlink_channel_t chan, float dynamicPressure, float staticPressure, uint16_t temperature)
{
	MAVLINK_ALIGNED_MESSAGE(msg, 10);
	mavlink_msg_air_data_pack_chan_send(chan, msg, dynamicPressure, staticPressure, temperature);
}

#endif

// MESSAGE AIR_DATA UNPACKING


/**
 * @brief Get field dynamicPressure from air_data message
 *
 * @return Dynamic pressure (Pa)
 */
static inline float mavlink_msg_air_data_get_dynamicPressure(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_float(msg,  0);
}

/**
 * @brief Get field staticPressure from air_data message
 *
 * @return Static pressure (Pa)
 */
static inline float mavlink_msg_air_data_get_staticPressure(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_float(msg,  4);
}

/**
 * @brief Get field temperature from air_data message
 *
 * @return Board temperature
 */
static inline uint16_t mavlink_msg_air_data_get_temperature(const mavlink_message_t* msg)
{
	return MAVLINK_MSG_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Decode a air_data message into a struct
 *
 * @param msg The message to decode
 * @param air_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_air_data_decode(const mavlink_message_t* msg, mavlink_air_data_t* air_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	air_data->dynamicPressure = mavlink_msg_air_data_get_dynamicPressure(msg);
	air_data->staticPressure = mavlink_msg_air_data_get_staticPressure(msg);
	air_data->temperature = mavlink_msg_air_data_get_temperature(msg);
#else
	memcpy(air_data, msg->payload, 10);
#endif
}
