// MESSAGE PWM_COMMANDS PACKING

#define MAVLINK_MSG_ID_PWM_COMMANDS 175

typedef struct __mavlink_pwm_commands_t
{
 UInt16 dt_c; ///< AutoPilot's throttle command 
 UInt16 dla_c; ///< AutoPilot's left aileron command 
 UInt16 dra_c; ///< AutoPilot's right aileron command 
 UInt16 dr_c; ///< AutoPilot's rudder command 
 UInt16 dle_c; ///< AutoPilot's left elevator command 
 UInt16 dre_c; ///< AutoPilot's right elevator command 
 UInt16 dlf_c; ///< AutoPilot's left  flap command 
 UInt16 drf_c; ///< AutoPilot's right flap command 
 UInt16 aux1; ///< AutoPilot's aux1 command 
 UInt16 aux2; ///< AutoPilot's aux2 command 
} mavlink_pwm_commands_t;

#define MAVLINK_MSG_ID_PWM_COMMANDS_LEN 20
#define MAVLINK_MSG_ID_175_LEN 20



#define MAVLINK_MESSAGE_INFO_PWM_COMMANDS { \
	"PWM_COMMANDS", \
	10, \
	{  { "dt_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_pwm_commands_t, dt_c) }, \
         { "dla_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_pwm_commands_t, dla_c) }, \
         { "dra_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_pwm_commands_t, dra_c) }, \
         { "dr_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_pwm_commands_t, dr_c) }, \
         { "dle_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_pwm_commands_t, dle_c) }, \
         { "dre_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_pwm_commands_t, dre_c) }, \
         { "dlf_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_pwm_commands_t, dlf_c) }, \
         { "drf_c", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_pwm_commands_t, drf_c) }, \
         { "aux1", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_pwm_commands_t, aux1) }, \
         { "aux2", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_pwm_commands_t, aux2) }, \
         } \
}


/**
 * @brief Pack a pwm_commands message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param dt_c AutoPilot's throttle command 
 * @param dla_c AutoPilot's left aileron command 
 * @param dra_c AutoPilot's right aileron command 
 * @param dr_c AutoPilot's rudder command 
 * @param dle_c AutoPilot's left elevator command 
 * @param dre_c AutoPilot's right elevator command 
 * @param dlf_c AutoPilot's left  flap command 
 * @param drf_c AutoPilot's right flap command 
 * @param aux1 AutoPilot's aux1 command 
 * @param aux2 AutoPilot's aux2 command 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pwm_commands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       UInt16 dt_c, UInt16 dla_c, UInt16 dra_c, UInt16 dr_c, UInt16 dle_c, UInt16 dre_c, UInt16 dlf_c, UInt16 drf_c, UInt16 aux1, UInt16 aux2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_UInt16(buf, 0, dt_c);
	_mav_put_UInt16(buf, 2, dla_c);
	_mav_put_UInt16(buf, 4, dra_c);
	_mav_put_UInt16(buf, 6, dr_c);
	_mav_put_UInt16(buf, 8, dle_c);
	_mav_put_UInt16(buf, 10, dre_c);
	_mav_put_UInt16(buf, 12, dlf_c);
	_mav_put_UInt16(buf, 14, drf_c);
	_mav_put_UInt16(buf, 16, aux1);
	_mav_put_UInt16(buf, 18, aux2);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
	mavlink_pwm_commands_t packet;
	packet.dt_c = dt_c;
	packet.dla_c = dla_c;
	packet.dra_c = dra_c;
	packet.dr_c = dr_c;
	packet.dle_c = dle_c;
	packet.dre_c = dre_c;
	packet.dlf_c = dlf_c;
	packet.drf_c = drf_c;
	packet.aux1 = aux1;
	packet.aux2 = aux2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_PWM_COMMANDS;
	return mavlink_finalize_message(msg, system_id, component_id, 20);
}

/**
 * @brief Pack a pwm_commands message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param dt_c AutoPilot's throttle command 
 * @param dla_c AutoPilot's left aileron command 
 * @param dra_c AutoPilot's right aileron command 
 * @param dr_c AutoPilot's rudder command 
 * @param dle_c AutoPilot's left elevator command 
 * @param dre_c AutoPilot's right elevator command 
 * @param dlf_c AutoPilot's left  flap command 
 * @param drf_c AutoPilot's right flap command 
 * @param aux1 AutoPilot's aux1 command 
 * @param aux2 AutoPilot's aux2 command 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pwm_commands_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           UInt16 dt_c,UInt16 dla_c,UInt16 dra_c,UInt16 dr_c,UInt16 dle_c,UInt16 dre_c,UInt16 dlf_c,UInt16 drf_c,UInt16 aux1,UInt16 aux2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_UInt16(buf, 0, dt_c);
	_mav_put_UInt16(buf, 2, dla_c);
	_mav_put_UInt16(buf, 4, dra_c);
	_mav_put_UInt16(buf, 6, dr_c);
	_mav_put_UInt16(buf, 8, dle_c);
	_mav_put_UInt16(buf, 10, dre_c);
	_mav_put_UInt16(buf, 12, dlf_c);
	_mav_put_UInt16(buf, 14, drf_c);
	_mav_put_UInt16(buf, 16, aux1);
	_mav_put_UInt16(buf, 18, aux2);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
	mavlink_pwm_commands_t packet;
	packet.dt_c = dt_c;
	packet.dla_c = dla_c;
	packet.dra_c = dra_c;
	packet.dr_c = dr_c;
	packet.dle_c = dle_c;
	packet.dre_c = dre_c;
	packet.dlf_c = dlf_c;
	packet.drf_c = drf_c;
	packet.aux1 = aux1;
	packet.aux2 = aux2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_PWM_COMMANDS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20);
}

/**
 * @brief Encode a pwm_commands struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pwm_commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pwm_commands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pwm_commands_t* pwm_commands)
{
	return mavlink_msg_pwm_commands_pack(system_id, component_id, msg, pwm_commands->dt_c, pwm_commands->dla_c, pwm_commands->dra_c, pwm_commands->dr_c, pwm_commands->dle_c, pwm_commands->dre_c, pwm_commands->dlf_c, pwm_commands->drf_c, pwm_commands->aux1, pwm_commands->aux2);
}

/**
 * @brief Send a pwm_commands message
 * @param chan MAVLink channel to send the message
 *
 * @param dt_c AutoPilot's throttle command 
 * @param dla_c AutoPilot's left aileron command 
 * @param dra_c AutoPilot's right aileron command 
 * @param dr_c AutoPilot's rudder command 
 * @param dle_c AutoPilot's left elevator command 
 * @param dre_c AutoPilot's right elevator command 
 * @param dlf_c AutoPilot's left  flap command 
 * @param drf_c AutoPilot's right flap command 
 * @param aux1 AutoPilot's aux1 command 
 * @param aux2 AutoPilot's aux2 command 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pwm_commands_send(mavlink_channel_t chan, UInt16 dt_c, UInt16 dla_c, UInt16 dra_c, UInt16 dr_c, UInt16 dle_c, UInt16 dre_c, UInt16 dlf_c, UInt16 drf_c, UInt16 aux1, UInt16 aux2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_UInt16(buf, 0, dt_c);
	_mav_put_UInt16(buf, 2, dla_c);
	_mav_put_UInt16(buf, 4, dra_c);
	_mav_put_UInt16(buf, 6, dr_c);
	_mav_put_UInt16(buf, 8, dle_c);
	_mav_put_UInt16(buf, 10, dre_c);
	_mav_put_UInt16(buf, 12, dlf_c);
	_mav_put_UInt16(buf, 14, drf_c);
	_mav_put_UInt16(buf, 16, aux1);
	_mav_put_UInt16(buf, 18, aux2);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PWM_COMMANDS, buf, 20);
#else
	mavlink_pwm_commands_t packet;
	packet.dt_c = dt_c;
	packet.dla_c = dla_c;
	packet.dra_c = dra_c;
	packet.dr_c = dr_c;
	packet.dle_c = dle_c;
	packet.dre_c = dre_c;
	packet.dlf_c = dlf_c;
	packet.drf_c = drf_c;
	packet.aux1 = aux1;
	packet.aux2 = aux2;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PWM_COMMANDS, (const char *)&packet, 20);
#endif
}

#endif

// MESSAGE PWM_COMMANDS UNPACKING


/**
 * @brief Get field dt_c from pwm_commands message
 *
 * @return AutoPilot's throttle command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dt_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  0);
}

/**
 * @brief Get field dla_c from pwm_commands message
 *
 * @return AutoPilot's left aileron command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dla_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  2);
}

/**
 * @brief Get field dra_c from pwm_commands message
 *
 * @return AutoPilot's right aileron command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dra_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  4);
}

/**
 * @brief Get field dr_c from pwm_commands message
 *
 * @return AutoPilot's rudder command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dr_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  6);
}

/**
 * @brief Get field dle_c from pwm_commands message
 *
 * @return AutoPilot's left elevator command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dle_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  8);
}

/**
 * @brief Get field dre_c from pwm_commands message
 *
 * @return AutoPilot's right elevator command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dre_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  10);
}

/**
 * @brief Get field dlf_c from pwm_commands message
 *
 * @return AutoPilot's left  flap command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_dlf_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  12);
}

/**
 * @brief Get field drf_c from pwm_commands message
 *
 * @return AutoPilot's right flap command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_drf_c(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  14);
}

/**
 * @brief Get field aux1 from pwm_commands message
 *
 * @return AutoPilot's aux1 command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_aux1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  16);
}

/**
 * @brief Get field aux2 from pwm_commands message
 *
 * @return AutoPilot's aux2 command 
 */
static inline UInt16 mavlink_msg_pwm_commands_get_aux2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_UInt16(msg,  18);
}

/**
 * @brief Decode a pwm_commands message into a struct
 *
 * @param msg The message to decode
 * @param pwm_commands C-struct to decode the message contents into
 */
static inline void mavlink_msg_pwm_commands_decode(const mavlink_message_t* msg, mavlink_pwm_commands_t* pwm_commands)
{
#if MAVLINK_NEED_BYTE_SWAP
	pwm_commands->dt_c = mavlink_msg_pwm_commands_get_dt_c(msg);
	pwm_commands->dla_c = mavlink_msg_pwm_commands_get_dla_c(msg);
	pwm_commands->dra_c = mavlink_msg_pwm_commands_get_dra_c(msg);
	pwm_commands->dr_c = mavlink_msg_pwm_commands_get_dr_c(msg);
	pwm_commands->dle_c = mavlink_msg_pwm_commands_get_dle_c(msg);
	pwm_commands->dre_c = mavlink_msg_pwm_commands_get_dre_c(msg);
	pwm_commands->dlf_c = mavlink_msg_pwm_commands_get_dlf_c(msg);
	pwm_commands->drf_c = mavlink_msg_pwm_commands_get_drf_c(msg);
	pwm_commands->aux1 = mavlink_msg_pwm_commands_get_aux1(msg);
	pwm_commands->aux2 = mavlink_msg_pwm_commands_get_aux2(msg);
#else
	memcpy(pwm_commands, _MAV_PAYLOAD(msg), 20);
#endif
}
