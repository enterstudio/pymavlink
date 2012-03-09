// MESSAGE COMMAND PACKING

#define MAVLINK_MSG_ID_COMMAND 75

typedef struct __mavlink_command_t
{
 byte target_system; ///< System which should execute the command
 byte target_component; ///< Component which should execute the command, 0 for all components
 byte command; ///< Command ID, as defined by MAV_CMD enum.
 byte confirmation; ///< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 Single param1; ///< Parameter 1, as defined by MAV_CMD enum.
 Single param2; ///< Parameter 2, as defined by MAV_CMD enum.
 Single param3; ///< Parameter 3, as defined by MAV_CMD enum.
 Single param4; ///< Parameter 4, as defined by MAV_CMD enum.
} mavlink_command_t;

#define MAVLINK_MSG_ID_COMMAND_LEN 20
#define MAVLINK_MSG_ID_75_LEN 20



#define MAVLINK_MESSAGE_INFO_COMMAND { \
	"COMMAND", \
	8, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_command_t, target_component) }, \
         { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_command_t, command) }, \
         { "confirmation", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_command_t, confirmation) }, \
         { "param1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_command_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_command_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_command_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_command_t, param4) }, \
         } \
}


/**
 * @brief Pack a command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       byte target_system, byte target_component, byte command, byte confirmation, Single param1, Single param2, Single param3, Single param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, command);
	_mav_put_byte(buf, 3, confirmation);
	_mav_put_Single(buf, 4, param1);
	_mav_put_Single(buf, 8, param2);
	_mav_put_Single(buf, 12, param3);
	_mav_put_Single(buf, 16, param4);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
	mavlink_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMAND;
	return mavlink_finalize_message(msg, system_id, component_id, 20);
}

/**
 * @brief Pack a command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           byte target_system,byte target_component,byte command,byte confirmation,Single param1,Single param2,Single param3,Single param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, command);
	_mav_put_byte(buf, 3, confirmation);
	_mav_put_Single(buf, 4, param1);
	_mav_put_Single(buf, 8, param2);
	_mav_put_Single(buf, 12, param3);
	_mav_put_Single(buf, 16, param4);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
	mavlink_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

	msg->msgid = MAVLINK_MSG_ID_COMMAND;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20);
}

/**
 * @brief Encode a command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_t* command)
{
	return mavlink_msg_command_pack(system_id, component_id, msg, command->target_system, command->target_component, command->command, command->confirmation, command->param1, command->param2, command->param3, command->param4);
}

/**
 * @brief Send a command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param confirmation 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 * @param param1 Parameter 1, as defined by MAV_CMD enum.
 * @param param2 Parameter 2, as defined by MAV_CMD enum.
 * @param param3 Parameter 3, as defined by MAV_CMD enum.
 * @param param4 Parameter 4, as defined by MAV_CMD enum.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_send(mavlink_channel_t chan, byte target_system, byte target_component, byte command, byte confirmation, Single param1, Single param2, Single param3, Single param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[20];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, command);
	_mav_put_byte(buf, 3, confirmation);
	_mav_put_Single(buf, 4, param1);
	_mav_put_Single(buf, 8, param2);
	_mav_put_Single(buf, 12, param3);
	_mav_put_Single(buf, 16, param4);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, buf, 20);
#else
	mavlink_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND, (const char *)&packet, 20);
#endif
}

#endif

// MESSAGE COMMAND UNPACKING


/**
 * @brief Get field target_system from command message
 *
 * @return System which should execute the command
 */
static inline byte mavlink_msg_command_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  0);
}

/**
 * @brief Get field target_component from command message
 *
 * @return Component which should execute the command, 0 for all components
 */
static inline byte mavlink_msg_command_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  1);
}

/**
 * @brief Get field command from command message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
static inline byte mavlink_msg_command_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  2);
}

/**
 * @brief Get field confirmation from command message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
static inline byte mavlink_msg_command_get_confirmation(const mavlink_message_t* msg)
{
	return _MAV_RETURN_byte(msg,  3);
}

/**
 * @brief Get field param1 from command message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
static inline Single mavlink_msg_command_get_param1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  4);
}

/**
 * @brief Get field param2 from command message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
static inline Single mavlink_msg_command_get_param2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  8);
}

/**
 * @brief Get field param3 from command message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
static inline Single mavlink_msg_command_get_param3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  12);
}

/**
 * @brief Get field param4 from command message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
static inline Single mavlink_msg_command_get_param4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_Single(msg,  16);
}

/**
 * @brief Decode a command message into a struct
 *
 * @param msg The message to decode
 * @param command C-struct to decode the message contents into
 */
static inline void mavlink_msg_command_decode(const mavlink_message_t* msg, mavlink_command_t* command)
{
#if MAVLINK_NEED_BYTE_SWAP
	command->target_system = mavlink_msg_command_get_target_system(msg);
	command->target_component = mavlink_msg_command_get_target_component(msg);
	command->command = mavlink_msg_command_get_command(msg);
	command->confirmation = mavlink_msg_command_get_confirmation(msg);
	command->param1 = mavlink_msg_command_get_param1(msg);
	command->param2 = mavlink_msg_command_get_param2(msg);
	command->param3 = mavlink_msg_command_get_param3(msg);
	command->param4 = mavlink_msg_command_get_param4(msg);
#else
	memcpy(command, _MAV_PAYLOAD(msg), 20);
#endif
}
