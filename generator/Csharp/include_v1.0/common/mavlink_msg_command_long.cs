// MESSAGE COMMAND_LONG PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_COMMAND_LONG = 76;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_command_long_t
    {
         public  Single param1; /// Parameter 1, as defined by MAV_CMD enum.
     public  Single param2; /// Parameter 2, as defined by MAV_CMD enum.
     public  Single param3; /// Parameter 3, as defined by MAV_CMD enum.
     public  Single param4; /// Parameter 4, as defined by MAV_CMD enum.
     public  Single param5; /// Parameter 5, as defined by MAV_CMD enum.
     public  Single param6; /// Parameter 6, as defined by MAV_CMD enum.
     public  Single param7; /// Parameter 7, as defined by MAV_CMD enum.
     public  byte target_system; /// System which should execute the command
     public  byte target_component; /// Component which should execute the command, 0 for all components
     public  byte command; /// Command ID, as defined by MAV_CMD enum.
     public  byte confirmation; /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
    
    };

/**
 * @brief Pack a command_long message
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
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_command_long_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target_system, byte public target_component, byte public command, byte public confirmation, Single public param1, Single public param2, Single public param3, Single public param4, Single public param5, Single public param6, Single public param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[32];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, param5);
	_mav_put_Single(buf, 20, param6);
	_mav_put_Single(buf, 24, param7);
	_mav_put_byte(buf, 28, target_system);
	_mav_put_byte(buf, 29, target_component);
	_mav_put_byte(buf, 30, command);
	_mav_put_byte(buf, 31, confirmation);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_command_long_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.param6 = param6;
	packet.param7 = param7;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
    return mavlink_finalize_message(msg, system_id, component_id, 32, 168);
}
*/
/**
 * @brief Pack a command_long message on a channel
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
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_command_long_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public command,byte public confirmation,Single public param1,Single public param2,Single public param3,Single public param4,Single public param5,Single public param6,Single public param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, param5);
	_mav_put_Single(buf, 20, param6);
	_mav_put_Single(buf, 24, param7);
	_mav_put_byte(buf, 28, target_system);
	_mav_put_byte(buf, 29, target_component);
	_mav_put_byte(buf, 30, command);
	_mav_put_byte(buf, 31, confirmation);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_command_long_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.param6 = param6;
	packet.param7 = param7;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32, 168);
}
*/
/**
 * @brief Encode a command_long struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_long C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_command_long_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_long_t* command_long)
{
    return mavlink_msg_command_long_pack(system_id, component_id, msg, command_long->target_system, command_long->target_component, command_long->command, command_long->confirmation, command_long->param1, command_long->param2, command_long->param3, command_long->param4, command_long->param5, command_long->param6, command_long->param7);
}
*/
/**
 * @brief Send a command_long message
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
 * @param param5 Parameter 5, as defined by MAV_CMD enum.
 * @param param6 Parameter 6, as defined by MAV_CMD enum.
 * @param param7 Parameter 7, as defined by MAV_CMD enum.
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_long_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public command, byte public confirmation, Single public param1, Single public param2, Single public param3, Single public param4, Single public param5, Single public param6, Single public param7)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, param5);
	_mav_put_Single(buf, 20, param6);
	_mav_put_Single(buf, 24, param7);
	_mav_put_byte(buf, 28, target_system);
	_mav_put_byte(buf, 29, target_component);
	_mav_put_byte(buf, 30, command);
	_mav_put_byte(buf, 31, confirmation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG, buf, 32, 168);
#else
    mavlink_command_long_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.param5 = param5;
	packet.param6 = param6;
	packet.param7 = param7;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_LONG, (const char *)&packet, 32, 168);
#endif
}

#endif
*/
// MESSAGE COMMAND_LONG UNPACKING


/**
 * @brief Get field target_system from command_long message
 *
 * @return System which should execute the command
 */
public static byte mavlink_msg_command_long_get_target_system(byte[] msg)
{
    return getByte(msg,  28);
}

/**
 * @brief Get field target_component from command_long message
 *
 * @return Component which should execute the command, 0 for all components
 */
public static byte mavlink_msg_command_long_get_target_component(byte[] msg)
{
    return getByte(msg,  29);
}

/**
 * @brief Get field command from command_long message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
public static byte mavlink_msg_command_long_get_command(byte[] msg)
{
    return getByte(msg,  30);
}

/**
 * @brief Get field confirmation from command_long message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
public static byte mavlink_msg_command_long_get_confirmation(byte[] msg)
{
    return getByte(msg,  31);
}

/**
 * @brief Get field param1 from command_long message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field param2 from command_long message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field param3 from command_long message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field param4 from command_long message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param4(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field param5 from command_long message
 *
 * @return Parameter 5, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param5(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field param6 from command_long message
 *
 * @return Parameter 6, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param6(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field param7 from command_long message
 *
 * @return Parameter 7, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_long_get_param7(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Decode a command_long message into a struct
 *
 * @param msg The message to decode
 * @param command_long C-struct to decode the message contents into
 */
public static void mavlink_msg_command_long_decode(byte[] msg, ref mavlink_command_long_t command_long)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	command_long.param1 = mavlink_msg_command_long_get_param1(msg);
	command_long.param2 = mavlink_msg_command_long_get_param2(msg);
	command_long.param3 = mavlink_msg_command_long_get_param3(msg);
	command_long.param4 = mavlink_msg_command_long_get_param4(msg);
	command_long.param5 = mavlink_msg_command_long_get_param5(msg);
	command_long.param6 = mavlink_msg_command_long_get_param6(msg);
	command_long.param7 = mavlink_msg_command_long_get_param7(msg);
	command_long.target_system = mavlink_msg_command_long_get_target_system(msg);
	command_long.target_component = mavlink_msg_command_long_get_target_component(msg);
	command_long.command = mavlink_msg_command_long_get_command(msg);
	command_long.confirmation = mavlink_msg_command_long_get_confirmation(msg);
} else {
    int len = 32; //Marshal.SizeOf(command_long);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    command_long = (mavlink_command_long_t)Marshal.PtrToStructure(i, ((object)command_long).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
