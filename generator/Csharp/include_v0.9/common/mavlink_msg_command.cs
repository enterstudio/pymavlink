// MESSAGE COMMAND PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_COMMAND = 75;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_command_t
    {
         public  byte target_system; /// System which should execute the command
     public  byte target_component; /// Component which should execute the command, 0 for all components
     public  byte command; /// Command ID, as defined by MAV_CMD enum.
     public  byte confirmation; /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
     public  Single param1; /// Parameter 1, as defined by MAV_CMD enum.
     public  Single param2; /// Parameter 2, as defined by MAV_CMD enum.
     public  Single param3; /// Parameter 3, as defined by MAV_CMD enum.
     public  Single param4; /// Parameter 4, as defined by MAV_CMD enum.
    
    };

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
 
public static UInt16 mavlink_msg_command_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, byte command, byte confirmation, Single param1, Single param2, Single param3, Single param4)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(command),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(confirmation),0,msg,3,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(param1),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param2),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param3),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param4),0,msg,16,sizeof(Single));

} else {
    mavlink_command_t packet = new mavlink_command_t();
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command = command;
	packet.confirmation = confirmation;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;

        
        int len = 20;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_COMMAND;
    //return mavlink_finalize_message(msg, system_id, component_id, 20);
    return 0;
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
 /*
static inline uint16_t mavlink_msg_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public command,byte public confirmation,Single public param1,Single public param2,Single public param3,Single public param4)
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
*/
/**
 * @brief Encode a command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_t* command)
{
    return mavlink_msg_command_pack(system_id, component_id, msg, command->target_system, command->target_component, command->command, command->confirmation, command->param1, command->param2, command->param3, command->param4);
}
*/
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
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public command, byte public confirmation, Single public param1, Single public param2, Single public param3, Single public param4)
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
*/
// MESSAGE COMMAND UNPACKING


/**
 * @brief Get field target_system from command message
 *
 * @return System which should execute the command
 */
public static byte mavlink_msg_command_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from command message
 *
 * @return Component which should execute the command, 0 for all components
 */
public static byte mavlink_msg_command_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field command from command message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
public static byte mavlink_msg_command_get_command(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field confirmation from command message
 *
 * @return 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
 */
public static byte mavlink_msg_command_get_confirmation(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field param1 from command message
 *
 * @return Parameter 1, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_get_param1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field param2 from command message
 *
 * @return Parameter 2, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_get_param2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field param3 from command message
 *
 * @return Parameter 3, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_get_param3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field param4 from command message
 *
 * @return Parameter 4, as defined by MAV_CMD enum.
 */
public static Single mavlink_msg_command_get_param4(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Decode a command message into a struct
 *
 * @param msg The message to decode
 * @param command C-struct to decode the message contents into
 */
public static void mavlink_msg_command_decode(byte[] msg, ref mavlink_command_t command)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	command.target_system = mavlink_msg_command_get_target_system(msg);
    	command.target_component = mavlink_msg_command_get_target_component(msg);
    	command.command = mavlink_msg_command_get_command(msg);
    	command.confirmation = mavlink_msg_command_get_confirmation(msg);
    	command.param1 = mavlink_msg_command_get_param1(msg);
    	command.param2 = mavlink_msg_command_get_param2(msg);
    	command.param3 = mavlink_msg_command_get_param3(msg);
    	command.param4 = mavlink_msg_command_get_param4(msg);
    
    } else {
        int len = 20; //Marshal.SizeOf(command);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        command = (mavlink_command_t)Marshal.PtrToStructure(i, ((object)command).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
