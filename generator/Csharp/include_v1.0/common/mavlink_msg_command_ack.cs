// MESSAGE COMMAND_ACK PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_COMMAND_ACK = 77;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_command_ack_t
    {
        /// <summary>
        /// Command ID, as defined by MAV_CMD enum.
        /// </summary>
        public  UInt16 command;
            /// <summary>
        /// See MAV_RESULT enum
        /// </summary>
        public  byte result;
    
    };

/// <summary>
/// * @brief Pack a command_ack message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param command Command ID, as defined by MAV_CMD enum.
/// * @param result See MAV_RESULT enum
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_command_ack_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 command, byte result)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(command),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(result),0,msg,2,sizeof(byte));

} else {
    mavlink_command_ack_t packet = new mavlink_command_ack_t();
	packet.command = command;
	packet.result = result;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_COMMAND_ACK;
    //return mavlink_finalize_message(msg, system_id, component_id, 3, 143);
    return 0;
}

/**
 * @brief Pack a command_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param result See MAV_RESULT enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_command_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public command,byte public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_UInt16(buf, 0, command);
	_mav_put_byte(buf, 2, result);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_command_ack_t packet;
	packet.command = command;
	packet.result = result;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 143);
}
*/
/**
 * @brief Encode a command_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_ack C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_command_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_ack_t* command_ack)
{
    return mavlink_msg_command_ack_pack(system_id, component_id, msg, command_ack->command, command_ack->result);
}
*/
/**
 * @brief Send a command_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param command Command ID, as defined by MAV_CMD enum.
 * @param result See MAV_RESULT enum
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_ack_send(mavlink_channel_t chan, UInt16 public command, byte public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_UInt16(buf, 0, command);
	_mav_put_byte(buf, 2, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_ACK, buf, 3, 143);
#else
    mavlink_command_ack_t packet;
	packet.command = command;
	packet.result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_ACK, (const char *)&packet, 3, 143);
#endif
}

#endif
*/
// MESSAGE COMMAND_ACK UNPACKING


/**
 * @brief Get field command from command_ack message
 *
 * @return Command ID, as defined by MAV_CMD enum.
 */
public static UInt16 mavlink_msg_command_ack_get_command(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field result from command_ack message
 *
 * @return See MAV_RESULT enum
 */
public static byte mavlink_msg_command_ack_get_result(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Decode a command_ack message into a struct
 *
 * @param msg The message to decode
 * @param command_ack C-struct to decode the message contents into
 */
public static void mavlink_msg_command_ack_decode(byte[] msg, ref mavlink_command_ack_t command_ack)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	command_ack.command = mavlink_msg_command_ack_get_command(msg);
    	command_ack.result = mavlink_msg_command_ack_get_result(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(command_ack);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        command_ack = (mavlink_command_ack_t)Marshal.PtrToStructure(i, ((object)command_ack).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
