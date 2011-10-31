// MESSAGE EXTENDED_MESSAGE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_EXTENDED_MESSAGE = 255;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_extended_message_t
    {
        /// <summary>
        /// System which should execute the command
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component which should execute the command, 0 for all components
        /// </summary>
        public  byte target_component;
            /// <summary>
        /// Retransmission / ACK flags
        /// </summary>
        public  byte protocol_flags;
    
    };

/// <summary>
/// * @brief Pack a extended_message message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System which should execute the command
/// * @param target_component Component which should execute the command, 0 for all components
/// * @param protocol_flags Retransmission / ACK flags
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_extended_message_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, byte protocol_flags)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(protocol_flags),0,msg,2,sizeof(byte));

} else {
    mavlink_extended_message_t packet = new mavlink_extended_message_t();
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.protocol_flags = protocol_flags;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_EXTENDED_MESSAGE;
    //return mavlink_finalize_message(msg, system_id, component_id, 3, 247);
    return 0;
}

/**
 * @brief Pack a extended_message message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param protocol_flags Retransmission / ACK flags
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_extended_message_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public protocol_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, protocol_flags);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_extended_message_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.protocol_flags = protocol_flags;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_EXTENDED_MESSAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 247);
}
*/
/**
 * @brief Encode a extended_message struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param extended_message C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_extended_message_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_extended_message_t* extended_message)
{
    return mavlink_msg_extended_message_pack(system_id, component_id, msg, extended_message->target_system, extended_message->target_component, extended_message->protocol_flags);
}
*/
/**
 * @brief Send a extended_message message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System which should execute the command
 * @param target_component Component which should execute the command, 0 for all components
 * @param protocol_flags Retransmission / ACK flags
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_extended_message_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public protocol_flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, protocol_flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTENDED_MESSAGE, buf, 3, 247);
#else
    mavlink_extended_message_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.protocol_flags = protocol_flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTENDED_MESSAGE, (const char *)&packet, 3, 247);
#endif
}

#endif
*/
// MESSAGE EXTENDED_MESSAGE UNPACKING


/**
 * @brief Get field target_system from extended_message message
 *
 * @return System which should execute the command
 */
public static byte mavlink_msg_extended_message_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from extended_message message
 *
 * @return Component which should execute the command, 0 for all components
 */
public static byte mavlink_msg_extended_message_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field protocol_flags from extended_message message
 *
 * @return Retransmission / ACK flags
 */
public static byte mavlink_msg_extended_message_get_protocol_flags(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Decode a extended_message message into a struct
 *
 * @param msg The message to decode
 * @param extended_message C-struct to decode the message contents into
 */
public static void mavlink_msg_extended_message_decode(byte[] msg, ref mavlink_extended_message_t extended_message)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	extended_message.target_system = mavlink_msg_extended_message_get_target_system(msg);
    	extended_message.target_component = mavlink_msg_extended_message_get_target_component(msg);
    	extended_message.protocol_flags = mavlink_msg_extended_message_get_protocol_flags(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(extended_message);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        extended_message = (mavlink_extended_message_t)Marshal.PtrToStructure(i, ((object)extended_message).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
