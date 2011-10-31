// MESSAGE PING PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PING = 4;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_ping_t
    {
        /// <summary>
        /// Unix timestamp in microseconds
        /// </summary>
        public  UInt64 time_usec;
            /// <summary>
        /// PING sequence
        /// </summary>
        public  UInt32 seq;
            /// <summary>
        /// 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
        /// </summary>
        public  byte target_component;
    
    };

/// <summary>
/// * @brief Pack a ping message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_usec Unix timestamp in microseconds
/// * @param seq PING sequence
/// * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
/// * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_ping_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec, UInt32 seq, byte target_system, byte target_component)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(seq),0,msg,8,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,12,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,13,sizeof(byte));

} else {
    mavlink_ping_t packet = new mavlink_ping_t();
	packet.time_usec = time_usec;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;

        
        int len = 14;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_PING;
    //return mavlink_finalize_message(msg, system_id, component_id, 14, 237);
    return 0;
}

/**
 * @brief Pack a ping message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Unix timestamp in microseconds
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_ping_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec,UInt32 public seq,byte public target_system,byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_UInt32(buf, 8, seq);
	_mav_put_byte(buf, 12, target_system);
	_mav_put_byte(buf, 13, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_ping_t packet;
	packet.time_usec = time_usec;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_PING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 237);
}
*/
/**
 * @brief Encode a ping struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ping C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_ping_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ping_t* ping)
{
    return mavlink_msg_ping_pack(system_id, component_id, msg, ping->time_usec, ping->seq, ping->target_system, ping->target_component);
}
*/
/**
 * @brief Send a ping message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Unix timestamp in microseconds
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ping_send(mavlink_channel_t chan, UInt64 public time_usec, UInt32 public seq, byte public target_system, byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_UInt32(buf, 8, seq);
	_mav_put_byte(buf, 12, target_system);
	_mav_put_byte(buf, 13, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING, buf, 14, 237);
#else
    mavlink_ping_t packet;
	packet.time_usec = time_usec;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING, (const char *)&packet, 14, 237);
#endif
}

#endif
*/
// MESSAGE PING UNPACKING


/**
 * @brief Get field time_usec from ping message
 *
 * @return Unix timestamp in microseconds
 */
public static UInt64 mavlink_msg_ping_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field seq from ping message
 *
 * @return PING sequence
 */
public static UInt32 mavlink_msg_ping_get_seq(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  8);
}

/**
 * @brief Get field target_system from ping message
 *
 * @return 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
public static byte mavlink_msg_ping_get_target_system(byte[] msg)
{
    return getByte(msg,  12);
}

/**
 * @brief Get field target_component from ping message
 *
 * @return 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
public static byte mavlink_msg_ping_get_target_component(byte[] msg)
{
    return getByte(msg,  13);
}

/**
 * @brief Decode a ping message into a struct
 *
 * @param msg The message to decode
 * @param ping C-struct to decode the message contents into
 */
public static void mavlink_msg_ping_decode(byte[] msg, ref mavlink_ping_t ping)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	ping.time_usec = mavlink_msg_ping_get_time_usec(msg);
    	ping.seq = mavlink_msg_ping_get_seq(msg);
    	ping.target_system = mavlink_msg_ping_get_target_system(msg);
    	ping.target_component = mavlink_msg_ping_get_target_component(msg);
    
    } else {
        int len = 14; //Marshal.SizeOf(ping);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        ping = (mavlink_ping_t)Marshal.PtrToStructure(i, ((object)ping).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}