// MESSAGE PING PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PING = 3;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_ping_t
    {
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
            /// <summary>
        /// Unix timestamp in microseconds
        /// </summary>
        public  UInt64 time;
    
    };

/// <summary>
/// * @brief Pack a ping message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param seq PING sequence
/// * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
/// * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
/// * @param time Unix timestamp in microseconds
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_ping_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 seq, byte target_system, byte target_component, UInt64 time)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(seq),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,4,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,5,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(time),0,msg,6,sizeof(UInt64));

} else {
    mavlink_ping_t packet = new mavlink_ping_t();
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.time = time;

        
        int len = 14;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_PING;
    //return mavlink_finalize_message(msg, system_id, component_id, 14);
    return 0;
}

/**
 * @brief Pack a ping message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_ping_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public seq,byte public target_system,byte public target_component,UInt64 public time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_UInt32(buf, 0, seq);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_UInt64(buf, 6, time);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_ping_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.time = time;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_PING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14);
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
    return mavlink_msg_ping_pack(system_id, component_id, msg, ping->seq, ping->target_system, ping->target_component, ping->time);
}
*/
/**
 * @brief Send a ping message
 * @param chan MAVLink channel to send the message
 *
 * @param seq PING sequence
 * @param target_system 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param target_component 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 * @param time Unix timestamp in microseconds
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ping_send(mavlink_channel_t chan, UInt32 public seq, byte public target_system, byte public target_component, UInt64 public time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_UInt32(buf, 0, seq);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_UInt64(buf, 6, time);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING, buf, 14);
#else
    mavlink_ping_t packet;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.time = time;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PING, (const char *)&packet, 14);
#endif
}

#endif
*/
// MESSAGE PING UNPACKING


/**
 * @brief Get field seq from ping message
 *
 * @return PING sequence
 */
public static UInt32 mavlink_msg_ping_get_seq(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field target_system from ping message
 *
 * @return 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
public static byte mavlink_msg_ping_get_target_system(byte[] msg)
{
    return getByte(msg,  4);
}

/**
 * @brief Get field target_component from ping message
 *
 * @return 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
 */
public static byte mavlink_msg_ping_get_target_component(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Get field time from ping message
 *
 * @return Unix timestamp in microseconds
 */
public static UInt64 mavlink_msg_ping_get_time(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  6);
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
    	ping.seq = mavlink_msg_ping_get_seq(msg);
    	ping.target_system = mavlink_msg_ping_get_target_system(msg);
    	ping.target_component = mavlink_msg_ping_get_target_component(msg);
    	ping.time = mavlink_msg_ping_get_time(msg);
    
    } else {
        int len = 14; //Marshal.SizeOf(ping);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        ping = (mavlink_ping_t)Marshal.PtrToStructure(i, ((object)ping).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
