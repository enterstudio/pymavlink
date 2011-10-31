// MESSAGE WAYPOINT_CURRENT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_WAYPOINT_CURRENT = 42;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_waypoint_current_t
    {
        /// <summary>
        /// Sequence
        /// </summary>
        public  UInt16 seq;
    
    };

/// <summary>
/// * @brief Pack a waypoint_current message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param seq Sequence
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_waypoint_current_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 seq)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(seq),0,msg,0,sizeof(UInt16));

} else {
    mavlink_waypoint_current_t packet = new mavlink_waypoint_current_t();
	packet.seq = seq;

        
        int len = 2;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_WAYPOINT_CURRENT;
    //return mavlink_finalize_message(msg, system_id, component_id, 2);
    return 0;
}

/**
 * @brief Pack a waypoint_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_waypoint_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_UInt16(buf, 0, seq);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_waypoint_current_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_WAYPOINT_CURRENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2);
}
*/
/**
 * @brief Encode a waypoint_current struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_current C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_waypoint_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_current_t* waypoint_current)
{
    return mavlink_msg_waypoint_current_pack(system_id, component_id, msg, waypoint_current->seq);
}
*/
/**
 * @brief Send a waypoint_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_current_send(mavlink_channel_t chan, UInt16 public seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_UInt16(buf, 0, seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_CURRENT, buf, 2);
#else
    mavlink_waypoint_current_t packet;
	packet.seq = seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_CURRENT, (const char *)&packet, 2);
#endif
}

#endif
*/
// MESSAGE WAYPOINT_CURRENT UNPACKING


/**
 * @brief Get field seq from waypoint_current message
 *
 * @return Sequence
 */
public static UInt16 mavlink_msg_waypoint_current_get_seq(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Decode a waypoint_current message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_current C-struct to decode the message contents into
 */
public static void mavlink_msg_waypoint_current_decode(byte[] msg, ref mavlink_waypoint_current_t waypoint_current)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	waypoint_current.seq = mavlink_msg_waypoint_current_get_seq(msg);
    
    } else {
        int len = 2; //Marshal.SizeOf(waypoint_current);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        waypoint_current = (mavlink_waypoint_current_t)Marshal.PtrToStructure(i, ((object)waypoint_current).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
