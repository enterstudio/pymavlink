// MESSAGE MISSION_CURRENT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MISSION_CURRENT = 42;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_mission_current_t
    {
         public  UInt16 seq; /// Sequence
    
    };

/**
 * @brief Pack a mission_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_mission_current_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 seq)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(seq),0,msg,0,sizeof(UInt16));

} else {
    mavlink_mission_current_t packet = new mavlink_mission_current_t();
	packet.seq = seq;

        
        int len = 2;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
    //return mavlink_finalize_message(msg, system_id, component_id, 2, 28);
    return 0;
}

/**
 * @brief Pack a mission_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_mission_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_UInt16(buf, 0, seq);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_mission_current_t packet;
	packet.seq = seq;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 28);
}
*/
/**
 * @brief Encode a mission_current struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_mission_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
    return mavlink_msg_mission_current_pack(system_id, component_id, msg, mission_current->seq);
}
*/
/**
 * @brief Send a mission_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq Sequence
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_current_send(mavlink_channel_t chan, UInt16 public seq)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_UInt16(buf, 0, seq);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, 2, 28);
#else
    mavlink_mission_current_t packet;
	packet.seq = seq;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)&packet, 2, 28);
#endif
}

#endif
*/
// MESSAGE MISSION_CURRENT UNPACKING


/**
 * @brief Get field seq from mission_current message
 *
 * @return Sequence
 */
public static UInt16 mavlink_msg_mission_current_get_seq(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Decode a mission_current message into a struct
 *
 * @param msg The message to decode
 * @param mission_current C-struct to decode the message contents into
 */
public static void mavlink_msg_mission_current_decode(byte[] msg, ref mavlink_mission_current_t mission_current)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	mission_current.seq = mavlink_msg_mission_current_get_seq(msg);
    
    } else {
        int len = 2; //Marshal.SizeOf(mission_current);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        mission_current = (mavlink_mission_current_t)Marshal.PtrToStructure(i, ((object)mission_current).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
