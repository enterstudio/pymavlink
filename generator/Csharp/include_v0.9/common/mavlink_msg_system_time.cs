// MESSAGE SYSTEM_TIME PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_system_time_t
    {
        /// <summary>
        /// Timestamp of the master clock in microseconds since UNIX epoch.
        /// </summary>
        public  UInt64 time_usec;
    
    };

/// <summary>
/// * @brief Pack a system_time message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_usec Timestamp of the master clock in microseconds since UNIX epoch.
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_system_time_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));

} else {
    mavlink_system_time_t packet = new mavlink_system_time_t();
	packet.time_usec = time_usec;

        
        int len = 8;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
    //return mavlink_finalize_message(msg, system_id, component_id, 8);
    return 0;
}

/**
 * @brief Pack a system_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_system_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_UInt64(buf, 0, time_usec);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_system_time_t packet;
	packet.time_usec = time_usec;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8);
}
*/
/**
 * @brief Encode a system_time struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_system_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_time_t* system_time)
{
    return mavlink_msg_system_time_pack(system_id, component_id, msg, system_time->time_usec);
}
*/
/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp of the master clock in microseconds since UNIX epoch.
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_send(mavlink_channel_t chan, UInt64 public time_usec)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_UInt64(buf, 0, time_usec);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, buf, 8);
#else
    mavlink_system_time_t packet;
	packet.time_usec = time_usec;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, (const char *)&packet, 8);
#endif
}

#endif
*/
// MESSAGE SYSTEM_TIME UNPACKING


/**
 * @brief Get field time_usec from system_time message
 *
 * @return Timestamp of the master clock in microseconds since UNIX epoch.
 */
public static UInt64 mavlink_msg_system_time_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Decode a system_time message into a struct
 *
 * @param msg The message to decode
 * @param system_time C-struct to decode the message contents into
 */
public static void mavlink_msg_system_time_decode(byte[] msg, ref mavlink_system_time_t system_time)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	system_time.time_usec = mavlink_msg_system_time_get_time_usec(msg);
    
    } else {
        int len = 8; //Marshal.SizeOf(system_time);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        system_time = (mavlink_system_time_t)Marshal.PtrToStructure(i, ((object)system_time).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
