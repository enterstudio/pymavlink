// MESSAGE SYSTEM_TIME PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SYSTEM_TIME = 2;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_system_time_t
    {
         public  UInt64 time_unix_usec; /// Timestamp of the master clock in microseconds since UNIX epoch.
     public  UInt32 time_boot_ms; /// Timestamp of the component clock since boot time in milliseconds.
    
    };

/**
 * @brief Pack a system_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_system_time_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_unix_usec, UInt32 time_boot_ms)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_unix_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,8,sizeof(UInt32));

} else {
    mavlink_system_time_t packet = new mavlink_system_time_t();
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_ms = time_boot_ms;

        
        int len = 12;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
    //return mavlink_finalize_message(msg, system_id, component_id, 12, 137);
    return 0;
}

/**
 * @brief Pack a system_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_system_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_unix_usec,UInt32 public time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[12];
	_mav_put_UInt64(buf, 0, time_unix_usec);
	_mav_put_UInt32(buf, 8, time_boot_ms);

        memcpy(_MAV_PAYLOAD(msg), buf, 12);
#else
    mavlink_system_time_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_ms = time_boot_ms;

        memcpy(_MAV_PAYLOAD(msg), &packet, 12);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12, 137);
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
    return mavlink_msg_system_time_pack(system_id, component_id, msg, system_time->time_unix_usec, system_time->time_boot_ms);
}
*/
/**
 * @brief Send a system_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_unix_usec Timestamp of the master clock in microseconds since UNIX epoch.
 * @param time_boot_ms Timestamp of the component clock since boot time in milliseconds.
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_send(mavlink_channel_t chan, UInt64 public time_unix_usec, UInt32 public time_boot_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[12];
	_mav_put_UInt64(buf, 0, time_unix_usec);
	_mav_put_UInt32(buf, 8, time_boot_ms);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, buf, 12, 137);
#else
    mavlink_system_time_t packet;
	packet.time_unix_usec = time_unix_usec;
	packet.time_boot_ms = time_boot_ms;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME, (const char *)&packet, 12, 137);
#endif
}

#endif
*/
// MESSAGE SYSTEM_TIME UNPACKING


/**
 * @brief Get field time_unix_usec from system_time message
 *
 * @return Timestamp of the master clock in microseconds since UNIX epoch.
 */
public static UInt64 mavlink_msg_system_time_get_time_unix_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field time_boot_ms from system_time message
 *
 * @return Timestamp of the component clock since boot time in milliseconds.
 */
public static UInt32 mavlink_msg_system_time_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  8);
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
    	system_time.time_unix_usec = mavlink_msg_system_time_get_time_unix_usec(msg);
    	system_time.time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(msg);
    
    } else {
        int len = 12; //Marshal.SizeOf(system_time);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        system_time = (mavlink_system_time_t)Marshal.PtrToStructure(i, ((object)system_time).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
