// MESSAGE SYSTEM_TIME_UTC PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SYSTEM_TIME_UTC = 4;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_system_time_utc_t
    {
        /// <summary>
        /// GPS UTC date ddmmyy
        /// </summary>
        public  UInt32 utc_date;
            /// <summary>
        /// GPS UTC time hhmmss
        /// </summary>
        public  UInt32 utc_time;
    
    };

/// <summary>
/// * @brief Pack a system_time_utc message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param utc_date GPS UTC date ddmmyy
/// * @param utc_time GPS UTC time hhmmss
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_system_time_utc_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 utc_date, UInt32 utc_time)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(utc_date),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(utc_time),0,msg,4,sizeof(UInt32));

} else {
    mavlink_system_time_utc_t packet = new mavlink_system_time_utc_t();
	packet.utc_date = utc_date;
	packet.utc_time = utc_time;

        
        int len = 8;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SYSTEM_TIME_UTC;
    //return mavlink_finalize_message(msg, system_id, component_id, 8);
    return 0;
}

/**
 * @brief Pack a system_time_utc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param utc_date GPS UTC date ddmmyy
 * @param utc_time GPS UTC time hhmmss
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_system_time_utc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public utc_date,UInt32 public utc_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_UInt32(buf, 0, utc_date);
	_mav_put_UInt32(buf, 4, utc_time);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_system_time_utc_t packet;
	packet.utc_date = utc_date;
	packet.utc_time = utc_time;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_TIME_UTC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8);
}
*/
/**
 * @brief Encode a system_time_utc struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_time_utc C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_system_time_utc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_time_utc_t* system_time_utc)
{
    return mavlink_msg_system_time_utc_pack(system_id, component_id, msg, system_time_utc->utc_date, system_time_utc->utc_time);
}
*/
/**
 * @brief Send a system_time_utc message
 * @param chan MAVLink channel to send the message
 *
 * @param utc_date GPS UTC date ddmmyy
 * @param utc_time GPS UTC time hhmmss
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_time_utc_send(mavlink_channel_t chan, UInt32 public utc_date, UInt32 public utc_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_UInt32(buf, 0, utc_date);
	_mav_put_UInt32(buf, 4, utc_time);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_UTC, buf, 8);
#else
    mavlink_system_time_utc_t packet;
	packet.utc_date = utc_date;
	packet.utc_time = utc_time;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_TIME_UTC, (const char *)&packet, 8);
#endif
}

#endif
*/
// MESSAGE SYSTEM_TIME_UTC UNPACKING


/**
 * @brief Get field utc_date from system_time_utc message
 *
 * @return GPS UTC date ddmmyy
 */
public static UInt32 mavlink_msg_system_time_utc_get_utc_date(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field utc_time from system_time_utc message
 *
 * @return GPS UTC time hhmmss
 */
public static UInt32 mavlink_msg_system_time_utc_get_utc_time(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  4);
}

/**
 * @brief Decode a system_time_utc message into a struct
 *
 * @param msg The message to decode
 * @param system_time_utc C-struct to decode the message contents into
 */
public static void mavlink_msg_system_time_utc_decode(byte[] msg, ref mavlink_system_time_utc_t system_time_utc)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	system_time_utc.utc_date = mavlink_msg_system_time_utc_get_utc_date(msg);
    	system_time_utc.utc_time = mavlink_msg_system_time_utc_get_utc_time(msg);
    
    } else {
        int len = 8; //Marshal.SizeOf(system_time_utc);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        system_time_utc = (mavlink_system_time_utc_t)Marshal.PtrToStructure(i, ((object)system_time_utc).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
