// MESSAGE DEBUG PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_DEBUG = 254;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_debug_t
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// DEBUG value
        /// </summary>
        public  Single value;
            /// <summary>
        /// index of debug variable
        /// </summary>
        public  byte ind;
    
    };

/// <summary>
/// * @brief Pack a debug message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp (milliseconds since system boot)
/// * @param ind index of debug variable
/// * @param value DEBUG value
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_debug_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, byte ind, Single value)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(value),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ind),0,msg,8,sizeof(byte));

} else {
    mavlink_debug_t packet = new mavlink_debug_t();
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.ind = ind;

        
        int len = 9;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_DEBUG;
    //return mavlink_finalize_message(msg, system_id, component_id, 9, 46);
    return 0;
}

/**
 * @brief Pack a debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,byte public ind,Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[9];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, value);
	_mav_put_byte(buf, 8, ind);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
    mavlink_debug_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.ind = ind;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9, 46);
}
*/
/**
 * @brief Encode a debug struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_t* debug)
{
    return mavlink_msg_debug_pack(system_id, component_id, msg, debug->time_boot_ms, debug->ind, debug->value);
}
*/
/**
 * @brief Send a debug message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param ind index of debug variable
 * @param value DEBUG value
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_send(mavlink_channel_t chan, UInt32 public time_boot_ms, byte public ind, Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[9];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, value);
	_mav_put_byte(buf, 8, ind);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, 9, 46);
#else
    mavlink_debug_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.ind = ind;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)&packet, 9, 46);
#endif
}

#endif
*/
// MESSAGE DEBUG UNPACKING


/**
 * @brief Get field time_boot_ms from debug message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_debug_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field ind from debug message
 *
 * @return index of debug variable
 */
public static byte mavlink_msg_debug_get_ind(byte[] msg)
{
    return getByte(msg,  8);
}

/**
 * @brief Get field value from debug message
 *
 * @return DEBUG value
 */
public static Single mavlink_msg_debug_get_value(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Decode a debug message into a struct
 *
 * @param msg The message to decode
 * @param debug C-struct to decode the message contents into
 */
public static void mavlink_msg_debug_decode(byte[] msg, ref mavlink_debug_t debug)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	debug.time_boot_ms = mavlink_msg_debug_get_time_boot_ms(msg);
    	debug.value = mavlink_msg_debug_get_value(msg);
    	debug.ind = mavlink_msg_debug_get_ind(msg);
    
    } else {
        int len = 9; //Marshal.SizeOf(debug);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        debug = (mavlink_debug_t)Marshal.PtrToStructure(i, ((object)debug).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
