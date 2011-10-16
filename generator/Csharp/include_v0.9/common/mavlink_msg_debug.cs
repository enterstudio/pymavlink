// MESSAGE DEBUG PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_DEBUG = 255;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_debug_t
    {
         public  byte ind; /// index of debug variable
     public  Single value; /// DEBUG value
    
    };

/**
 * @brief Pack a debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_debug_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public ind, Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[5];
	_mav_put_byte(buf, 0, ind);
	_mav_put_Single(buf, 1, value);

        memcpy(_MAV_PAYLOAD(msg), buf, 5);
#else
    mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

        memcpy(_MAV_PAYLOAD(msg), &packet, 5);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG;
    return mavlink_finalize_message(msg, system_id, component_id, 5);
}
*/
/**
 * @brief Pack a debug message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param ind index of debug variable
 * @param value DEBUG value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public ind,Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[5];
	_mav_put_byte(buf, 0, ind);
	_mav_put_Single(buf, 1, value);

        memcpy(_MAV_PAYLOAD(msg), buf, 5);
#else
    mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

        memcpy(_MAV_PAYLOAD(msg), &packet, 5);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 5);
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
    return mavlink_msg_debug_pack(system_id, component_id, msg, debug->ind, debug->value);
}
*/
/**
 * @brief Send a debug message
 * @param chan MAVLink channel to send the message
 *
 * @param ind index of debug variable
 * @param value DEBUG value
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_send(mavlink_channel_t chan, byte public ind, Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[5];
	_mav_put_byte(buf, 0, ind);
	_mav_put_Single(buf, 1, value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, buf, 5);
#else
    mavlink_debug_t packet;
	packet.ind = ind;
	packet.value = value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG, (const char *)&packet, 5);
#endif
}

#endif
*/
// MESSAGE DEBUG UNPACKING


/**
 * @brief Get field ind from debug message
 *
 * @return index of debug variable
 */
public static byte mavlink_msg_debug_get_ind(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field value from debug message
 *
 * @return DEBUG value
 */
public static Single mavlink_msg_debug_get_value(byte[] msg)
{
    return BitConverter.ToSingle(msg,  1);
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
	debug.ind = mavlink_msg_debug_get_ind(msg);
	debug.value = mavlink_msg_debug_get_value(msg);
} else {
    int len = 5; //Marshal.SizeOf(debug);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    debug = (mavlink_debug_t)Marshal.PtrToStructure(i, ((object)debug).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
