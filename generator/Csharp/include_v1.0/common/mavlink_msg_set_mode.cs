// MESSAGE SET_MODE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_MODE = 11;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_mode_t
    {
         public  byte target; /// The system setting the mode
     public  byte mode; /// The new mode
    
    };

/**
 * @brief Pack a set_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the mode
 * @param mode The new mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_set_mode_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target, byte public mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[2];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, mode);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_set_mode_t packet;
	packet.target = target;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_MODE;
    return mavlink_finalize_message(msg, system_id, component_id, 2, 186);
}
*/
/**
 * @brief Pack a set_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the mode
 * @param mode The new mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,byte public mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, mode);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_set_mode_t packet;
	packet.target = target;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_MODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 186);
}
*/
/**
 * @brief Encode a set_mode struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_mode C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_mode_t* set_mode)
{
    return mavlink_msg_set_mode_pack(system_id, component_id, msg, set_mode->target, set_mode->mode);
}
*/
/**
 * @brief Send a set_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the mode
 * @param mode The new mode
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_mode_send(mavlink_channel_t chan, byte public target, byte public mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MODE, buf, 2, 186);
#else
    mavlink_set_mode_t packet;
	packet.target = target;
	packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_MODE, (const char *)&packet, 2, 186);
#endif
}

#endif
*/
// MESSAGE SET_MODE UNPACKING


/**
 * @brief Get field target from set_mode message
 *
 * @return The system setting the mode
 */
public static byte mavlink_msg_set_mode_get_target(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field mode from set_mode message
 *
 * @return The new mode
 */
public static byte mavlink_msg_set_mode_get_mode(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Decode a set_mode message into a struct
 *
 * @param msg The message to decode
 * @param set_mode C-struct to decode the message contents into
 */
public static void mavlink_msg_set_mode_decode(byte[] msg, ref mavlink_set_mode_t set_mode)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	set_mode.target = mavlink_msg_set_mode_get_target(msg);
	set_mode.mode = mavlink_msg_set_mode_get_mode(msg);
} else {
    int len = 2; //Marshal.SizeOf(set_mode);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    set_mode = (mavlink_set_mode_t)Marshal.PtrToStructure(i, ((object)set_mode).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
