// MESSAGE SET_NAV_MODE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_NAV_MODE = 12;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_nav_mode_t
    {
         public  byte target; /// The system setting the mode
     public  byte nav_mode; /// The new navigation mode
    
    };

/**
 * @brief Pack a set_nav_mode message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the mode
 * @param nav_mode The new navigation mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_set_nav_mode_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, byte nav_mode)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(nav_mode),0,msg,1,sizeof(byte));

} else {
    mavlink_set_nav_mode_t packet = new mavlink_set_nav_mode_t();
	packet.target = target;
	packet.nav_mode = nav_mode;

        
        int len = 2;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SET_NAV_MODE;
    //return mavlink_finalize_message(msg, system_id, component_id, 2);
    return 0;
}

/**
 * @brief Pack a set_nav_mode message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the mode
 * @param nav_mode The new navigation mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_nav_mode_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,byte public nav_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, nav_mode);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_set_nav_mode_t packet;
	packet.target = target;
	packet.nav_mode = nav_mode;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_NAV_MODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2);
}
*/
/**
 * @brief Encode a set_nav_mode struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_nav_mode C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_nav_mode_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_nav_mode_t* set_nav_mode)
{
    return mavlink_msg_set_nav_mode_pack(system_id, component_id, msg, set_nav_mode->target, set_nav_mode->nav_mode);
}
*/
/**
 * @brief Send a set_nav_mode message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the mode
 * @param nav_mode The new navigation mode
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_nav_mode_send(mavlink_channel_t chan, byte public target, byte public nav_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, nav_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NAV_MODE, buf, 2);
#else
    mavlink_set_nav_mode_t packet;
	packet.target = target;
	packet.nav_mode = nav_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_NAV_MODE, (const char *)&packet, 2);
#endif
}

#endif
*/
// MESSAGE SET_NAV_MODE UNPACKING


/**
 * @brief Get field target from set_nav_mode message
 *
 * @return The system setting the mode
 */
public static byte mavlink_msg_set_nav_mode_get_target(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field nav_mode from set_nav_mode message
 *
 * @return The new navigation mode
 */
public static byte mavlink_msg_set_nav_mode_get_nav_mode(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Decode a set_nav_mode message into a struct
 *
 * @param msg The message to decode
 * @param set_nav_mode C-struct to decode the message contents into
 */
public static void mavlink_msg_set_nav_mode_decode(byte[] msg, ref mavlink_set_nav_mode_t set_nav_mode)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	set_nav_mode.target = mavlink_msg_set_nav_mode_get_target(msg);
    	set_nav_mode.nav_mode = mavlink_msg_set_nav_mode_get_nav_mode(msg);
    
    } else {
        int len = 2; //Marshal.SizeOf(set_nav_mode);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        set_nav_mode = (mavlink_set_nav_mode_t)Marshal.PtrToStructure(i, ((object)set_nav_mode).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
