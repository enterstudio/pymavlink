// MESSAGE UALBERTA_SYS_STATUS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_UALBERTA_SYS_STATUS = 222;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_ualberta_sys_status_t
    {
         public  byte mode; /// System mode, see UALBERTA_AUTOPILOT_MODE ENUM
     public  byte nav_mode; /// Navigation mode, see UALBERTA_NAV_MODE ENUM
     public  byte pilot; /// Pilot mode, see UALBERTA_PILOT_MODE
    
    };

/**
 * @brief Pack a ualberta_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_ualberta_sys_status_pack(byte system_id, byte component_id, byte[] msg,
                               byte mode, byte nav_mode, byte pilot)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(mode),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(nav_mode),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(pilot),0,msg,2,sizeof(byte));

} else {
    mavlink_ualberta_sys_status_t packet = new mavlink_ualberta_sys_status_t();
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.pilot = pilot;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
    //return mavlink_finalize_message(msg, system_id, component_id, 3, 15);
    return 0;
}

/**
 * @brief Pack a ualberta_sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_ualberta_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public mode,byte public nav_mode,byte public pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, pilot);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_ualberta_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.pilot = pilot;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 15);
}
*/
/**
 * @brief Encode a ualberta_sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_sys_status C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_ualberta_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
    return mavlink_msg_ualberta_sys_status_pack(system_id, component_id, msg, ualberta_sys_status->mode, ualberta_sys_status->nav_mode, ualberta_sys_status->pilot);
}
*/
/**
 * @brief Send a ualberta_sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot Pilot mode, see UALBERTA_PILOT_MODE
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_sys_status_send(mavlink_channel_t chan, byte public mode, byte public nav_mode, byte public pilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, pilot);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, buf, 3, 15);
#else
    mavlink_ualberta_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.pilot = pilot;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UALBERTA_SYS_STATUS, (const char *)&packet, 3, 15);
#endif
}

#endif
*/
// MESSAGE UALBERTA_SYS_STATUS UNPACKING


/**
 * @brief Get field mode from ualberta_sys_status message
 *
 * @return System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 */
public static byte mavlink_msg_ualberta_sys_status_get_mode(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field nav_mode from ualberta_sys_status message
 *
 * @return Navigation mode, see UALBERTA_NAV_MODE ENUM
 */
public static byte mavlink_msg_ualberta_sys_status_get_nav_mode(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field pilot from ualberta_sys_status message
 *
 * @return Pilot mode, see UALBERTA_PILOT_MODE
 */
public static byte mavlink_msg_ualberta_sys_status_get_pilot(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Decode a ualberta_sys_status message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_sys_status C-struct to decode the message contents into
 */
public static void mavlink_msg_ualberta_sys_status_decode(byte[] msg, ref mavlink_ualberta_sys_status_t ualberta_sys_status)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	ualberta_sys_status.mode = mavlink_msg_ualberta_sys_status_get_mode(msg);
    	ualberta_sys_status.nav_mode = mavlink_msg_ualberta_sys_status_get_nav_mode(msg);
    	ualberta_sys_status.pilot = mavlink_msg_ualberta_sys_status_get_pilot(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(ualberta_sys_status);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        ualberta_sys_status = (mavlink_ualberta_sys_status_t)Marshal.PtrToStructure(i, ((object)ualberta_sys_status).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
