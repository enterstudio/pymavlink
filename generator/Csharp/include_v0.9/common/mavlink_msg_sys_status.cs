// MESSAGE SYS_STATUS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SYS_STATUS = 34;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_sys_status_t
    {
         public  byte mode; /// System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
     public  byte nav_mode; /// Navigation mode, see MAV_NAV_MODE ENUM
     public  byte status; /// System status flag, see MAV_STATUS ENUM
     public  UInt16 load; /// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
     public  UInt16 vbat; /// Battery voltage, in millivolts (1 = 1 millivolt)
     public  UInt16 battery_remaining; /// Remaining battery energy: (0%: 0, 100%: 1000)
     public  UInt16 packet_drop; /// Dropped packets (packets that were corrupted on reception on the MAV)
    
    };

/**
 * @brief Pack a sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param vbat Battery voltage, in millivolts (1 = 1 millivolt)
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 1000)
 * @param packet_drop Dropped packets (packets that were corrupted on reception on the MAV)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_sys_status_pack(byte system_id, byte component_id, byte[] msg,
                               byte mode, byte nav_mode, byte status, UInt16 load, UInt16 vbat, UInt16 battery_remaining, UInt16 packet_drop)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(mode),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(nav_mode),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(status),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(load),0,msg,3,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(vbat),0,msg,5,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(battery_remaining),0,msg,7,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(packet_drop),0,msg,9,sizeof(UInt16));

} else {
    mavlink_sys_status_t packet = new mavlink_sys_status_t();
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.load = load;
	packet.vbat = vbat;
	packet.battery_remaining = battery_remaining;
	packet.packet_drop = packet_drop;

        
        int len = 11;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SYS_STATUS;
    //return mavlink_finalize_message(msg, system_id, component_id, 11);
    return 0;
}

/**
 * @brief Pack a sys_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param vbat Battery voltage, in millivolts (1 = 1 millivolt)
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 1000)
 * @param packet_drop Dropped packets (packets that were corrupted on reception on the MAV)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public mode,byte public nav_mode,byte public status,UInt16 public load,UInt16 public vbat,UInt16 public battery_remaining,UInt16 public packet_drop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[11];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, status);
	_mav_put_UInt16(buf, 3, load);
	_mav_put_UInt16(buf, 5, vbat);
	_mav_put_UInt16(buf, 7, battery_remaining);
	_mav_put_UInt16(buf, 9, packet_drop);

        memcpy(_MAV_PAYLOAD(msg), buf, 11);
#else
    mavlink_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.load = load;
	packet.vbat = vbat;
	packet.battery_remaining = battery_remaining;
	packet.packet_drop = packet_drop;

        memcpy(_MAV_PAYLOAD(msg), &packet, 11);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 11);
}
*/
/**
 * @brief Encode a sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sys_status C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sys_status_t* sys_status)
{
    return mavlink_msg_sys_status_pack(system_id, component_id, msg, sys_status->mode, sys_status->nav_mode, sys_status->status, sys_status->load, sys_status->vbat, sys_status->battery_remaining, sys_status->packet_drop);
}
*/
/**
 * @brief Send a sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param vbat Battery voltage, in millivolts (1 = 1 millivolt)
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 1000)
 * @param packet_drop Dropped packets (packets that were corrupted on reception on the MAV)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sys_status_send(mavlink_channel_t chan, byte public mode, byte public nav_mode, byte public status, UInt16 public load, UInt16 public vbat, UInt16 public battery_remaining, UInt16 public packet_drop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[11];
	_mav_put_byte(buf, 0, mode);
	_mav_put_byte(buf, 1, nav_mode);
	_mav_put_byte(buf, 2, status);
	_mav_put_UInt16(buf, 3, load);
	_mav_put_UInt16(buf, 5, vbat);
	_mav_put_UInt16(buf, 7, battery_remaining);
	_mav_put_UInt16(buf, 9, packet_drop);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, buf, 11);
#else
    mavlink_sys_status_t packet;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.load = load;
	packet.vbat = vbat;
	packet.battery_remaining = battery_remaining;
	packet.packet_drop = packet_drop;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYS_STATUS, (const char *)&packet, 11);
#endif
}

#endif
*/
// MESSAGE SYS_STATUS UNPACKING


/**
 * @brief Get field mode from sys_status message
 *
 * @return System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 */
public static byte mavlink_msg_sys_status_get_mode(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field nav_mode from sys_status message
 *
 * @return Navigation mode, see MAV_NAV_MODE ENUM
 */
public static byte mavlink_msg_sys_status_get_nav_mode(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field status from sys_status message
 *
 * @return System status flag, see MAV_STATUS ENUM
 */
public static byte mavlink_msg_sys_status_get_status(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field load from sys_status message
 *
 * @return Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 */
public static UInt16 mavlink_msg_sys_status_get_load(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  3);
}

/**
 * @brief Get field vbat from sys_status message
 *
 * @return Battery voltage, in millivolts (1 = 1 millivolt)
 */
public static UInt16 mavlink_msg_sys_status_get_vbat(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  5);
}

/**
 * @brief Get field battery_remaining from sys_status message
 *
 * @return Remaining battery energy: (0%: 0, 100%: 1000)
 */
public static UInt16 mavlink_msg_sys_status_get_battery_remaining(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  7);
}

/**
 * @brief Get field packet_drop from sys_status message
 *
 * @return Dropped packets (packets that were corrupted on reception on the MAV)
 */
public static UInt16 mavlink_msg_sys_status_get_packet_drop(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  9);
}

/**
 * @brief Decode a sys_status message into a struct
 *
 * @param msg The message to decode
 * @param sys_status C-struct to decode the message contents into
 */
public static void mavlink_msg_sys_status_decode(byte[] msg, ref mavlink_sys_status_t sys_status)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	sys_status.mode = mavlink_msg_sys_status_get_mode(msg);
    	sys_status.nav_mode = mavlink_msg_sys_status_get_nav_mode(msg);
    	sys_status.status = mavlink_msg_sys_status_get_status(msg);
    	sys_status.load = mavlink_msg_sys_status_get_load(msg);
    	sys_status.vbat = mavlink_msg_sys_status_get_vbat(msg);
    	sys_status.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(msg);
    	sys_status.packet_drop = mavlink_msg_sys_status_get_packet_drop(msg);
    
    } else {
        int len = 11; //Marshal.SizeOf(sys_status);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        sys_status = (mavlink_sys_status_t)Marshal.PtrToStructure(i, ((object)sys_status).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
