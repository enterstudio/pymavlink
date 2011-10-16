// MESSAGE HEARTBEAT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_heartbeat_t
    {
         public  byte type; /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
     public  byte autopilot; /// Autopilot type / class. defined in MAV_CLASS ENUM
     public  byte mode; /// System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
     public  byte nav_mode; /// Navigation mode, see MAV_NAV_MODE ENUM
     public  byte status; /// System status flag, see MAV_STATUS ENUM
     public  byte safety_status; /// System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
     public  byte link_status; /// Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
     public  byte mavlink_version; /// MAVLink version
    
    };

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param safety_status System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 * @param link_status Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_heartbeat_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public type, byte public autopilot, byte public mode, byte public nav_mode, byte public status, byte public safety_status, byte public link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[8];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, mode);
	_mav_put_byte(buf, 3, nav_mode);
	_mav_put_byte(buf, 4, status);
	_mav_put_byte(buf, 5, safety_status);
	_mav_put_byte(buf, 6, link_status);
	_mav_put_byte(buf, 7, 2);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.safety_status = safety_status;
	packet.link_status = link_status;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, 8, 11);
}
*/
/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param safety_status System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 * @param link_status Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,byte public autopilot,byte public mode,byte public nav_mode,byte public status,byte public safety_status,byte public link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, mode);
	_mav_put_byte(buf, 3, nav_mode);
	_mav_put_byte(buf, 4, status);
	_mav_put_byte(buf, 5, safety_status);
	_mav_put_byte(buf, 6, link_status);
	_mav_put_byte(buf, 7, 2);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.safety_status = safety_status;
	packet.link_status = link_status;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 11);
}
*/
/**
 * @brief Encode a heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot, heartbeat->mode, heartbeat->nav_mode, heartbeat->status, heartbeat->safety_status, heartbeat->link_status);
}
*/
/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param nav_mode Navigation mode, see MAV_NAV_MODE ENUM
 * @param status System status flag, see MAV_STATUS ENUM
 * @param safety_status System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 * @param link_status Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, byte public type, byte public autopilot, byte public mode, byte public nav_mode, byte public status, byte public safety_status, byte public link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, mode);
	_mav_put_byte(buf, 3, nav_mode);
	_mav_put_byte(buf, 4, status);
	_mav_put_byte(buf, 5, safety_status);
	_mav_put_byte(buf, 6, link_status);
	_mav_put_byte(buf, 7, 2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 8, 11);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mode = mode;
	packet.nav_mode = nav_mode;
	packet.status = status;
	packet.safety_status = safety_status;
	packet.link_status = link_status;
	packet.mavlink_version = 2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 8, 11);
#endif
}

#endif
*/
// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
public static byte mavlink_msg_heartbeat_get_type(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Autopilot type / class. defined in MAV_CLASS ENUM
 */
public static byte mavlink_msg_heartbeat_get_autopilot(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field mode from heartbeat message
 *
 * @return System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 */
public static byte mavlink_msg_heartbeat_get_mode(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field nav_mode from heartbeat message
 *
 * @return Navigation mode, see MAV_NAV_MODE ENUM
 */
public static byte mavlink_msg_heartbeat_get_nav_mode(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field status from heartbeat message
 *
 * @return System status flag, see MAV_STATUS ENUM
 */
public static byte mavlink_msg_heartbeat_get_status(byte[] msg)
{
    return getByte(msg,  4);
}

/**
 * @brief Get field safety_status from heartbeat message
 *
 * @return System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 */
public static byte mavlink_msg_heartbeat_get_safety_status(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Get field link_status from heartbeat message
 *
 * @return Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 */
public static byte mavlink_msg_heartbeat_get_link_status(byte[] msg)
{
    return getByte(msg,  6);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
public static byte mavlink_msg_heartbeat_get_mavlink_version(byte[] msg)
{
    return getByte(msg,  7);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
public static void mavlink_msg_heartbeat_decode(byte[] msg, ref mavlink_heartbeat_t heartbeat)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	heartbeat.type = mavlink_msg_heartbeat_get_type(msg);
	heartbeat.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
	heartbeat.mode = mavlink_msg_heartbeat_get_mode(msg);
	heartbeat.nav_mode = mavlink_msg_heartbeat_get_nav_mode(msg);
	heartbeat.status = mavlink_msg_heartbeat_get_status(msg);
	heartbeat.safety_status = mavlink_msg_heartbeat_get_safety_status(msg);
	heartbeat.link_status = mavlink_msg_heartbeat_get_link_status(msg);
	heartbeat.mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
} else {
    int len = 8; //Marshal.SizeOf(heartbeat);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    heartbeat = (mavlink_heartbeat_t)Marshal.PtrToStructure(i, ((object)heartbeat).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
