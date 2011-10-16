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
     public  byte system_mode; /// System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
     public  byte flight_mode; /// Navigation mode, see MAV_FLIGHT_MODE ENUM
     public  byte system_status; /// System status flag, see MAV_STATUS ENUM
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
 * @param system_mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param flight_mode Navigation mode, see MAV_FLIGHT_MODE ENUM
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @param safety_status System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 * @param link_status Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_heartbeat_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public type, byte public autopilot, byte public system_mode, byte public flight_mode, byte public system_status, byte public safety_status, byte public link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[8];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, system_mode);
	_mav_put_byte(buf, 3, flight_mode);
	_mav_put_byte(buf, 4, system_status);
	_mav_put_byte(buf, 5, safety_status);
	_mav_put_byte(buf, 6, link_status);
	_mav_put_byte(buf, 7, 3);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.system_mode = system_mode;
	packet.flight_mode = flight_mode;
	packet.system_status = system_status;
	packet.safety_status = safety_status;
	packet.link_status = link_status;
	packet.mavlink_version = 3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, 8, 153);
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
 * @param system_mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param flight_mode Navigation mode, see MAV_FLIGHT_MODE ENUM
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @param safety_status System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 * @param link_status Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,byte public autopilot,byte public system_mode,byte public flight_mode,byte public system_status,byte public safety_status,byte public link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, system_mode);
	_mav_put_byte(buf, 3, flight_mode);
	_mav_put_byte(buf, 4, system_status);
	_mav_put_byte(buf, 5, safety_status);
	_mav_put_byte(buf, 6, link_status);
	_mav_put_byte(buf, 7, 3);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.system_mode = system_mode;
	packet.flight_mode = flight_mode;
	packet.system_status = system_status;
	packet.safety_status = safety_status;
	packet.link_status = link_status;
	packet.mavlink_version = 3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 153);
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
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot, heartbeat->system_mode, heartbeat->flight_mode, heartbeat->system_status, heartbeat->safety_status, heartbeat->link_status);
}
*/
/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param system_mode System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 * @param flight_mode Navigation mode, see MAV_FLIGHT_MODE ENUM
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @param safety_status System safety lock state, see MAV_SAFETY enum. Also indicates HIL operation
 * @param link_status Bitmask showing which links are ok / enabled. 0 for disabled/non functional, 1: enabled Indices: 0: RC, 1: UART1, 2: UART2, 3: UART3, 4: UART4, 5: UART5, 6: I2C, 7: CAN
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, byte public type, byte public autopilot, byte public system_mode, byte public flight_mode, byte public system_status, byte public safety_status, byte public link_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, system_mode);
	_mav_put_byte(buf, 3, flight_mode);
	_mav_put_byte(buf, 4, system_status);
	_mav_put_byte(buf, 5, safety_status);
	_mav_put_byte(buf, 6, link_status);
	_mav_put_byte(buf, 7, 3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 8, 153);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.system_mode = system_mode;
	packet.flight_mode = flight_mode;
	packet.system_status = system_status;
	packet.safety_status = safety_status;
	packet.link_status = link_status;
	packet.mavlink_version = 3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 8, 153);
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
 * @brief Get field system_mode from heartbeat message
 *
 * @return System mode, see MAV_MODE ENUM in mavlink/include/mavlink_types.h
 */
public static byte mavlink_msg_heartbeat_get_system_mode(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field flight_mode from heartbeat message
 *
 * @return Navigation mode, see MAV_FLIGHT_MODE ENUM
 */
public static byte mavlink_msg_heartbeat_get_flight_mode(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field system_status from heartbeat message
 *
 * @return System status flag, see MAV_STATUS ENUM
 */
public static byte mavlink_msg_heartbeat_get_system_status(byte[] msg)
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
	heartbeat.system_mode = mavlink_msg_heartbeat_get_system_mode(msg);
	heartbeat.flight_mode = mavlink_msg_heartbeat_get_flight_mode(msg);
	heartbeat.system_status = mavlink_msg_heartbeat_get_system_status(msg);
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
