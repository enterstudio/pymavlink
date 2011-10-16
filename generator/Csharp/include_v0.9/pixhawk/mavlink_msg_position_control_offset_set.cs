// MESSAGE POSITION_CONTROL_OFFSET_SET PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET = 160;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_position_control_offset_set_t
    {
         public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
     public  Single x; /// x position offset
     public  Single y; /// y position offset
     public  Single z; /// z position offset
     public  Single yaw; /// yaw orientation offset in radians, 0 = NORTH
    
    };

/**
 * @brief Pack a position_control_offset_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position offset
 * @param y y position offset
 * @param z z position offset
 * @param yaw yaw orientation offset in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_position_control_offset_set_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target_system, byte public target_component, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[18];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_offset_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET;
    return mavlink_finalize_message(msg, system_id, component_id, 18);
}
*/
/**
 * @brief Pack a position_control_offset_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position offset
 * @param y y position offset
 * @param z z position offset
 * @param yaw yaw orientation offset in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_position_control_offset_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_offset_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18);
}
*/
/**
 * @brief Encode a position_control_offset_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_control_offset_set C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_position_control_offset_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_offset_set_t* position_control_offset_set)
{
    return mavlink_msg_position_control_offset_set_pack(system_id, component_id, msg, position_control_offset_set->target_system, position_control_offset_set->target_component, position_control_offset_set->x, position_control_offset_set->y, position_control_offset_set->z, position_control_offset_set->yaw);
}
*/
/**
 * @brief Send a position_control_offset_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param x x position offset
 * @param y y position offset
 * @param z z position offset
 * @param yaw yaw orientation offset in radians, 0 = NORTH
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_offset_set_send(mavlink_channel_t chan, byte public target_system, byte public target_component, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET, buf, 18);
#else
    mavlink_position_control_offset_set_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET, (const char *)&packet, 18);
#endif
}

#endif
*/
// MESSAGE POSITION_CONTROL_OFFSET_SET UNPACKING


/**
 * @brief Get field target_system from position_control_offset_set message
 *
 * @return System ID
 */
public static byte mavlink_msg_position_control_offset_set_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from position_control_offset_set message
 *
 * @return Component ID
 */
public static byte mavlink_msg_position_control_offset_set_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field x from position_control_offset_set message
 *
 * @return x position offset
 */
public static Single mavlink_msg_position_control_offset_set_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  2);
}

/**
 * @brief Get field y from position_control_offset_set message
 *
 * @return y position offset
 */
public static Single mavlink_msg_position_control_offset_set_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  6);
}

/**
 * @brief Get field z from position_control_offset_set message
 *
 * @return z position offset
 */
public static Single mavlink_msg_position_control_offset_set_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  10);
}

/**
 * @brief Get field yaw from position_control_offset_set message
 *
 * @return yaw orientation offset in radians, 0 = NORTH
 */
public static Single mavlink_msg_position_control_offset_set_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  14);
}

/**
 * @brief Decode a position_control_offset_set message into a struct
 *
 * @param msg The message to decode
 * @param position_control_offset_set C-struct to decode the message contents into
 */
public static void mavlink_msg_position_control_offset_set_decode(byte[] msg, ref mavlink_position_control_offset_set_t position_control_offset_set)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	position_control_offset_set.target_system = mavlink_msg_position_control_offset_set_get_target_system(msg);
	position_control_offset_set.target_component = mavlink_msg_position_control_offset_set_get_target_component(msg);
	position_control_offset_set.x = mavlink_msg_position_control_offset_set_get_x(msg);
	position_control_offset_set.y = mavlink_msg_position_control_offset_set_get_y(msg);
	position_control_offset_set.z = mavlink_msg_position_control_offset_set_get_z(msg);
	position_control_offset_set.yaw = mavlink_msg_position_control_offset_set_get_yaw(msg);
} else {
    int len = 18; //Marshal.SizeOf(position_control_offset_set);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    position_control_offset_set = (mavlink_position_control_offset_set_t)Marshal.PtrToStructure(i, ((object)position_control_offset_set).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
