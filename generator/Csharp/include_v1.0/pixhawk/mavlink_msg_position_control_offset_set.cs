// MESSAGE POSITION_CONTROL_OFFSET_SET PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET = 160;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_position_control_offset_set_t
    {
         public  Single x; /// x position offset
     public  Single y; /// y position offset
     public  Single z; /// z position offset
     public  Single yaw; /// yaw orientation offset in radians, 0 = NORTH
     public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
    
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
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_offset_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET;
    return mavlink_finalize_message(msg, system_id, component_id, 18, 244);
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
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_offset_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 244);
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
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET, buf, 18, 244);
#else
    mavlink_position_control_offset_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_OFFSET_SET, (const char *)&packet, 18, 244);
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
    return getByte(msg,  16);
}

/**
 * @brief Get field target_component from position_control_offset_set message
 *
 * @return Component ID
 */
public static byte mavlink_msg_position_control_offset_set_get_target_component(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field x from position_control_offset_set message
 *
 * @return x position offset
 */
public static Single mavlink_msg_position_control_offset_set_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from position_control_offset_set message
 *
 * @return y position offset
 */
public static Single mavlink_msg_position_control_offset_set_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from position_control_offset_set message
 *
 * @return z position offset
 */
public static Single mavlink_msg_position_control_offset_set_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from position_control_offset_set message
 *
 * @return yaw orientation offset in radians, 0 = NORTH
 */
public static Single mavlink_msg_position_control_offset_set_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
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
	position_control_offset_set.x = mavlink_msg_position_control_offset_set_get_x(msg);
	position_control_offset_set.y = mavlink_msg_position_control_offset_set_get_y(msg);
	position_control_offset_set.z = mavlink_msg_position_control_offset_set_get_z(msg);
	position_control_offset_set.yaw = mavlink_msg_position_control_offset_set_get_yaw(msg);
	position_control_offset_set.target_system = mavlink_msg_position_control_offset_set_get_target_system(msg);
	position_control_offset_set.target_component = mavlink_msg_position_control_offset_set_get_target_component(msg);
} else {
    int len = 18; //Marshal.SizeOf(position_control_offset_set);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    position_control_offset_set = (mavlink_position_control_offset_set_t)Marshal.PtrToStructure(i, ((object)position_control_offset_set).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
