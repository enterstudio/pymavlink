// MESSAGE SET_LOCAL_POSITION_SETPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT = 50;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_local_position_setpoint_t
    {
         public  Single x; /// x position
     public  Single y; /// y position
     public  Single z; /// z position
     public  Single yaw; /// Desired yaw angle
     public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
     public  byte coordinate_frame; /// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
    
    };

/**
 * @brief Pack a set_local_position_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_set_local_position_setpoint_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target_system, byte public target_component, byte public coordinate_frame, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[19];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);
	_mav_put_byte(buf, 18, coordinate_frame);

        memcpy(_MAV_PAYLOAD(msg), buf, 19);
#else
    mavlink_set_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD(msg), &packet, 19);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, 19, 214);
}
*/
/**
 * @brief Pack a set_local_position_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_local_position_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public coordinate_frame,Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[19];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);
	_mav_put_byte(buf, 18, coordinate_frame);

        memcpy(_MAV_PAYLOAD(msg), buf, 19);
#else
    mavlink_set_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD(msg), &packet, 19);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 19, 214);
}
*/
/**
 * @brief Encode a set_local_position_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_local_position_setpoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_local_position_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_local_position_setpoint_t* set_local_position_setpoint)
{
    return mavlink_msg_set_local_position_setpoint_pack(system_id, component_id, msg, set_local_position_setpoint->target_system, set_local_position_setpoint->target_component, set_local_position_setpoint->coordinate_frame, set_local_position_setpoint->x, set_local_position_setpoint->y, set_local_position_setpoint->z, set_local_position_setpoint->yaw);
}
*/
/**
 * @brief Send a set_local_position_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_local_position_setpoint_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public coordinate_frame, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[19];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);
	_mav_put_byte(buf, 18, coordinate_frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT, buf, 19, 214);
#else
    mavlink_set_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.coordinate_frame = coordinate_frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_LOCAL_POSITION_SETPOINT, (const char *)&packet, 19, 214);
#endif
}

#endif
*/
// MESSAGE SET_LOCAL_POSITION_SETPOINT UNPACKING


/**
 * @brief Get field target_system from set_local_position_setpoint message
 *
 * @return System ID
 */
public static byte mavlink_msg_set_local_position_setpoint_get_target_system(byte[] msg)
{
    return getByte(msg,  16);
}

/**
 * @brief Get field target_component from set_local_position_setpoint message
 *
 * @return Component ID
 */
public static byte mavlink_msg_set_local_position_setpoint_get_target_component(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field coordinate_frame from set_local_position_setpoint message
 *
 * @return Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU
 */
public static byte mavlink_msg_set_local_position_setpoint_get_coordinate_frame(byte[] msg)
{
    return getByte(msg,  18);
}

/**
 * @brief Get field x from set_local_position_setpoint message
 *
 * @return x position
 */
public static Single mavlink_msg_set_local_position_setpoint_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from set_local_position_setpoint message
 *
 * @return y position
 */
public static Single mavlink_msg_set_local_position_setpoint_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from set_local_position_setpoint message
 *
 * @return z position
 */
public static Single mavlink_msg_set_local_position_setpoint_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from set_local_position_setpoint message
 *
 * @return Desired yaw angle
 */
public static Single mavlink_msg_set_local_position_setpoint_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a set_local_position_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param set_local_position_setpoint C-struct to decode the message contents into
 */
public static void mavlink_msg_set_local_position_setpoint_decode(byte[] msg, ref mavlink_set_local_position_setpoint_t set_local_position_setpoint)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	set_local_position_setpoint.x = mavlink_msg_set_local_position_setpoint_get_x(msg);
    	set_local_position_setpoint.y = mavlink_msg_set_local_position_setpoint_get_y(msg);
    	set_local_position_setpoint.z = mavlink_msg_set_local_position_setpoint_get_z(msg);
    	set_local_position_setpoint.yaw = mavlink_msg_set_local_position_setpoint_get_yaw(msg);
    	set_local_position_setpoint.target_system = mavlink_msg_set_local_position_setpoint_get_target_system(msg);
    	set_local_position_setpoint.target_component = mavlink_msg_set_local_position_setpoint_get_target_component(msg);
    	set_local_position_setpoint.coordinate_frame = mavlink_msg_set_local_position_setpoint_get_coordinate_frame(msg);
    
    } else {
        int len = 19; //Marshal.SizeOf(set_local_position_setpoint);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        set_local_position_setpoint = (mavlink_set_local_position_setpoint_t)Marshal.PtrToStructure(i, ((object)set_local_position_setpoint).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
