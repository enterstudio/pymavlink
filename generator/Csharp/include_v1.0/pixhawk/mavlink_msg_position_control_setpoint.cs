// MESSAGE POSITION_CONTROL_SETPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT = 170;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_position_control_setpoint_t
    {
         public  Single x; /// x position
     public  Single y; /// y position
     public  Single z; /// z position
     public  Single yaw; /// yaw orientation in radians, 0 = NORTH
     public  UInt16 id; /// ID of waypoint, 0 for plain position
    
    };

/**
 * @brief Pack a position_control_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_position_control_setpoint_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt16 public id, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[18];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_UInt16(buf, 16, id);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, 18, 28);
}
*/
/**
 * @brief Pack a position_control_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_position_control_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public id,Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_UInt16(buf, 16, id);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 28);
}
*/
/**
 * @brief Encode a position_control_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_control_setpoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_position_control_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_setpoint_t* position_control_setpoint)
{
    return mavlink_msg_position_control_setpoint_pack(system_id, component_id, msg, position_control_setpoint->id, position_control_setpoint->x, position_control_setpoint->y, position_control_setpoint->z, position_control_setpoint->yaw);
}
*/
/**
 * @brief Send a position_control_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_setpoint_send(mavlink_channel_t chan, UInt16 public id, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_UInt16(buf, 16, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT, buf, 18, 28);
#else
    mavlink_position_control_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT, (const char *)&packet, 18, 28);
#endif
}

#endif
*/
// MESSAGE POSITION_CONTROL_SETPOINT UNPACKING


/**
 * @brief Get field id from position_control_setpoint message
 *
 * @return ID of waypoint, 0 for plain position
 */
public static UInt16 mavlink_msg_position_control_setpoint_get_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  16);
}

/**
 * @brief Get field x from position_control_setpoint message
 *
 * @return x position
 */
public static Single mavlink_msg_position_control_setpoint_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from position_control_setpoint message
 *
 * @return y position
 */
public static Single mavlink_msg_position_control_setpoint_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from position_control_setpoint message
 *
 * @return z position
 */
public static Single mavlink_msg_position_control_setpoint_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from position_control_setpoint message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
public static Single mavlink_msg_position_control_setpoint_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a position_control_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param position_control_setpoint C-struct to decode the message contents into
 */
public static void mavlink_msg_position_control_setpoint_decode(byte[] msg, ref mavlink_position_control_setpoint_t position_control_setpoint)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	position_control_setpoint.x = mavlink_msg_position_control_setpoint_get_x(msg);
	position_control_setpoint.y = mavlink_msg_position_control_setpoint_get_y(msg);
	position_control_setpoint.z = mavlink_msg_position_control_setpoint_get_z(msg);
	position_control_setpoint.yaw = mavlink_msg_position_control_setpoint_get_yaw(msg);
	position_control_setpoint.id = mavlink_msg_position_control_setpoint_get_id(msg);
} else {
    int len = 18; //Marshal.SizeOf(position_control_setpoint);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    position_control_setpoint = (mavlink_position_control_setpoint_t)Marshal.PtrToStructure(i, ((object)position_control_setpoint).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
