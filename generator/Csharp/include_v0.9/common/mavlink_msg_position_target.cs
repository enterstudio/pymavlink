// MESSAGE POSITION_TARGET PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POSITION_TARGET = 63;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_position_target_t
    {
         public  Single x; /// x position
     public  Single y; /// y position
     public  Single z; /// z position
     public  Single yaw; /// yaw orientation in radians, 0 = NORTH
    
    };

/**
 * @brief Pack a position_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_position_target_pack(byte system_id, byte component_id, ref byte[] msg,
                               Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[16];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
    mavlink_position_target_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, 16);
}
*/
/**
 * @brief Pack a position_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_position_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
    mavlink_position_target_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16);
}
*/
/**
 * @brief Encode a position_target struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_target C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_position_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_target_t* position_target)
{
    return mavlink_msg_position_target_pack(system_id, component_id, msg, position_target->x, position_target->y, position_target->z, position_target->yaw);
}
*/
/**
 * @brief Send a position_target message
 * @param chan MAVLink channel to send the message
 *
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_target_send(mavlink_channel_t chan, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_TARGET, buf, 16);
#else
    mavlink_position_target_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_TARGET, (const char *)&packet, 16);
#endif
}

#endif
*/
// MESSAGE POSITION_TARGET UNPACKING


/**
 * @brief Get field x from position_target message
 *
 * @return x position
 */
public static Single mavlink_msg_position_target_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from position_target message
 *
 * @return y position
 */
public static Single mavlink_msg_position_target_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from position_target message
 *
 * @return z position
 */
public static Single mavlink_msg_position_target_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from position_target message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
public static Single mavlink_msg_position_target_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a position_target message into a struct
 *
 * @param msg The message to decode
 * @param position_target C-struct to decode the message contents into
 */
public static void mavlink_msg_position_target_decode(byte[] msg, ref mavlink_position_target_t position_target)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	position_target.x = mavlink_msg_position_target_get_x(msg);
	position_target.y = mavlink_msg_position_target_get_y(msg);
	position_target.z = mavlink_msg_position_target_get_z(msg);
	position_target.yaw = mavlink_msg_position_target_get_yaw(msg);
} else {
    int len = 16; //Marshal.SizeOf(position_target);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    position_target = (mavlink_position_target_t)Marshal.PtrToStructure(i, ((object)position_target).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
