// MESSAGE MARKER PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MARKER = 171;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_marker_t
    {
         public  UInt16 id; /// ID
     public  Single x; /// x position
     public  Single y; /// y position
     public  Single z; /// z position
     public  Single roll; /// roll orientation
     public  Single pitch; /// pitch orientation
     public  Single yaw; /// yaw orientation
    
    };

/**
 * @brief Pack a marker message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param roll roll orientation
 * @param pitch pitch orientation
 * @param yaw yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_marker_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt16 public id, Single public x, Single public y, Single public z, Single public roll, Single public pitch, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[26];
	_mav_put_UInt16(buf, 0, id);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, roll);
	_mav_put_Single(buf, 18, pitch);
	_mav_put_Single(buf, 22, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_marker_t packet;
	packet.id = id;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_MARKER;
    return mavlink_finalize_message(msg, system_id, component_id, 26);
}
*/
/**
 * @brief Pack a marker message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param id ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param roll roll orientation
 * @param pitch pitch orientation
 * @param yaw yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_marker_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public id,Single public x,Single public y,Single public z,Single public roll,Single public pitch,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_UInt16(buf, 0, id);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, roll);
	_mav_put_Single(buf, 18, pitch);
	_mav_put_Single(buf, 22, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_marker_t packet;
	packet.id = id;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_MARKER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26);
}
*/
/**
 * @brief Encode a marker struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param marker C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_marker_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_marker_t* marker)
{
    return mavlink_msg_marker_pack(system_id, component_id, msg, marker->id, marker->x, marker->y, marker->z, marker->roll, marker->pitch, marker->yaw);
}
*/
/**
 * @brief Send a marker message
 * @param chan MAVLink channel to send the message
 *
 * @param id ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param roll roll orientation
 * @param pitch pitch orientation
 * @param yaw yaw orientation
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_marker_send(mavlink_channel_t chan, UInt16 public id, Single public x, Single public y, Single public z, Single public roll, Single public pitch, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_UInt16(buf, 0, id);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, roll);
	_mav_put_Single(buf, 18, pitch);
	_mav_put_Single(buf, 22, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER, buf, 26);
#else
    mavlink_marker_t packet;
	packet.id = id;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER, (const char *)&packet, 26);
#endif
}

#endif
*/
// MESSAGE MARKER UNPACKING


/**
 * @brief Get field id from marker message
 *
 * @return ID
 */
public static UInt16 mavlink_msg_marker_get_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field x from marker message
 *
 * @return x position
 */
public static Single mavlink_msg_marker_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  2);
}

/**
 * @brief Get field y from marker message
 *
 * @return y position
 */
public static Single mavlink_msg_marker_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  6);
}

/**
 * @brief Get field z from marker message
 *
 * @return z position
 */
public static Single mavlink_msg_marker_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  10);
}

/**
 * @brief Get field roll from marker message
 *
 * @return roll orientation
 */
public static Single mavlink_msg_marker_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  14);
}

/**
 * @brief Get field pitch from marker message
 *
 * @return pitch orientation
 */
public static Single mavlink_msg_marker_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  18);
}

/**
 * @brief Get field yaw from marker message
 *
 * @return yaw orientation
 */
public static Single mavlink_msg_marker_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  22);
}

/**
 * @brief Decode a marker message into a struct
 *
 * @param msg The message to decode
 * @param marker C-struct to decode the message contents into
 */
public static void mavlink_msg_marker_decode(byte[] msg, ref mavlink_marker_t marker)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	marker.id = mavlink_msg_marker_get_id(msg);
	marker.x = mavlink_msg_marker_get_x(msg);
	marker.y = mavlink_msg_marker_get_y(msg);
	marker.z = mavlink_msg_marker_get_z(msg);
	marker.roll = mavlink_msg_marker_get_roll(msg);
	marker.pitch = mavlink_msg_marker_get_pitch(msg);
	marker.yaw = mavlink_msg_marker_get_yaw(msg);
} else {
    int len = 26; //Marshal.SizeOf(marker);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    marker = (mavlink_marker_t)Marshal.PtrToStructure(i, ((object)marker).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
