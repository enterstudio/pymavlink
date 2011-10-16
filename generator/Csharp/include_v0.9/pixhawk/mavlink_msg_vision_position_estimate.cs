// MESSAGE VISION_POSITION_ESTIMATE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE = 156;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_vision_position_estimate_t
    {
         public  UInt64 usec; /// Timestamp (milliseconds)
     public  Single x; /// Global X position
     public  Single y; /// Global Y position
     public  Single z; /// Global Z position
     public  Single roll; /// Roll angle in rad
     public  Single pitch; /// Pitch angle in rad
     public  Single yaw; /// Yaw angle in rad
    
    };

/**
 * @brief Pack a vision_position_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_vision_position_estimate_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt64 public usec, Single public x, Single public y, Single public z, Single public roll, Single public pitch, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, roll);
	_mav_put_Single(buf, 24, pitch);
	_mav_put_Single(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, 32);
}
*/
/**
 * @brief Pack a vision_position_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (milliseconds)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_vision_position_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public x,Single public y,Single public z,Single public roll,Single public pitch,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, roll);
	_mav_put_Single(buf, 24, pitch);
	_mav_put_Single(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}
*/
/**
 * @brief Encode a vision_position_estimate struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_position_estimate C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_vision_position_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_position_estimate_t* vision_position_estimate)
{
    return mavlink_msg_vision_position_estimate_pack(system_id, component_id, msg, vision_position_estimate->usec, vision_position_estimate->x, vision_position_estimate->y, vision_position_estimate->z, vision_position_estimate->roll, vision_position_estimate->pitch, vision_position_estimate->yaw);
}
*/
/**
 * @brief Send a vision_position_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_position_estimate_send(mavlink_channel_t chan, UInt64 public usec, Single public x, Single public y, Single public z, Single public roll, Single public pitch, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, roll);
	_mav_put_Single(buf, 24, pitch);
	_mav_put_Single(buf, 28, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, buf, 32);
#else
    mavlink_vision_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, (const char *)&packet, 32);
#endif
}

#endif
*/
// MESSAGE VISION_POSITION_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vision_position_estimate message
 *
 * @return Timestamp (milliseconds)
 */
public static UInt64 mavlink_msg_vision_position_estimate_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field x from vision_position_estimate message
 *
 * @return Global X position
 */
public static Single mavlink_msg_vision_position_estimate_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field y from vision_position_estimate message
 *
 * @return Global Y position
 */
public static Single mavlink_msg_vision_position_estimate_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field z from vision_position_estimate message
 *
 * @return Global Z position
 */
public static Single mavlink_msg_vision_position_estimate_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field roll from vision_position_estimate message
 *
 * @return Roll angle in rad
 */
public static Single mavlink_msg_vision_position_estimate_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field pitch from vision_position_estimate message
 *
 * @return Pitch angle in rad
 */
public static Single mavlink_msg_vision_position_estimate_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field yaw from vision_position_estimate message
 *
 * @return Yaw angle in rad
 */
public static Single mavlink_msg_vision_position_estimate_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a vision_position_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_position_estimate C-struct to decode the message contents into
 */
public static void mavlink_msg_vision_position_estimate_decode(byte[] msg, ref mavlink_vision_position_estimate_t vision_position_estimate)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	vision_position_estimate.usec = mavlink_msg_vision_position_estimate_get_usec(msg);
	vision_position_estimate.x = mavlink_msg_vision_position_estimate_get_x(msg);
	vision_position_estimate.y = mavlink_msg_vision_position_estimate_get_y(msg);
	vision_position_estimate.z = mavlink_msg_vision_position_estimate_get_z(msg);
	vision_position_estimate.roll = mavlink_msg_vision_position_estimate_get_roll(msg);
	vision_position_estimate.pitch = mavlink_msg_vision_position_estimate_get_pitch(msg);
	vision_position_estimate.yaw = mavlink_msg_vision_position_estimate_get_yaw(msg);
} else {
    int len = 32; //Marshal.SizeOf(vision_position_estimate);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    vision_position_estimate = (mavlink_vision_position_estimate_t)Marshal.PtrToStructure(i, ((object)vision_position_estimate).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
