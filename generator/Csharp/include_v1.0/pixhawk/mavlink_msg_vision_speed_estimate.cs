// MESSAGE VISION_SPEED_ESTIMATE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE = 158;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_vision_speed_estimate_t
    {
         public  UInt64 usec; /// Timestamp (milliseconds)
     public  Single x; /// Global X speed
     public  Single y; /// Global Y speed
     public  Single z; /// Global Z speed
    
    };

/**
 * @brief Pack a vision_speed_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X speed
 * @param y Global Y speed
 * @param z Global Z speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_vision_speed_estimate_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt64 public usec, Single public x, Single public y, Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[20];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_vision_speed_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, 20, 208);
}
*/
/**
 * @brief Pack a vision_speed_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (milliseconds)
 * @param x Global X speed
 * @param y Global Y speed
 * @param z Global Z speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_vision_speed_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public x,Single public y,Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_vision_speed_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 208);
}
*/
/**
 * @brief Encode a vision_speed_estimate struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_speed_estimate C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_vision_speed_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_speed_estimate_t* vision_speed_estimate)
{
    return mavlink_msg_vision_speed_estimate_pack(system_id, component_id, msg, vision_speed_estimate->usec, vision_speed_estimate->x, vision_speed_estimate->y, vision_speed_estimate->z);
}
*/
/**
 * @brief Send a vision_speed_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X speed
 * @param y Global Y speed
 * @param z Global Z speed
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_speed_estimate_send(mavlink_channel_t chan, UInt64 public usec, Single public x, Single public y, Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, buf, 20, 208);
#else
    mavlink_vision_speed_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE, (const char *)&packet, 20, 208);
#endif
}

#endif
*/
// MESSAGE VISION_SPEED_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vision_speed_estimate message
 *
 * @return Timestamp (milliseconds)
 */
public static UInt64 mavlink_msg_vision_speed_estimate_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field x from vision_speed_estimate message
 *
 * @return Global X speed
 */
public static Single mavlink_msg_vision_speed_estimate_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field y from vision_speed_estimate message
 *
 * @return Global Y speed
 */
public static Single mavlink_msg_vision_speed_estimate_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field z from vision_speed_estimate message
 *
 * @return Global Z speed
 */
public static Single mavlink_msg_vision_speed_estimate_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Decode a vision_speed_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vision_speed_estimate C-struct to decode the message contents into
 */
public static void mavlink_msg_vision_speed_estimate_decode(byte[] msg, ref mavlink_vision_speed_estimate_t vision_speed_estimate)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	vision_speed_estimate.usec = mavlink_msg_vision_speed_estimate_get_usec(msg);
	vision_speed_estimate.x = mavlink_msg_vision_speed_estimate_get_x(msg);
	vision_speed_estimate.y = mavlink_msg_vision_speed_estimate_get_y(msg);
	vision_speed_estimate.z = mavlink_msg_vision_speed_estimate_get_z(msg);
} else {
    int len = 20; //Marshal.SizeOf(vision_speed_estimate);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    vision_speed_estimate = (mavlink_vision_speed_estimate_t)Marshal.PtrToStructure(i, ((object)vision_speed_estimate).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
