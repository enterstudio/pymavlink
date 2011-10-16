// MESSAGE SAFETY_SET_ALLOWED_AREA PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 53;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_safety_set_allowed_area_t
    {
         public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
     public  byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
     public  Single p1x; /// x position 1 / Latitude 1
     public  Single p1y; /// y position 1 / Longitude 1
     public  Single p1z; /// z position 1 / Altitude 1
     public  Single p2x; /// x position 2 / Latitude 2
     public  Single p2y; /// y position 2 / Longitude 2
     public  Single p2z; /// z position 2 / Altitude 2
    
    };

/**
 * @brief Pack a safety_set_allowed_area message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_safety_set_allowed_area_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target_system, byte public target_component, byte public frame, Single public p1x, Single public p1y, Single public p1z, Single public p2x, Single public p2y, Single public p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[27];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, frame);
	_mav_put_Single(buf, 3, p1x);
	_mav_put_Single(buf, 7, p1y);
	_mav_put_Single(buf, 11, p1z);
	_mav_put_Single(buf, 15, p2x);
	_mav_put_Single(buf, 19, p2y);
	_mav_put_Single(buf, 23, p2z);

        memcpy(_MAV_PAYLOAD(msg), buf, 27);
#else
    mavlink_safety_set_allowed_area_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 27);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    return mavlink_finalize_message(msg, system_id, component_id, 27);
}
*/
/**
 * @brief Pack a safety_set_allowed_area message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_safety_set_allowed_area_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public frame,Single public p1x,Single public p1y,Single public p1z,Single public p2x,Single public p2y,Single public p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[27];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, frame);
	_mav_put_Single(buf, 3, p1x);
	_mav_put_Single(buf, 7, p1y);
	_mav_put_Single(buf, 11, p1z);
	_mav_put_Single(buf, 15, p2x);
	_mav_put_Single(buf, 19, p2y);
	_mav_put_Single(buf, 23, p2z);

        memcpy(_MAV_PAYLOAD(msg), buf, 27);
#else
    mavlink_safety_set_allowed_area_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 27);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 27);
}
*/
/**
 * @brief Encode a safety_set_allowed_area struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param safety_set_allowed_area C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_safety_set_allowed_area_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_safety_set_allowed_area_t* safety_set_allowed_area)
{
    return mavlink_msg_safety_set_allowed_area_pack(system_id, component_id, msg, safety_set_allowed_area->target_system, safety_set_allowed_area->target_component, safety_set_allowed_area->frame, safety_set_allowed_area->p1x, safety_set_allowed_area->p1y, safety_set_allowed_area->p1z, safety_set_allowed_area->p2x, safety_set_allowed_area->p2y, safety_set_allowed_area->p2z);
}
*/
/**
 * @brief Send a safety_set_allowed_area message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_safety_set_allowed_area_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public frame, Single public p1x, Single public p1y, Single public p1z, Single public p2x, Single public p2y, Single public p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[27];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, frame);
	_mav_put_Single(buf, 3, p1x);
	_mav_put_Single(buf, 7, p1y);
	_mav_put_Single(buf, 11, p1z);
	_mav_put_Single(buf, 15, p2x);
	_mav_put_Single(buf, 19, p2y);
	_mav_put_Single(buf, 23, p2z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, buf, 27);
#else
    mavlink_safety_set_allowed_area_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, (const char *)&packet, 27);
#endif
}

#endif
*/
// MESSAGE SAFETY_SET_ALLOWED_AREA UNPACKING


/**
 * @brief Get field target_system from safety_set_allowed_area message
 *
 * @return System ID
 */
public static byte mavlink_msg_safety_set_allowed_area_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from safety_set_allowed_area message
 *
 * @return Component ID
 */
public static byte mavlink_msg_safety_set_allowed_area_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field frame from safety_set_allowed_area message
 *
 * @return Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 */
public static byte mavlink_msg_safety_set_allowed_area_get_frame(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field p1x from safety_set_allowed_area message
 *
 * @return x position 1 / Latitude 1
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p1x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  3);
}

/**
 * @brief Get field p1y from safety_set_allowed_area message
 *
 * @return y position 1 / Longitude 1
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p1y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  7);
}

/**
 * @brief Get field p1z from safety_set_allowed_area message
 *
 * @return z position 1 / Altitude 1
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p1z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  11);
}

/**
 * @brief Get field p2x from safety_set_allowed_area message
 *
 * @return x position 2 / Latitude 2
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p2x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  15);
}

/**
 * @brief Get field p2y from safety_set_allowed_area message
 *
 * @return y position 2 / Longitude 2
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p2y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  19);
}

/**
 * @brief Get field p2z from safety_set_allowed_area message
 *
 * @return z position 2 / Altitude 2
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p2z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  23);
}

/**
 * @brief Decode a safety_set_allowed_area message into a struct
 *
 * @param msg The message to decode
 * @param safety_set_allowed_area C-struct to decode the message contents into
 */
public static void mavlink_msg_safety_set_allowed_area_decode(byte[] msg, ref mavlink_safety_set_allowed_area_t safety_set_allowed_area)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	safety_set_allowed_area.target_system = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
	safety_set_allowed_area.target_component = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
	safety_set_allowed_area.frame = mavlink_msg_safety_set_allowed_area_get_frame(msg);
	safety_set_allowed_area.p1x = mavlink_msg_safety_set_allowed_area_get_p1x(msg);
	safety_set_allowed_area.p1y = mavlink_msg_safety_set_allowed_area_get_p1y(msg);
	safety_set_allowed_area.p1z = mavlink_msg_safety_set_allowed_area_get_p1z(msg);
	safety_set_allowed_area.p2x = mavlink_msg_safety_set_allowed_area_get_p2x(msg);
	safety_set_allowed_area.p2y = mavlink_msg_safety_set_allowed_area_get_p2y(msg);
	safety_set_allowed_area.p2z = mavlink_msg_safety_set_allowed_area_get_p2z(msg);
} else {
    int len = 27; //Marshal.SizeOf(safety_set_allowed_area);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    safety_set_allowed_area = (mavlink_safety_set_allowed_area_t)Marshal.PtrToStructure(i, ((object)safety_set_allowed_area).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
