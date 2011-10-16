// MESSAGE SAFETY_ALLOWED_AREA PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA = 55;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_safety_allowed_area_t
    {
         public  Single p1x; /// x position 1 / Latitude 1
     public  Single p1y; /// y position 1 / Longitude 1
     public  Single p1z; /// z position 1 / Altitude 1
     public  Single p2x; /// x position 2 / Latitude 2
     public  Single p2y; /// y position 2 / Longitude 2
     public  Single p2z; /// z position 2 / Altitude 2
     public  byte frame; /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    
    };

/**
 * @brief Pack a safety_allowed_area message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_safety_allowed_area_pack(byte system_id, byte component_id, byte[] msg,
                               byte frame, Single p1x, Single p1y, Single p1z, Single p2x, Single p2y, Single p2z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(p1x),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p1y),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p1z),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p2x),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p2y),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p2z),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(frame),0,msg,24,sizeof(byte));

} else {
    mavlink_safety_allowed_area_t packet = new mavlink_safety_allowed_area_t();
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;
	packet.frame = frame;

        
        int len = 25;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA;
    //return mavlink_finalize_message(msg, system_id, component_id, 25, 3);
    return 0;
}

/**
 * @brief Pack a safety_allowed_area message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
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
static inline uint16_t mavlink_msg_safety_allowed_area_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public frame,Single public p1x,Single public p1y,Single public p1z,Single public p2x,Single public p2y,Single public p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[25];
	_mav_put_Single(buf, 0, p1x);
	_mav_put_Single(buf, 4, p1y);
	_mav_put_Single(buf, 8, p1z);
	_mav_put_Single(buf, 12, p2x);
	_mav_put_Single(buf, 16, p2y);
	_mav_put_Single(buf, 20, p2z);
	_mav_put_byte(buf, 24, frame);

        memcpy(_MAV_PAYLOAD(msg), buf, 25);
#else
    mavlink_safety_allowed_area_t packet;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;
	packet.frame = frame;

        memcpy(_MAV_PAYLOAD(msg), &packet, 25);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 25, 3);
}
*/
/**
 * @brief Encode a safety_allowed_area struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param safety_allowed_area C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_safety_allowed_area_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_safety_allowed_area_t* safety_allowed_area)
{
    return mavlink_msg_safety_allowed_area_pack(system_id, component_id, msg, safety_allowed_area->frame, safety_allowed_area->p1x, safety_allowed_area->p1y, safety_allowed_area->p1z, safety_allowed_area->p2x, safety_allowed_area->p2y, safety_allowed_area->p2z);
}
*/
/**
 * @brief Send a safety_allowed_area message
 * @param chan MAVLink channel to send the message
 *
 * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 * @param p1x x position 1 / Latitude 1
 * @param p1y y position 1 / Longitude 1
 * @param p1z z position 1 / Altitude 1
 * @param p2x x position 2 / Latitude 2
 * @param p2y y position 2 / Longitude 2
 * @param p2z z position 2 / Altitude 2
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_safety_allowed_area_send(mavlink_channel_t chan, byte public frame, Single public p1x, Single public p1y, Single public p1z, Single public p2x, Single public p2y, Single public p2z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[25];
	_mav_put_Single(buf, 0, p1x);
	_mav_put_Single(buf, 4, p1y);
	_mav_put_Single(buf, 8, p1z);
	_mav_put_Single(buf, 12, p2x);
	_mav_put_Single(buf, 16, p2y);
	_mav_put_Single(buf, 20, p2z);
	_mav_put_byte(buf, 24, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA, buf, 25, 3);
#else
    mavlink_safety_allowed_area_t packet;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;
	packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA, (const char *)&packet, 25, 3);
#endif
}

#endif
*/
// MESSAGE SAFETY_ALLOWED_AREA UNPACKING


/**
 * @brief Get field frame from safety_allowed_area message
 *
 * @return Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 */
public static byte mavlink_msg_safety_allowed_area_get_frame(byte[] msg)
{
    return getByte(msg,  24);
}

/**
 * @brief Get field p1x from safety_allowed_area message
 *
 * @return x position 1 / Latitude 1
 */
public static Single mavlink_msg_safety_allowed_area_get_p1x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field p1y from safety_allowed_area message
 *
 * @return y position 1 / Longitude 1
 */
public static Single mavlink_msg_safety_allowed_area_get_p1y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field p1z from safety_allowed_area message
 *
 * @return z position 1 / Altitude 1
 */
public static Single mavlink_msg_safety_allowed_area_get_p1z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field p2x from safety_allowed_area message
 *
 * @return x position 2 / Latitude 2
 */
public static Single mavlink_msg_safety_allowed_area_get_p2x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field p2y from safety_allowed_area message
 *
 * @return y position 2 / Longitude 2
 */
public static Single mavlink_msg_safety_allowed_area_get_p2y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field p2z from safety_allowed_area message
 *
 * @return z position 2 / Altitude 2
 */
public static Single mavlink_msg_safety_allowed_area_get_p2z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Decode a safety_allowed_area message into a struct
 *
 * @param msg The message to decode
 * @param safety_allowed_area C-struct to decode the message contents into
 */
public static void mavlink_msg_safety_allowed_area_decode(byte[] msg, ref mavlink_safety_allowed_area_t safety_allowed_area)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	safety_allowed_area.p1x = mavlink_msg_safety_allowed_area_get_p1x(msg);
    	safety_allowed_area.p1y = mavlink_msg_safety_allowed_area_get_p1y(msg);
    	safety_allowed_area.p1z = mavlink_msg_safety_allowed_area_get_p1z(msg);
    	safety_allowed_area.p2x = mavlink_msg_safety_allowed_area_get_p2x(msg);
    	safety_allowed_area.p2y = mavlink_msg_safety_allowed_area_get_p2y(msg);
    	safety_allowed_area.p2z = mavlink_msg_safety_allowed_area_get_p2z(msg);
    	safety_allowed_area.frame = mavlink_msg_safety_allowed_area_get_frame(msg);
    
    } else {
        int len = 25; //Marshal.SizeOf(safety_allowed_area);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        safety_allowed_area = (mavlink_safety_allowed_area_t)Marshal.PtrToStructure(i, ((object)safety_allowed_area).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
