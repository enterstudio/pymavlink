// MESSAGE SAFETY_SET_ALLOWED_AREA PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA = 54;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_safety_set_allowed_area_t
    {
        /// <summary>
        /// x position 1 / Latitude 1
        /// </summary>
        public  Single p1x;
            /// <summary>
        /// y position 1 / Longitude 1
        /// </summary>
        public  Single p1y;
            /// <summary>
        /// z position 1 / Altitude 1
        /// </summary>
        public  Single p1z;
            /// <summary>
        /// x position 2 / Latitude 2
        /// </summary>
        public  Single p2x;
            /// <summary>
        /// y position 2 / Longitude 2
        /// </summary>
        public  Single p2y;
            /// <summary>
        /// z position 2 / Altitude 2
        /// </summary>
        public  Single p2z;
            /// <summary>
        /// System ID
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component ID
        /// </summary>
        public  byte target_component;
            /// <summary>
        /// Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
        /// </summary>
        public  byte frame;
    
    };

/// <summary>
/// * @brief Pack a safety_set_allowed_area message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param frame Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
/// * @param p1x x position 1 / Latitude 1
/// * @param p1y y position 1 / Longitude 1
/// * @param p1z z position 1 / Altitude 1
/// * @param p2x x position 2 / Latitude 2
/// * @param p2y y position 2 / Longitude 2
/// * @param p2z z position 2 / Altitude 2
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_safety_set_allowed_area_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, byte frame, Single p1x, Single p1y, Single p1z, Single p2x, Single p2y, Single p2z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(p1x),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p1y),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p1z),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p2x),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p2y),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(p2z),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,24,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,25,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(frame),0,msg,26,sizeof(byte));

} else {
    mavlink_safety_set_allowed_area_t packet = new mavlink_safety_set_allowed_area_t();
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;

        
        int len = 27;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    //return mavlink_finalize_message(msg, system_id, component_id, 27, 15);
    return 0;
}

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
	_mav_put_Single(buf, 0, p1x);
	_mav_put_Single(buf, 4, p1y);
	_mav_put_Single(buf, 8, p1z);
	_mav_put_Single(buf, 12, p2x);
	_mav_put_Single(buf, 16, p2y);
	_mav_put_Single(buf, 20, p2z);
	_mav_put_byte(buf, 24, target_system);
	_mav_put_byte(buf, 25, target_component);
	_mav_put_byte(buf, 26, frame);

        memcpy(_MAV_PAYLOAD(msg), buf, 27);
#else
    mavlink_safety_set_allowed_area_t packet;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;

        memcpy(_MAV_PAYLOAD(msg), &packet, 27);
#endif

    msg->msgid = MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 27, 15);
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
	_mav_put_Single(buf, 0, p1x);
	_mav_put_Single(buf, 4, p1y);
	_mav_put_Single(buf, 8, p1z);
	_mav_put_Single(buf, 12, p2x);
	_mav_put_Single(buf, 16, p2y);
	_mav_put_Single(buf, 20, p2z);
	_mav_put_byte(buf, 24, target_system);
	_mav_put_byte(buf, 25, target_component);
	_mav_put_byte(buf, 26, frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, buf, 27, 15);
#else
    mavlink_safety_set_allowed_area_t packet;
	packet.p1x = p1x;
	packet.p1y = p1y;
	packet.p1z = p1z;
	packet.p2x = p2x;
	packet.p2y = p2y;
	packet.p2z = p2z;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA, (const char *)&packet, 27, 15);
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
    return getByte(msg,  24);
}

/**
 * @brief Get field target_component from safety_set_allowed_area message
 *
 * @return Component ID
 */
public static byte mavlink_msg_safety_set_allowed_area_get_target_component(byte[] msg)
{
    return getByte(msg,  25);
}

/**
 * @brief Get field frame from safety_set_allowed_area message
 *
 * @return Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
 */
public static byte mavlink_msg_safety_set_allowed_area_get_frame(byte[] msg)
{
    return getByte(msg,  26);
}

/**
 * @brief Get field p1x from safety_set_allowed_area message
 *
 * @return x position 1 / Latitude 1
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p1x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field p1y from safety_set_allowed_area message
 *
 * @return y position 1 / Longitude 1
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p1y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field p1z from safety_set_allowed_area message
 *
 * @return z position 1 / Altitude 1
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p1z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field p2x from safety_set_allowed_area message
 *
 * @return x position 2 / Latitude 2
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p2x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field p2y from safety_set_allowed_area message
 *
 * @return y position 2 / Longitude 2
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p2y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field p2z from safety_set_allowed_area message
 *
 * @return z position 2 / Altitude 2
 */
public static Single mavlink_msg_safety_set_allowed_area_get_p2z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
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
    	safety_set_allowed_area.p1x = mavlink_msg_safety_set_allowed_area_get_p1x(msg);
    	safety_set_allowed_area.p1y = mavlink_msg_safety_set_allowed_area_get_p1y(msg);
    	safety_set_allowed_area.p1z = mavlink_msg_safety_set_allowed_area_get_p1z(msg);
    	safety_set_allowed_area.p2x = mavlink_msg_safety_set_allowed_area_get_p2x(msg);
    	safety_set_allowed_area.p2y = mavlink_msg_safety_set_allowed_area_get_p2y(msg);
    	safety_set_allowed_area.p2z = mavlink_msg_safety_set_allowed_area_get_p2z(msg);
    	safety_set_allowed_area.target_system = mavlink_msg_safety_set_allowed_area_get_target_system(msg);
    	safety_set_allowed_area.target_component = mavlink_msg_safety_set_allowed_area_get_target_component(msg);
    	safety_set_allowed_area.frame = mavlink_msg_safety_set_allowed_area_get_frame(msg);
    
    } else {
        int len = 27; //Marshal.SizeOf(safety_set_allowed_area);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        safety_set_allowed_area = (mavlink_safety_set_allowed_area_t)Marshal.PtrToStructure(i, ((object)safety_set_allowed_area).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
