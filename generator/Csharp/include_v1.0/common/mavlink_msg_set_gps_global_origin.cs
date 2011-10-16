// MESSAGE SET_GPS_GLOBAL_ORIGIN PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_gps_global_origin_t
    {
         public  Int32 latitude; /// global position * 1E7
     public  Int32 longitude; /// global position * 1E7
     public  Int32 altitude; /// global position * 1000
     public  byte target_system; /// System ID
    
    };

/**
 * @brief Pack a set_gps_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_set_gps_global_origin_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, Int32 latitude, Int32 longitude, Int32 altitude)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(latitude),0,msg,0,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(longitude),0,msg,4,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(altitude),0,msg,8,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,12,sizeof(byte));

} else {
    mavlink_set_gps_global_origin_t packet = new mavlink_set_gps_global_origin_t();
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.target_system = target_system;

        
        int len = 13;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN;
    //return mavlink_finalize_message(msg, system_id, component_id, 13, 41);
    return 0;
}

/**
 * @brief Pack a set_gps_global_origin message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_gps_global_origin_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,Int32 public latitude,Int32 public longitude,Int32 public altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[13];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_byte(buf, 12, target_system);

        memcpy(_MAV_PAYLOAD(msg), buf, 13);
#else
    mavlink_set_gps_global_origin_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD(msg), &packet, 13);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 13, 41);
}
*/
/**
 * @brief Encode a set_gps_global_origin struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_gps_global_origin C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_gps_global_origin_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_gps_global_origin_t* set_gps_global_origin)
{
    return mavlink_msg_set_gps_global_origin_pack(system_id, component_id, msg, set_gps_global_origin->target_system, set_gps_global_origin->latitude, set_gps_global_origin->longitude, set_gps_global_origin->altitude);
}
*/
/**
 * @brief Send a set_gps_global_origin message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param latitude global position * 1E7
 * @param longitude global position * 1E7
 * @param altitude global position * 1000
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_gps_global_origin_send(mavlink_channel_t chan, byte public target_system, Int32 public latitude, Int32 public longitude, Int32 public altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[13];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_byte(buf, 12, target_system);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN, buf, 13, 41);
#else
    mavlink_set_gps_global_origin_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.target_system = target_system;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN, (const char *)&packet, 13, 41);
#endif
}

#endif
*/
// MESSAGE SET_GPS_GLOBAL_ORIGIN UNPACKING


/**
 * @brief Get field target_system from set_gps_global_origin message
 *
 * @return System ID
 */
public static byte mavlink_msg_set_gps_global_origin_get_target_system(byte[] msg)
{
    return getByte(msg,  12);
}

/**
 * @brief Get field latitude from set_gps_global_origin message
 *
 * @return global position * 1E7
 */
public static Int32 mavlink_msg_set_gps_global_origin_get_latitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  0);
}

/**
 * @brief Get field longitude from set_gps_global_origin message
 *
 * @return global position * 1E7
 */
public static Int32 mavlink_msg_set_gps_global_origin_get_longitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
}

/**
 * @brief Get field altitude from set_gps_global_origin message
 *
 * @return global position * 1000
 */
public static Int32 mavlink_msg_set_gps_global_origin_get_altitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Decode a set_gps_global_origin message into a struct
 *
 * @param msg The message to decode
 * @param set_gps_global_origin C-struct to decode the message contents into
 */
public static void mavlink_msg_set_gps_global_origin_decode(byte[] msg, ref mavlink_set_gps_global_origin_t set_gps_global_origin)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	set_gps_global_origin.latitude = mavlink_msg_set_gps_global_origin_get_latitude(msg);
    	set_gps_global_origin.longitude = mavlink_msg_set_gps_global_origin_get_longitude(msg);
    	set_gps_global_origin.altitude = mavlink_msg_set_gps_global_origin_get_altitude(msg);
    	set_gps_global_origin.target_system = mavlink_msg_set_gps_global_origin_get_target_system(msg);
    
    } else {
        int len = 13; //Marshal.SizeOf(set_gps_global_origin);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        set_gps_global_origin = (mavlink_set_gps_global_origin_t)Marshal.PtrToStructure(i, ((object)set_gps_global_origin).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
