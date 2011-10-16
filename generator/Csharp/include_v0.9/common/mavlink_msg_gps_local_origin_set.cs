// MESSAGE GPS_LOCAL_ORIGIN_SET PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET = 49;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_gps_local_origin_set_t
    {
         public  Int32 latitude; /// Latitude (WGS84), expressed as * 1E7
     public  Int32 longitude; /// Longitude (WGS84), expressed as * 1E7
     public  Int32 altitude; /// Altitude(WGS84), expressed as * 1000
    
    };

/**
 * @brief Pack a gps_local_origin_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude Latitude (WGS84), expressed as * 1E7
 * @param longitude Longitude (WGS84), expressed as * 1E7
 * @param altitude Altitude(WGS84), expressed as * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_gps_local_origin_set_pack(byte system_id, byte component_id, byte[] msg,
                               Int32 latitude, Int32 longitude, Int32 altitude)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(latitude),0,msg,0,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(longitude),0,msg,4,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(altitude),0,msg,8,sizeof(Int32));

} else {
    mavlink_gps_local_origin_set_t packet = new mavlink_gps_local_origin_set_t();
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

        
        int len = 12;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET;
    //return mavlink_finalize_message(msg, system_id, component_id, 12);
    return 0;
}

/**
 * @brief Pack a gps_local_origin_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude Latitude (WGS84), expressed as * 1E7
 * @param longitude Longitude (WGS84), expressed as * 1E7
 * @param altitude Altitude(WGS84), expressed as * 1000
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_gps_local_origin_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Int32 public latitude,Int32 public longitude,Int32 public altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[12];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);

        memcpy(_MAV_PAYLOAD(msg), buf, 12);
#else
    mavlink_gps_local_origin_set_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD(msg), &packet, 12);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 12);
}
*/
/**
 * @brief Encode a gps_local_origin_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_local_origin_set C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_gps_local_origin_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_local_origin_set_t* gps_local_origin_set)
{
    return mavlink_msg_gps_local_origin_set_pack(system_id, component_id, msg, gps_local_origin_set->latitude, gps_local_origin_set->longitude, gps_local_origin_set->altitude);
}
*/
/**
 * @brief Send a gps_local_origin_set message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude Latitude (WGS84), expressed as * 1E7
 * @param longitude Longitude (WGS84), expressed as * 1E7
 * @param altitude Altitude(WGS84), expressed as * 1000
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_local_origin_set_send(mavlink_channel_t chan, Int32 public latitude, Int32 public longitude, Int32 public altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[12];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET, buf, 12);
#else
    mavlink_gps_local_origin_set_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_LOCAL_ORIGIN_SET, (const char *)&packet, 12);
#endif
}

#endif
*/
// MESSAGE GPS_LOCAL_ORIGIN_SET UNPACKING


/**
 * @brief Get field latitude from gps_local_origin_set message
 *
 * @return Latitude (WGS84), expressed as * 1E7
 */
public static Int32 mavlink_msg_gps_local_origin_set_get_latitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  0);
}

/**
 * @brief Get field longitude from gps_local_origin_set message
 *
 * @return Longitude (WGS84), expressed as * 1E7
 */
public static Int32 mavlink_msg_gps_local_origin_set_get_longitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
}

/**
 * @brief Get field altitude from gps_local_origin_set message
 *
 * @return Altitude(WGS84), expressed as * 1000
 */
public static Int32 mavlink_msg_gps_local_origin_set_get_altitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Decode a gps_local_origin_set message into a struct
 *
 * @param msg The message to decode
 * @param gps_local_origin_set C-struct to decode the message contents into
 */
public static void mavlink_msg_gps_local_origin_set_decode(byte[] msg, ref mavlink_gps_local_origin_set_t gps_local_origin_set)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	gps_local_origin_set.latitude = mavlink_msg_gps_local_origin_set_get_latitude(msg);
    	gps_local_origin_set.longitude = mavlink_msg_gps_local_origin_set_get_longitude(msg);
    	gps_local_origin_set.altitude = mavlink_msg_gps_local_origin_set_get_altitude(msg);
    
    } else {
        int len = 12; //Marshal.SizeOf(gps_local_origin_set);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        gps_local_origin_set = (mavlink_gps_local_origin_set_t)Marshal.PtrToStructure(i, ((object)gps_local_origin_set).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
