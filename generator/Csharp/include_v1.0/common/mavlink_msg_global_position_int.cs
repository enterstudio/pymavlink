// MESSAGE GLOBAL_POSITION_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 73;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_global_position_int_t
    {
         public  Int32 lat; /// Latitude, expressed as * 1E7
     public  Int32 lon; /// Longitude, expressed as * 1E7
     public  Int32 alt; /// Altitude in meters, expressed as * 1000 (millimeters), above MSL
     public  Int16 vx; /// Ground X Speed (Latitude), expressed as m/s * 100
     public  Int16 vy; /// Ground Y Speed (Longitude), expressed as m/s * 100
     public  Int16 vz; /// Ground Z Speed (Altitude), expressed as m/s * 100
     public  UInt16 hdg; /// Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
    
    };

/**
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_global_position_int_pack(byte system_id, byte component_id, ref byte[] msg,
                               Int32 public lat, Int32 public lon, Int32 public alt, Int16 public vx, Int16 public vy, Int16 public vz, UInt16 public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[20];
	_mav_put_Int32(buf, 0, lat);
	_mav_put_Int32(buf, 4, lon);
	_mav_put_Int32(buf, 8, alt);
	_mav_put_Int16(buf, 12, vx);
	_mav_put_Int16(buf, 14, vy);
	_mav_put_Int16(buf, 16, vz);
	_mav_put_UInt16(buf, 18, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_global_position_int_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    return mavlink_finalize_message(msg, system_id, component_id, 20, 241);
}
*/
/**
 * @brief Pack a global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Int32 public lat,Int32 public lon,Int32 public alt,Int16 public vx,Int16 public vy,Int16 public vz,UInt16 public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Int32(buf, 0, lat);
	_mav_put_Int32(buf, 4, lon);
	_mav_put_Int32(buf, 8, alt);
	_mav_put_Int16(buf, 12, vx);
	_mav_put_Int16(buf, 14, vy);
	_mav_put_Int16(buf, 16, vz);
	_mav_put_UInt16(buf, 18, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_global_position_int_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 241);
}
*/
/**
 * @brief Encode a global_position_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_int C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_int_t* global_position_int)
{
    return mavlink_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->vx, global_position_int->vy, global_position_int->vz, global_position_int->hdg);
}
*/
/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int_send(mavlink_channel_t chan, Int32 public lat, Int32 public lon, Int32 public alt, Int16 public vx, Int16 public vy, Int16 public vz, UInt16 public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Int32(buf, 0, lat);
	_mav_put_Int32(buf, 4, lon);
	_mav_put_Int32(buf, 8, alt);
	_mav_put_Int16(buf, 12, vx);
	_mav_put_Int16(buf, 14, vy);
	_mav_put_Int16(buf, 16, vz);
	_mav_put_UInt16(buf, 18, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, 20, 241);
#else
    mavlink_global_position_int_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, 20, 241);
#endif
}

#endif
*/
// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude, expressed as * 1E7
 */
public static Int32 mavlink_msg_global_position_int_get_lat(byte[] msg)
{
    return BitConverter.ToInt32(msg,  0);
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude, expressed as * 1E7
 */
public static Int32 mavlink_msg_global_position_int_get_lon(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
public static Int32 mavlink_msg_global_position_int_get_alt(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Get field vx from global_position_int message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
public static Int16 mavlink_msg_global_position_int_get_vx(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field vy from global_position_int message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
public static Int16 mavlink_msg_global_position_int_get_vy(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Get field vz from global_position_int message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
public static Int16 mavlink_msg_global_position_int_get_vz(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Get field hdg from global_position_int message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
public static UInt16 mavlink_msg_global_position_int_get_hdg(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  18);
}

/**
 * @brief Decode a global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_int C-struct to decode the message contents into
 */
public static void mavlink_msg_global_position_int_decode(byte[] msg, ref mavlink_global_position_int_t global_position_int)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	global_position_int.lat = mavlink_msg_global_position_int_get_lat(msg);
	global_position_int.lon = mavlink_msg_global_position_int_get_lon(msg);
	global_position_int.alt = mavlink_msg_global_position_int_get_alt(msg);
	global_position_int.vx = mavlink_msg_global_position_int_get_vx(msg);
	global_position_int.vy = mavlink_msg_global_position_int_get_vy(msg);
	global_position_int.vz = mavlink_msg_global_position_int_get_vz(msg);
	global_position_int.hdg = mavlink_msg_global_position_int_get_hdg(msg);
} else {
    int len = 20; //Marshal.SizeOf(global_position_int);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    global_position_int = (mavlink_global_position_int_t)Marshal.PtrToStructure(i, ((object)global_position_int).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
