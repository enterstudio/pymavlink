// MESSAGE GLOBAL_POSITION_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 34;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_global_position_int_t
    {
         public  UInt32 time_boot_ms; /// Timestamp (milliseconds since system boot)
     public  Int32 lat; /// Latitude, expressed as * 1E7
     public  Int32 lon; /// Longitude, expressed as * 1E7
     public  Int32 alt; /// Altitude in meters, expressed as * 1000 (millimeters), above MSL
     public  Int32 relative_alt; /// Altitude above ground in meters, expressed as * 1000 (millimeters)
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
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_global_position_int_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Int32 lat, Int32 lon, Int32 alt, Int32 relative_alt, Int16 vx, Int16 vy, Int16 vz, UInt16 hdg)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,4,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,8,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,12,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(relative_alt),0,msg,16,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(vx),0,msg,20,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(vy),0,msg,22,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(vz),0,msg,24,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(hdg),0,msg,26,sizeof(UInt16));

} else {
    mavlink_global_position_int_t packet = new mavlink_global_position_int_t();
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        
        int len = 28;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    //return mavlink_finalize_message(msg, system_id, component_id, 28, 104);
    return 0;
}

/**
 * @brief Pack a global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,Int32 public lat,Int32 public lon,Int32 public alt,Int32 public relative_alt,Int16 public vx,Int16 public vy,Int16 public vz,UInt16 public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, lat);
	_mav_put_Int32(buf, 8, lon);
	_mav_put_Int32(buf, 12, alt);
	_mav_put_Int32(buf, 16, relative_alt);
	_mav_put_Int16(buf, 20, vx);
	_mav_put_Int16(buf, 22, vy);
	_mav_put_Int16(buf, 24, vz);
	_mav_put_UInt16(buf, 26, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
    mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 104);
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
    return mavlink_msg_global_position_int_pack(system_id, component_id, msg, global_position_int->time_boot_ms, global_position_int->lat, global_position_int->lon, global_position_int->alt, global_position_int->relative_alt, global_position_int->vx, global_position_int->vy, global_position_int->vz, global_position_int->hdg);
}
*/
/**
 * @brief Send a global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_int_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Int32 public lat, Int32 public lon, Int32 public alt, Int32 public relative_alt, Int16 public vx, Int16 public vy, Int16 public vz, UInt16 public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, lat);
	_mav_put_Int32(buf, 8, lon);
	_mav_put_Int32(buf, 12, alt);
	_mav_put_Int32(buf, 16, relative_alt);
	_mav_put_Int16(buf, 20, vx);
	_mav_put_Int16(buf, 22, vy);
	_mav_put_Int16(buf, 24, vz);
	_mav_put_UInt16(buf, 26, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, buf, 28, 104);
#else
    mavlink_global_position_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;
	packet.hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, (const char *)&packet, 28, 104);
#endif
}

#endif
*/
// MESSAGE GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field time_boot_ms from global_position_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_global_position_int_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field lat from global_position_int message
 *
 * @return Latitude, expressed as * 1E7
 */
public static Int32 mavlink_msg_global_position_int_get_lat(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
}

/**
 * @brief Get field lon from global_position_int message
 *
 * @return Longitude, expressed as * 1E7
 */
public static Int32 mavlink_msg_global_position_int_get_lon(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Get field alt from global_position_int message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
public static Int32 mavlink_msg_global_position_int_get_alt(byte[] msg)
{
    return BitConverter.ToInt32(msg,  12);
}

/**
 * @brief Get field relative_alt from global_position_int message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
public static Int32 mavlink_msg_global_position_int_get_relative_alt(byte[] msg)
{
    return BitConverter.ToInt32(msg,  16);
}

/**
 * @brief Get field vx from global_position_int message
 *
 * @return Ground X Speed (Latitude), expressed as m/s * 100
 */
public static Int16 mavlink_msg_global_position_int_get_vx(byte[] msg)
{
    return BitConverter.ToInt16(msg,  20);
}

/**
 * @brief Get field vy from global_position_int message
 *
 * @return Ground Y Speed (Longitude), expressed as m/s * 100
 */
public static Int16 mavlink_msg_global_position_int_get_vy(byte[] msg)
{
    return BitConverter.ToInt16(msg,  22);
}

/**
 * @brief Get field vz from global_position_int message
 *
 * @return Ground Z Speed (Altitude), expressed as m/s * 100
 */
public static Int16 mavlink_msg_global_position_int_get_vz(byte[] msg)
{
    return BitConverter.ToInt16(msg,  24);
}

/**
 * @brief Get field hdg from global_position_int message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
public static UInt16 mavlink_msg_global_position_int_get_hdg(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  26);
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
    	global_position_int.time_boot_ms = mavlink_msg_global_position_int_get_time_boot_ms(msg);
    	global_position_int.lat = mavlink_msg_global_position_int_get_lat(msg);
    	global_position_int.lon = mavlink_msg_global_position_int_get_lon(msg);
    	global_position_int.alt = mavlink_msg_global_position_int_get_alt(msg);
    	global_position_int.relative_alt = mavlink_msg_global_position_int_get_relative_alt(msg);
    	global_position_int.vx = mavlink_msg_global_position_int_get_vx(msg);
    	global_position_int.vy = mavlink_msg_global_position_int_get_vy(msg);
    	global_position_int.vz = mavlink_msg_global_position_int_get_vz(msg);
    	global_position_int.hdg = mavlink_msg_global_position_int_get_hdg(msg);
    
    } else {
        int len = 28; //Marshal.SizeOf(global_position_int);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        global_position_int = (mavlink_global_position_int_t)Marshal.PtrToStructure(i, ((object)global_position_int).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
