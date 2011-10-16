// MESSAGE GPS_RAW_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 25;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_gps_raw_int_t
    {
         public  UInt64 usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     public  byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
     public  Int32 lat; /// Latitude in 1E7 degrees
     public  Int32 lon; /// Longitude in 1E7 degrees
     public  Int32 alt; /// Altitude in 1E3 meters (millimeters)
     public  Single eph; /// GPS HDOP
     public  Single epv; /// GPS VDOP
     public  Single v; /// GPS ground speed (m/s)
     public  Single hdg; /// Compass heading in degrees, 0..360 degrees
    
    };

/**
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed (m/s)
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_gps_raw_int_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 usec, byte fix_type, Int32 lat, Int32 lon, Int32 alt, Single eph, Single epv, Single v, Single hdg)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(fix_type),0,msg,8,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,9,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,13,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,17,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(eph),0,msg,21,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(epv),0,msg,25,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(v),0,msg,29,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(hdg),0,msg,33,sizeof(Single));

} else {
    mavlink_gps_raw_int_t packet = new mavlink_gps_raw_int_t();
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

        
        int len = 37;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
    //return mavlink_finalize_message(msg, system_id, component_id, 37);
    return 0;
}

/**
 * @brief Pack a gps_raw_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed (m/s)
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_gps_raw_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,byte public fix_type,Int32 public lat,Int32 public lon,Int32 public alt,Single public eph,Single public epv,Single public v,Single public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[37];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_byte(buf, 8, fix_type);
	_mav_put_Int32(buf, 9, lat);
	_mav_put_Int32(buf, 13, lon);
	_mav_put_Int32(buf, 17, alt);
	_mav_put_Single(buf, 21, eph);
	_mav_put_Single(buf, 25, epv);
	_mav_put_Single(buf, 29, v);
	_mav_put_Single(buf, 33, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 37);
#else
    mavlink_gps_raw_int_t packet;
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

        memcpy(_MAV_PAYLOAD(msg), &packet, 37);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 37);
}
*/
/**
 * @brief Encode a gps_raw_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw_int C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_gps_raw_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_raw_int_t* gps_raw_int)
{
    return mavlink_msg_gps_raw_int_pack(system_id, component_id, msg, gps_raw_int->usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->v, gps_raw_int->hdg);
}
*/
/**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters)
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed (m/s)
 * @param hdg Compass heading in degrees, 0..360 degrees
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_int_send(mavlink_channel_t chan, UInt64 public usec, byte public fix_type, Int32 public lat, Int32 public lon, Int32 public alt, Single public eph, Single public epv, Single public v, Single public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[37];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_byte(buf, 8, fix_type);
	_mav_put_Int32(buf, 9, lat);
	_mav_put_Int32(buf, 13, lon);
	_mav_put_Int32(buf, 17, alt);
	_mav_put_Single(buf, 21, eph);
	_mav_put_Single(buf, 25, epv);
	_mav_put_Single(buf, 29, v);
	_mav_put_Single(buf, 33, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, buf, 37);
#else
    mavlink_gps_raw_int_t packet;
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, (const char *)&packet, 37);
#endif
}

#endif
*/
// MESSAGE GPS_RAW_INT UNPACKING


/**
 * @brief Get field usec from gps_raw_int message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_gps_raw_int_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field fix_type from gps_raw_int message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
public static byte mavlink_msg_gps_raw_int_get_fix_type(byte[] msg)
{
    return getByte(msg,  8);
}

/**
 * @brief Get field lat from gps_raw_int message
 *
 * @return Latitude in 1E7 degrees
 */
public static Int32 mavlink_msg_gps_raw_int_get_lat(byte[] msg)
{
    return BitConverter.ToInt32(msg,  9);
}

/**
 * @brief Get field lon from gps_raw_int message
 *
 * @return Longitude in 1E7 degrees
 */
public static Int32 mavlink_msg_gps_raw_int_get_lon(byte[] msg)
{
    return BitConverter.ToInt32(msg,  13);
}

/**
 * @brief Get field alt from gps_raw_int message
 *
 * @return Altitude in 1E3 meters (millimeters)
 */
public static Int32 mavlink_msg_gps_raw_int_get_alt(byte[] msg)
{
    return BitConverter.ToInt32(msg,  17);
}

/**
 * @brief Get field eph from gps_raw_int message
 *
 * @return GPS HDOP
 */
public static Single mavlink_msg_gps_raw_int_get_eph(byte[] msg)
{
    return BitConverter.ToSingle(msg,  21);
}

/**
 * @brief Get field epv from gps_raw_int message
 *
 * @return GPS VDOP
 */
public static Single mavlink_msg_gps_raw_int_get_epv(byte[] msg)
{
    return BitConverter.ToSingle(msg,  25);
}

/**
 * @brief Get field v from gps_raw_int message
 *
 * @return GPS ground speed (m/s)
 */
public static Single mavlink_msg_gps_raw_int_get_v(byte[] msg)
{
    return BitConverter.ToSingle(msg,  29);
}

/**
 * @brief Get field hdg from gps_raw_int message
 *
 * @return Compass heading in degrees, 0..360 degrees
 */
public static Single mavlink_msg_gps_raw_int_get_hdg(byte[] msg)
{
    return BitConverter.ToSingle(msg,  33);
}

/**
 * @brief Decode a gps_raw_int message into a struct
 *
 * @param msg The message to decode
 * @param gps_raw_int C-struct to decode the message contents into
 */
public static void mavlink_msg_gps_raw_int_decode(byte[] msg, ref mavlink_gps_raw_int_t gps_raw_int)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	gps_raw_int.usec = mavlink_msg_gps_raw_int_get_usec(msg);
    	gps_raw_int.fix_type = mavlink_msg_gps_raw_int_get_fix_type(msg);
    	gps_raw_int.lat = mavlink_msg_gps_raw_int_get_lat(msg);
    	gps_raw_int.lon = mavlink_msg_gps_raw_int_get_lon(msg);
    	gps_raw_int.alt = mavlink_msg_gps_raw_int_get_alt(msg);
    	gps_raw_int.eph = mavlink_msg_gps_raw_int_get_eph(msg);
    	gps_raw_int.epv = mavlink_msg_gps_raw_int_get_epv(msg);
    	gps_raw_int.v = mavlink_msg_gps_raw_int_get_v(msg);
    	gps_raw_int.hdg = mavlink_msg_gps_raw_int_get_hdg(msg);
    
    } else {
        int len = 37; //Marshal.SizeOf(gps_raw_int);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        gps_raw_int = (mavlink_gps_raw_int_t)Marshal.PtrToStructure(i, ((object)gps_raw_int).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
