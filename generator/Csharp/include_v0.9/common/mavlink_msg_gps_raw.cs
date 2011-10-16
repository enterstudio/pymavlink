// MESSAGE GPS_RAW PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GPS_RAW = 32;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_gps_raw_t
    {
         public  UInt64 usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     public  byte fix_type; /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
     public  Single lat; /// Latitude in degrees
     public  Single lon; /// Longitude in degrees
     public  Single alt; /// Altitude in meters
     public  Single eph; /// GPS HDOP
     public  Single epv; /// GPS VDOP
     public  Single v; /// GPS ground speed
     public  Single hdg; /// Compass heading in degrees, 0..360 degrees
    
    };

/**
 * @brief Pack a gps_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_gps_raw_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 usec, byte fix_type, Single lat, Single lon, Single alt, Single eph, Single epv, Single v, Single hdg)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(fix_type),0,msg,8,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,9,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,13,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,17,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(eph),0,msg,21,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(epv),0,msg,25,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(v),0,msg,29,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(hdg),0,msg,33,sizeof(Single));

} else {
    mavlink_gps_raw_t packet = new mavlink_gps_raw_t();
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

    //msg.msgid = MAVLINK_MSG_ID_GPS_RAW;
    //return mavlink_finalize_message(msg, system_id, component_id, 37);
    return 0;
}

/**
 * @brief Pack a gps_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed
 * @param hdg Compass heading in degrees, 0..360 degrees
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_gps_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,byte public fix_type,Single public lat,Single public lon,Single public alt,Single public eph,Single public epv,Single public v,Single public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[37];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_byte(buf, 8, fix_type);
	_mav_put_Single(buf, 9, lat);
	_mav_put_Single(buf, 13, lon);
	_mav_put_Single(buf, 17, alt);
	_mav_put_Single(buf, 21, eph);
	_mav_put_Single(buf, 25, epv);
	_mav_put_Single(buf, 29, v);
	_mav_put_Single(buf, 33, hdg);

        memcpy(_MAV_PAYLOAD(msg), buf, 37);
#else
    mavlink_gps_raw_t packet;
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

    msg->msgid = MAVLINK_MSG_ID_GPS_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 37);
}
*/
/**
 * @brief Encode a gps_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_raw C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_gps_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_raw_t* gps_raw)
{
    return mavlink_msg_gps_raw_pack(system_id, component_id, msg, gps_raw->usec, gps_raw->fix_type, gps_raw->lat, gps_raw->lon, gps_raw->alt, gps_raw->eph, gps_raw->epv, gps_raw->v, gps_raw->hdg);
}
*/
/**
 * @brief Send a gps_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in degrees
 * @param lon Longitude in degrees
 * @param alt Altitude in meters
 * @param eph GPS HDOP
 * @param epv GPS VDOP
 * @param v GPS ground speed
 * @param hdg Compass heading in degrees, 0..360 degrees
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_send(mavlink_channel_t chan, UInt64 public usec, byte public fix_type, Single public lat, Single public lon, Single public alt, Single public eph, Single public epv, Single public v, Single public hdg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[37];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_byte(buf, 8, fix_type);
	_mav_put_Single(buf, 9, lat);
	_mav_put_Single(buf, 13, lon);
	_mav_put_Single(buf, 17, alt);
	_mav_put_Single(buf, 21, eph);
	_mav_put_Single(buf, 25, epv);
	_mav_put_Single(buf, 29, v);
	_mav_put_Single(buf, 33, hdg);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW, buf, 37);
#else
    mavlink_gps_raw_t packet;
	packet.usec = usec;
	packet.fix_type = fix_type;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.v = v;
	packet.hdg = hdg;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW, (const char *)&packet, 37);
#endif
}

#endif
*/
// MESSAGE GPS_RAW UNPACKING


/**
 * @brief Get field usec from gps_raw message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_gps_raw_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field fix_type from gps_raw message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
public static byte mavlink_msg_gps_raw_get_fix_type(byte[] msg)
{
    return getByte(msg,  8);
}

/**
 * @brief Get field lat from gps_raw message
 *
 * @return Latitude in degrees
 */
public static Single mavlink_msg_gps_raw_get_lat(byte[] msg)
{
    return BitConverter.ToSingle(msg,  9);
}

/**
 * @brief Get field lon from gps_raw message
 *
 * @return Longitude in degrees
 */
public static Single mavlink_msg_gps_raw_get_lon(byte[] msg)
{
    return BitConverter.ToSingle(msg,  13);
}

/**
 * @brief Get field alt from gps_raw message
 *
 * @return Altitude in meters
 */
public static Single mavlink_msg_gps_raw_get_alt(byte[] msg)
{
    return BitConverter.ToSingle(msg,  17);
}

/**
 * @brief Get field eph from gps_raw message
 *
 * @return GPS HDOP
 */
public static Single mavlink_msg_gps_raw_get_eph(byte[] msg)
{
    return BitConverter.ToSingle(msg,  21);
}

/**
 * @brief Get field epv from gps_raw message
 *
 * @return GPS VDOP
 */
public static Single mavlink_msg_gps_raw_get_epv(byte[] msg)
{
    return BitConverter.ToSingle(msg,  25);
}

/**
 * @brief Get field v from gps_raw message
 *
 * @return GPS ground speed
 */
public static Single mavlink_msg_gps_raw_get_v(byte[] msg)
{
    return BitConverter.ToSingle(msg,  29);
}

/**
 * @brief Get field hdg from gps_raw message
 *
 * @return Compass heading in degrees, 0..360 degrees
 */
public static Single mavlink_msg_gps_raw_get_hdg(byte[] msg)
{
    return BitConverter.ToSingle(msg,  33);
}

/**
 * @brief Decode a gps_raw message into a struct
 *
 * @param msg The message to decode
 * @param gps_raw C-struct to decode the message contents into
 */
public static void mavlink_msg_gps_raw_decode(byte[] msg, ref mavlink_gps_raw_t gps_raw)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	gps_raw.usec = mavlink_msg_gps_raw_get_usec(msg);
    	gps_raw.fix_type = mavlink_msg_gps_raw_get_fix_type(msg);
    	gps_raw.lat = mavlink_msg_gps_raw_get_lat(msg);
    	gps_raw.lon = mavlink_msg_gps_raw_get_lon(msg);
    	gps_raw.alt = mavlink_msg_gps_raw_get_alt(msg);
    	gps_raw.eph = mavlink_msg_gps_raw_get_eph(msg);
    	gps_raw.epv = mavlink_msg_gps_raw_get_epv(msg);
    	gps_raw.v = mavlink_msg_gps_raw_get_v(msg);
    	gps_raw.hdg = mavlink_msg_gps_raw_get_hdg(msg);
    
    } else {
        int len = 37; //Marshal.SizeOf(gps_raw);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        gps_raw = (mavlink_gps_raw_t)Marshal.PtrToStructure(i, ((object)gps_raw).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
