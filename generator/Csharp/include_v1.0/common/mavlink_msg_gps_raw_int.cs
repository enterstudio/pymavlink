// MESSAGE GPS_RAW_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GPS_RAW_INT = 24;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_gps_raw_int_t
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public  UInt64 time_usec;
            /// <summary>
        /// Latitude in 1E7 degrees
        /// </summary>
        public  Int32 lat;
            /// <summary>
        /// Longitude in 1E7 degrees
        /// </summary>
        public  Int32 lon;
            /// <summary>
        /// Altitude in 1E3 meters (millimeters) above MSL
        /// </summary>
        public  Int32 alt;
            /// <summary>
        /// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        /// </summary>
        public  UInt16 eph;
            /// <summary>
        /// GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
        /// </summary>
        public  UInt16 epv;
            /// <summary>
        /// GPS ground speed (m/s * 100). If unknown, set to: 65535
        /// </summary>
        public  UInt16 vel;
            /// <summary>
        /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
        /// </summary>
        public  UInt16 cog;
            /// <summary>
        /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
        /// </summary>
        public  byte fix_type;
            /// <summary>
        /// Number of satellites visible. If unknown, set to 255
        /// </summary>
        public  byte satellites_visible;
    
    };

/// <summary>
/// * @brief Pack a gps_raw_int message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
/// * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
/// * @param lat Latitude in 1E7 degrees
/// * @param lon Longitude in 1E7 degrees
/// * @param alt Altitude in 1E3 meters (millimeters) above MSL
/// * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
/// * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
/// * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
/// * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
/// * @param satellites_visible Number of satellites visible. If unknown, set to 255
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_gps_raw_int_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec, byte fix_type, Int32 lat, Int32 lon, Int32 alt, UInt16 eph, UInt16 epv, UInt16 vel, UInt16 cog, byte satellites_visible)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,8,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,12,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,16,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(eph),0,msg,20,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(epv),0,msg,22,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(vel),0,msg,24,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(cog),0,msg,26,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(fix_type),0,msg,28,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(satellites_visible),0,msg,29,sizeof(byte));

} else {
    mavlink_gps_raw_int_t packet = new mavlink_gps_raw_int_t();
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.vel = vel;
	packet.cog = cog;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        
        int len = 30;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
    //return mavlink_finalize_message(msg, system_id, component_id, 30, 24);
    return 0;
}

/**
 * @brief Pack a gps_raw_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters) above MSL
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_gps_raw_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec,byte public fix_type,Int32 public lat,Int32 public lon,Int32 public alt,UInt16 public eph,UInt16 public epv,UInt16 public vel,UInt16 public cog,byte public satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[30];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Int32(buf, 8, lat);
	_mav_put_Int32(buf, 12, lon);
	_mav_put_Int32(buf, 16, alt);
	_mav_put_UInt16(buf, 20, eph);
	_mav_put_UInt16(buf, 22, epv);
	_mav_put_UInt16(buf, 24, vel);
	_mav_put_UInt16(buf, 26, cog);
	_mav_put_byte(buf, 28, fix_type);
	_mav_put_byte(buf, 29, satellites_visible);

        memcpy(_MAV_PAYLOAD(msg), buf, 30);
#else
    mavlink_gps_raw_int_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.vel = vel;
	packet.cog = cog;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD(msg), &packet, 30);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_RAW_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 30, 24);
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
    return mavlink_msg_gps_raw_int_pack(system_id, component_id, msg, gps_raw_int->time_usec, gps_raw_int->fix_type, gps_raw_int->lat, gps_raw_int->lon, gps_raw_int->alt, gps_raw_int->eph, gps_raw_int->epv, gps_raw_int->vel, gps_raw_int->cog, gps_raw_int->satellites_visible);
}
*/
/**
 * @brief Send a gps_raw_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude in 1E7 degrees
 * @param lon Longitude in 1E7 degrees
 * @param alt Altitude in 1E3 meters (millimeters) above MSL
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param epv GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: 65535
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_raw_int_send(mavlink_channel_t chan, UInt64 public time_usec, byte public fix_type, Int32 public lat, Int32 public lon, Int32 public alt, UInt16 public eph, UInt16 public epv, UInt16 public vel, UInt16 public cog, byte public satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[30];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Int32(buf, 8, lat);
	_mav_put_Int32(buf, 12, lon);
	_mav_put_Int32(buf, 16, alt);
	_mav_put_UInt16(buf, 20, eph);
	_mav_put_UInt16(buf, 22, epv);
	_mav_put_UInt16(buf, 24, vel);
	_mav_put_UInt16(buf, 26, cog);
	_mav_put_byte(buf, 28, fix_type);
	_mav_put_byte(buf, 29, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, buf, 30, 24);
#else
    mavlink_gps_raw_int_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.eph = eph;
	packet.epv = epv;
	packet.vel = vel;
	packet.cog = cog;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_RAW_INT, (const char *)&packet, 30, 24);
#endif
}

#endif
*/
// MESSAGE GPS_RAW_INT UNPACKING


/**
 * @brief Get field time_usec from gps_raw_int message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_gps_raw_int_get_time_usec(byte[] msg)
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
    return getByte(msg,  28);
}

/**
 * @brief Get field lat from gps_raw_int message
 *
 * @return Latitude in 1E7 degrees
 */
public static Int32 mavlink_msg_gps_raw_int_get_lat(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Get field lon from gps_raw_int message
 *
 * @return Longitude in 1E7 degrees
 */
public static Int32 mavlink_msg_gps_raw_int_get_lon(byte[] msg)
{
    return BitConverter.ToInt32(msg,  12);
}

/**
 * @brief Get field alt from gps_raw_int message
 *
 * @return Altitude in 1E3 meters (millimeters) above MSL
 */
public static Int32 mavlink_msg_gps_raw_int_get_alt(byte[] msg)
{
    return BitConverter.ToInt32(msg,  16);
}

/**
 * @brief Get field eph from gps_raw_int message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
public static UInt16 mavlink_msg_gps_raw_int_get_eph(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  20);
}

/**
 * @brief Get field epv from gps_raw_int message
 *
 * @return GPS VDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
public static UInt16 mavlink_msg_gps_raw_int_get_epv(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  22);
}

/**
 * @brief Get field vel from gps_raw_int message
 *
 * @return GPS ground speed (m/s * 100). If unknown, set to: 65535
 */
public static UInt16 mavlink_msg_gps_raw_int_get_vel(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  24);
}

/**
 * @brief Get field cog from gps_raw_int message
 *
 * @return Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
 */
public static UInt16 mavlink_msg_gps_raw_int_get_cog(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  26);
}

/**
 * @brief Get field satellites_visible from gps_raw_int message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
public static byte mavlink_msg_gps_raw_int_get_satellites_visible(byte[] msg)
{
    return getByte(msg,  29);
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
    	gps_raw_int.time_usec = mavlink_msg_gps_raw_int_get_time_usec(msg);
    	gps_raw_int.lat = mavlink_msg_gps_raw_int_get_lat(msg);
    	gps_raw_int.lon = mavlink_msg_gps_raw_int_get_lon(msg);
    	gps_raw_int.alt = mavlink_msg_gps_raw_int_get_alt(msg);
    	gps_raw_int.eph = mavlink_msg_gps_raw_int_get_eph(msg);
    	gps_raw_int.epv = mavlink_msg_gps_raw_int_get_epv(msg);
    	gps_raw_int.vel = mavlink_msg_gps_raw_int_get_vel(msg);
    	gps_raw_int.cog = mavlink_msg_gps_raw_int_get_cog(msg);
    	gps_raw_int.fix_type = mavlink_msg_gps_raw_int_get_fix_type(msg);
    	gps_raw_int.satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(msg);
    
    } else {
        int len = 30; //Marshal.SizeOf(gps_raw_int);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        gps_raw_int = (mavlink_gps_raw_int_t)Marshal.PtrToStructure(i, ((object)gps_raw_int).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
