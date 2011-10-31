// MESSAGE GLOBAL_POSITION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GLOBAL_POSITION = 33;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_global_position_t
    {
        /// <summary>
        /// Timestamp (microseconds since unix epoch)
        /// </summary>
        public  UInt64 usec;
            /// <summary>
        /// Latitude, in degrees
        /// </summary>
        public  Single lat;
            /// <summary>
        /// Longitude, in degrees
        /// </summary>
        public  Single lon;
            /// <summary>
        /// Absolute altitude, in meters
        /// </summary>
        public  Single alt;
            /// <summary>
        /// X Speed (in Latitude direction, positive: going north)
        /// </summary>
        public  Single vx;
            /// <summary>
        /// Y Speed (in Longitude direction, positive: going east)
        /// </summary>
        public  Single vy;
            /// <summary>
        /// Z Speed (in Altitude direction, positive: going up)
        /// </summary>
        public  Single vz;
    
    };

/// <summary>
/// * @brief Pack a global_position message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param usec Timestamp (microseconds since unix epoch)
/// * @param lat Latitude, in degrees
/// * @param lon Longitude, in degrees
/// * @param alt Absolute altitude, in meters
/// * @param vx X Speed (in Latitude direction, positive: going north)
/// * @param vy Y Speed (in Longitude direction, positive: going east)
/// * @param vz Z Speed (in Altitude direction, positive: going up)
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_global_position_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 usec, Single lat, Single lon, Single alt, Single vx, Single vy, Single vz)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(lat),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(lon),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(alt),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vx),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vy),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vz),0,msg,28,sizeof(Single));

} else {
    mavlink_global_position_t packet = new mavlink_global_position_t();
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        
        int len = 32;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GLOBAL_POSITION;
    //return mavlink_finalize_message(msg, system_id, component_id, 32);
    return 0;
}

/**
 * @brief Pack a global_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since unix epoch)
 * @param lat Latitude, in degrees
 * @param lon Longitude, in degrees
 * @param alt Absolute altitude, in meters
 * @param vx X Speed (in Latitude direction, positive: going north)
 * @param vy Y Speed (in Longitude direction, positive: going east)
 * @param vz Z Speed (in Altitude direction, positive: going up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_global_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public lat,Single public lon,Single public alt,Single public vx,Single public vy,Single public vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, lat);
	_mav_put_Single(buf, 12, lon);
	_mav_put_Single(buf, 16, alt);
	_mav_put_Single(buf, 20, vx);
	_mav_put_Single(buf, 24, vy);
	_mav_put_Single(buf, 28, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_global_position_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}
*/
/**
 * @brief Encode a global_position struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_global_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_t* global_position)
{
    return mavlink_msg_global_position_pack(system_id, component_id, msg, global_position->usec, global_position->lat, global_position->lon, global_position->alt, global_position->vx, global_position->vy, global_position->vz);
}
*/
/**
 * @brief Send a global_position message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since unix epoch)
 * @param lat Latitude, in degrees
 * @param lon Longitude, in degrees
 * @param alt Absolute altitude, in meters
 * @param vx X Speed (in Latitude direction, positive: going north)
 * @param vy Y Speed (in Longitude direction, positive: going east)
 * @param vz Z Speed (in Altitude direction, positive: going up)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_send(mavlink_channel_t chan, UInt64 public usec, Single public lat, Single public lon, Single public alt, Single public vx, Single public vy, Single public vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, lat);
	_mav_put_Single(buf, 12, lon);
	_mav_put_Single(buf, 16, alt);
	_mav_put_Single(buf, 20, vx);
	_mav_put_Single(buf, 24, vy);
	_mav_put_Single(buf, 28, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION, buf, 32);
#else
    mavlink_global_position_t packet;
	packet.usec = usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION, (const char *)&packet, 32);
#endif
}

#endif
*/
// MESSAGE GLOBAL_POSITION UNPACKING


/**
 * @brief Get field usec from global_position message
 *
 * @return Timestamp (microseconds since unix epoch)
 */
public static UInt64 mavlink_msg_global_position_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field lat from global_position message
 *
 * @return Latitude, in degrees
 */
public static Single mavlink_msg_global_position_get_lat(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field lon from global_position message
 *
 * @return Longitude, in degrees
 */
public static Single mavlink_msg_global_position_get_lon(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field alt from global_position message
 *
 * @return Absolute altitude, in meters
 */
public static Single mavlink_msg_global_position_get_alt(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field vx from global_position message
 *
 * @return X Speed (in Latitude direction, positive: going north)
 */
public static Single mavlink_msg_global_position_get_vx(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field vy from global_position message
 *
 * @return Y Speed (in Longitude direction, positive: going east)
 */
public static Single mavlink_msg_global_position_get_vy(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field vz from global_position message
 *
 * @return Z Speed (in Altitude direction, positive: going up)
 */
public static Single mavlink_msg_global_position_get_vz(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a global_position message into a struct
 *
 * @param msg The message to decode
 * @param global_position C-struct to decode the message contents into
 */
public static void mavlink_msg_global_position_decode(byte[] msg, ref mavlink_global_position_t global_position)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	global_position.usec = mavlink_msg_global_position_get_usec(msg);
    	global_position.lat = mavlink_msg_global_position_get_lat(msg);
    	global_position.lon = mavlink_msg_global_position_get_lon(msg);
    	global_position.alt = mavlink_msg_global_position_get_alt(msg);
    	global_position.vx = mavlink_msg_global_position_get_vx(msg);
    	global_position.vy = mavlink_msg_global_position_get_vy(msg);
    	global_position.vz = mavlink_msg_global_position_get_vz(msg);
    
    } else {
        int len = 32; //Marshal.SizeOf(global_position);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        global_position = (mavlink_global_position_t)Marshal.PtrToStructure(i, ((object)global_position).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
