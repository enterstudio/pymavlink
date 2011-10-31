// MESSAGE GPS_STATUS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GPS_STATUS = 25;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_gps_status_t
    {
        /// <summary>
        /// Number of satellites visible
        /// </summary>
        public  byte satellites_visible;
            /// <summary>
        /// Global satellite ID
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 public byte[] satellite_prn;
            /// <summary>
        /// 0: Satellite not used, 1: used for localization
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 public byte[] satellite_used;
            /// <summary>
        /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 public byte[] satellite_elevation;
            /// <summary>
        /// Direction of satellite, 0: 0 deg, 255: 360 deg.
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 public byte[] satellite_azimuth;
            /// <summary>
        /// Signal to noise ratio of satellite
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 public byte[] satellite_snr;
    
    };

/// <summary>
/// * @brief Pack a gps_status message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param satellites_visible Number of satellites visible
/// * @param satellite_prn Global satellite ID
/// * @param satellite_used 0: Satellite not used, 1: used for localization
/// * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
/// * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
/// * @param satellite_snr Signal to noise ratio of satellite
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_gps_status_pack(byte system_id, byte component_id, byte[] msg,
                               byte satellites_visible, byte[] satellite_prn, byte[] satellite_used, byte[] satellite_elevation, byte[] satellite_azimuth, byte[] satellite_snr)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(satellites_visible),0,msg,0,sizeof(byte));
	Array.Copy(toArray(satellite_prn),0,msg,1,20);
	Array.Copy(toArray(satellite_used),0,msg,21,20);
	Array.Copy(toArray(satellite_elevation),0,msg,41,20);
	Array.Copy(toArray(satellite_azimuth),0,msg,61,20);
	Array.Copy(toArray(satellite_snr),0,msg,81,20);
} else {
    mavlink_gps_status_t packet = new mavlink_gps_status_t();
	packet.satellites_visible = satellites_visible;
	packet.satellite_prn = satellite_prn;
	packet.satellite_used = satellite_used;
	packet.satellite_elevation = satellite_elevation;
	packet.satellite_azimuth = satellite_azimuth;
	packet.satellite_snr = satellite_snr;
        
        int len = 101;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GPS_STATUS;
    //return mavlink_finalize_message(msg, system_id, component_id, 101, 23);
    return 0;
}

/**
 * @brief Pack a gps_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_gps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public satellites_visible,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_prn,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_used,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_elevation,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_azimuth,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_snr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[101];
	_mav_put_byte(buf, 0, satellites_visible);
	_mav_put_byte[]_array(buf, 1, satellite_prn, 20);
	_mav_put_byte[]_array(buf, 21, satellite_used, 20);
	_mav_put_byte[]_array(buf, 41, satellite_elevation, 20);
	_mav_put_byte[]_array(buf, 61, satellite_azimuth, 20);
	_mav_put_byte[]_array(buf, 81, satellite_snr, 20);
        memcpy(_MAV_PAYLOAD(msg), buf, 101);
#else
    mavlink_gps_status_t packet;
	packet.satellites_visible = satellites_visible;
	memcpy(packet.satellite_prn, satellite_prn, sizeof(byte[])*20);
	memcpy(packet.satellite_used, satellite_used, sizeof(byte[])*20);
	memcpy(packet.satellite_elevation, satellite_elevation, sizeof(byte[])*20);
	memcpy(packet.satellite_azimuth, satellite_azimuth, sizeof(byte[])*20);
	memcpy(packet.satellite_snr, satellite_snr, sizeof(byte[])*20);
        memcpy(_MAV_PAYLOAD(msg), &packet, 101);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 101, 23);
}
*/
/**
 * @brief Encode a gps_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_status C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_gps_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_status_t* gps_status)
{
    return mavlink_msg_gps_status_pack(system_id, component_id, msg, gps_status->satellites_visible, gps_status->satellite_prn, gps_status->satellite_used, gps_status->satellite_elevation, gps_status->satellite_azimuth, gps_status->satellite_snr);
}
*/
/**
 * @brief Send a gps_status message
 * @param chan MAVLink channel to send the message
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_status_send(mavlink_channel_t chan, byte public satellites_visible, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_prn, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_used, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_elevation, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_azimuth, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=20)]
 publicsatellite_snr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[101];
	_mav_put_byte(buf, 0, satellites_visible);
	_mav_put_byte[]_array(buf, 1, satellite_prn, 20);
	_mav_put_byte[]_array(buf, 21, satellite_used, 20);
	_mav_put_byte[]_array(buf, 41, satellite_elevation, 20);
	_mav_put_byte[]_array(buf, 61, satellite_azimuth, 20);
	_mav_put_byte[]_array(buf, 81, satellite_snr, 20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_STATUS, buf, 101, 23);
#else
    mavlink_gps_status_t packet;
	packet.satellites_visible = satellites_visible;
	memcpy(packet.satellite_prn, satellite_prn, sizeof(byte[])*20);
	memcpy(packet.satellite_used, satellite_used, sizeof(byte[])*20);
	memcpy(packet.satellite_elevation, satellite_elevation, sizeof(byte[])*20);
	memcpy(packet.satellite_azimuth, satellite_azimuth, sizeof(byte[])*20);
	memcpy(packet.satellite_snr, satellite_snr, sizeof(byte[])*20);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_STATUS, (const char *)&packet, 101, 23);
#endif
}

#endif
*/
// MESSAGE GPS_STATUS UNPACKING


/**
 * @brief Get field satellites_visible from gps_status message
 *
 * @return Number of satellites visible
 */
public static byte mavlink_msg_gps_status_get_satellites_visible(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field satellite_prn from gps_status message
 *
 * @return Global satellite ID
 */
public static byte[] mavlink_msg_gps_status_get_satellite_prn(byte[] msg)
{
    return getBytes(msg, 20,  1);
}

/**
 * @brief Get field satellite_used from gps_status message
 *
 * @return 0: Satellite not used, 1: used for localization
 */
public static byte[] mavlink_msg_gps_status_get_satellite_used(byte[] msg)
{
    return getBytes(msg, 20,  21);
}

/**
 * @brief Get field satellite_elevation from gps_status message
 *
 * @return Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 */
public static byte[] mavlink_msg_gps_status_get_satellite_elevation(byte[] msg)
{
    return getBytes(msg, 20,  41);
}

/**
 * @brief Get field satellite_azimuth from gps_status message
 *
 * @return Direction of satellite, 0: 0 deg, 255: 360 deg.
 */
public static byte[] mavlink_msg_gps_status_get_satellite_azimuth(byte[] msg)
{
    return getBytes(msg, 20,  61);
}

/**
 * @brief Get field satellite_snr from gps_status message
 *
 * @return Signal to noise ratio of satellite
 */
public static byte[] mavlink_msg_gps_status_get_satellite_snr(byte[] msg)
{
    return getBytes(msg, 20,  81);
}

/**
 * @brief Decode a gps_status message into a struct
 *
 * @param msg The message to decode
 * @param gps_status C-struct to decode the message contents into
 */
public static void mavlink_msg_gps_status_decode(byte[] msg, ref mavlink_gps_status_t gps_status)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	gps_status.satellites_visible = mavlink_msg_gps_status_get_satellites_visible(msg);
    	gps_status.satellite_prn = mavlink_msg_gps_status_get_satellite_prn(msg);
    	gps_status.satellite_used = mavlink_msg_gps_status_get_satellite_used(msg);
    	gps_status.satellite_elevation = mavlink_msg_gps_status_get_satellite_elevation(msg);
    	gps_status.satellite_azimuth = mavlink_msg_gps_status_get_satellite_azimuth(msg);
    	gps_status.satellite_snr = mavlink_msg_gps_status_get_satellite_snr(msg);
    
    } else {
        int len = 101; //Marshal.SizeOf(gps_status);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        gps_status = (mavlink_gps_status_t)Marshal.PtrToStructure(i, ((object)gps_status).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
