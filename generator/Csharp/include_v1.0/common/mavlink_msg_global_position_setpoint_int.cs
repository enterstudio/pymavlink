// MESSAGE GLOBAL_POSITION_SETPOINT_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT = 52;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_global_position_setpoint_int_t
    {
         public  Int32 latitude; /// WGS84 Latitude position in degrees * 1E7
     public  Int32 longitude; /// WGS84 Longitude position in degrees * 1E7
     public  Int32 altitude; /// WGS84 Altitude in meters * 1000 (positive for up)
     public  Int16 yaw; /// Desired yaw angle in degrees * 100
    
    };

/**
 * @brief Pack a global_position_setpoint_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude WGS84 Latitude position in degrees * 1E7
 * @param longitude WGS84 Longitude position in degrees * 1E7
 * @param altitude WGS84 Altitude in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_global_position_setpoint_int_pack(byte system_id, byte component_id, ref byte[] msg,
                               Int32 public latitude, Int32 public longitude, Int32 public altitude, Int16 public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[14];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_Int16(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_global_position_setpoint_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT;
    return mavlink_finalize_message(msg, system_id, component_id, 14, 142);
}
*/
/**
 * @brief Pack a global_position_setpoint_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param latitude WGS84 Latitude position in degrees * 1E7
 * @param longitude WGS84 Longitude position in degrees * 1E7
 * @param altitude WGS84 Altitude in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_global_position_setpoint_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Int32 public latitude,Int32 public longitude,Int32 public altitude,Int16 public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_Int16(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_global_position_setpoint_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 142);
}
*/
/**
 * @brief Encode a global_position_setpoint_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_setpoint_int C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_global_position_setpoint_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_setpoint_int_t* global_position_setpoint_int)
{
    return mavlink_msg_global_position_setpoint_int_pack(system_id, component_id, msg, global_position_setpoint_int->latitude, global_position_setpoint_int->longitude, global_position_setpoint_int->altitude, global_position_setpoint_int->yaw);
}
*/
/**
 * @brief Send a global_position_setpoint_int message
 * @param chan MAVLink channel to send the message
 *
 * @param latitude WGS84 Latitude position in degrees * 1E7
 * @param longitude WGS84 Longitude position in degrees * 1E7
 * @param altitude WGS84 Altitude in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_setpoint_int_send(mavlink_channel_t chan, Int32 public latitude, Int32 public longitude, Int32 public altitude, Int16 public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_Int16(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT, buf, 14, 142);
#else
    mavlink_global_position_setpoint_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT, (const char *)&packet, 14, 142);
#endif
}

#endif
*/
// MESSAGE GLOBAL_POSITION_SETPOINT_INT UNPACKING


/**
 * @brief Get field latitude from global_position_setpoint_int message
 *
 * @return WGS84 Latitude position in degrees * 1E7
 */
public static Int32 mavlink_msg_global_position_setpoint_int_get_latitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  0);
}

/**
 * @brief Get field longitude from global_position_setpoint_int message
 *
 * @return WGS84 Longitude position in degrees * 1E7
 */
public static Int32 mavlink_msg_global_position_setpoint_int_get_longitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
}

/**
 * @brief Get field altitude from global_position_setpoint_int message
 *
 * @return WGS84 Altitude in meters * 1000 (positive for up)
 */
public static Int32 mavlink_msg_global_position_setpoint_int_get_altitude(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Get field yaw from global_position_setpoint_int message
 *
 * @return Desired yaw angle in degrees * 100
 */
public static Int16 mavlink_msg_global_position_setpoint_int_get_yaw(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Decode a global_position_setpoint_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_setpoint_int C-struct to decode the message contents into
 */
public static void mavlink_msg_global_position_setpoint_int_decode(byte[] msg, ref mavlink_global_position_setpoint_int_t global_position_setpoint_int)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	global_position_setpoint_int.latitude = mavlink_msg_global_position_setpoint_int_get_latitude(msg);
	global_position_setpoint_int.longitude = mavlink_msg_global_position_setpoint_int_get_longitude(msg);
	global_position_setpoint_int.altitude = mavlink_msg_global_position_setpoint_int_get_altitude(msg);
	global_position_setpoint_int.yaw = mavlink_msg_global_position_setpoint_int_get_yaw(msg);
} else {
    int len = 14; //Marshal.SizeOf(global_position_setpoint_int);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    global_position_setpoint_int = (mavlink_global_position_setpoint_int_t)Marshal.PtrToStructure(i, ((object)global_position_setpoint_int).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
