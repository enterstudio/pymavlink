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
     public  byte coordinate_frame; /// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
    
    };

/**
 * @brief Pack a global_position_setpoint_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param latitude WGS84 Latitude position in degrees * 1E7
 * @param longitude WGS84 Longitude position in degrees * 1E7
 * @param altitude WGS84 Altitude in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_global_position_setpoint_int_pack(byte system_id, byte component_id, byte[] msg,
                               byte coordinate_frame, Int32 latitude, Int32 longitude, Int32 altitude, Int16 yaw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(latitude),0,msg,0,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(longitude),0,msg,4,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(altitude),0,msg,8,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(coordinate_frame),0,msg,14,sizeof(byte));

} else {
    mavlink_global_position_setpoint_int_t packet = new mavlink_global_position_setpoint_int_t();
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;
	packet.coordinate_frame = coordinate_frame;

        
        int len = 15;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT;
    //return mavlink_finalize_message(msg, system_id, component_id, 15, 141);
    return 0;
}

/**
 * @brief Pack a global_position_setpoint_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param latitude WGS84 Latitude position in degrees * 1E7
 * @param longitude WGS84 Longitude position in degrees * 1E7
 * @param altitude WGS84 Altitude in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_global_position_setpoint_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public coordinate_frame,Int32 public latitude,Int32 public longitude,Int32 public altitude,Int16 public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[15];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_Int16(buf, 12, yaw);
	_mav_put_byte(buf, 14, coordinate_frame);

        memcpy(_MAV_PAYLOAD(msg), buf, 15);
#else
    mavlink_global_position_setpoint_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD(msg), &packet, 15);
#endif

    msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 15, 141);
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
    return mavlink_msg_global_position_setpoint_int_pack(system_id, component_id, msg, global_position_setpoint_int->coordinate_frame, global_position_setpoint_int->latitude, global_position_setpoint_int->longitude, global_position_setpoint_int->altitude, global_position_setpoint_int->yaw);
}
*/
/**
 * @brief Send a global_position_setpoint_int message
 * @param chan MAVLink channel to send the message
 *
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param latitude WGS84 Latitude position in degrees * 1E7
 * @param longitude WGS84 Longitude position in degrees * 1E7
 * @param altitude WGS84 Altitude in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_setpoint_int_send(mavlink_channel_t chan, byte public coordinate_frame, Int32 public latitude, Int32 public longitude, Int32 public altitude, Int16 public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[15];
	_mav_put_Int32(buf, 0, latitude);
	_mav_put_Int32(buf, 4, longitude);
	_mav_put_Int32(buf, 8, altitude);
	_mav_put_Int16(buf, 12, yaw);
	_mav_put_byte(buf, 14, coordinate_frame);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT, buf, 15, 141);
#else
    mavlink_global_position_setpoint_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;
	packet.coordinate_frame = coordinate_frame;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT, (const char *)&packet, 15, 141);
#endif
}

#endif
*/
// MESSAGE GLOBAL_POSITION_SETPOINT_INT UNPACKING


/**
 * @brief Get field coordinate_frame from global_position_setpoint_int message
 *
 * @return Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 */
public static byte mavlink_msg_global_position_setpoint_int_get_coordinate_frame(byte[] msg)
{
    return getByte(msg,  14);
}

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
    	global_position_setpoint_int.coordinate_frame = mavlink_msg_global_position_setpoint_int_get_coordinate_frame(msg);
    
    } else {
        int len = 15; //Marshal.SizeOf(global_position_setpoint_int);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        global_position_setpoint_int = (mavlink_global_position_setpoint_int_t)Marshal.PtrToStructure(i, ((object)global_position_setpoint_int).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
