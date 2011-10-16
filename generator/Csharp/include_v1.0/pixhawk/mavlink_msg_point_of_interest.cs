// MESSAGE POINT_OF_INTEREST PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POINT_OF_INTEREST = 191;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_point_of_interest_t
    {
         public  Single x; /// X Position
     public  Single y; /// Y Position
     public  Single z; /// Z Position
     public  UInt16 timeout; /// 0: no timeout, >1: timeout in seconds
     public  byte type; /// 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
     public  byte color; /// 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
     public  byte coordinate_system; /// 0: global, 1:local
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 public string name; /// POI name
    
    };

/**
 * @brief Pack a point_of_interest message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_point_of_interest_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public type, byte public color, byte public coordinate_system, UInt16 public timeout, Single public x, Single public y, Single public z, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 publicname)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[43];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_UInt16(buf, 12, timeout);
	_mav_put_byte(buf, 14, type);
	_mav_put_byte(buf, 15, color);
	_mav_put_byte(buf, 16, coordinate_system);
	_mav_put_string_array(buf, 17, name, 26);
        memcpy(_MAV_PAYLOAD(msg), buf, 43);
#else
    mavlink_point_of_interest_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.timeout = timeout;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	memcpy(packet.name, name, sizeof(string)*26);
        memcpy(_MAV_PAYLOAD(msg), &packet, 43);
#endif

    msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;
    return mavlink_finalize_message(msg, system_id, component_id, 43, 95);
}
*/
/**
 * @brief Pack a point_of_interest message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_point_of_interest_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,byte public color,byte public coordinate_system,UInt16 public timeout,Single public x,Single public y,Single public z,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 publicname)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[43];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_UInt16(buf, 12, timeout);
	_mav_put_byte(buf, 14, type);
	_mav_put_byte(buf, 15, color);
	_mav_put_byte(buf, 16, coordinate_system);
	_mav_put_string_array(buf, 17, name, 26);
        memcpy(_MAV_PAYLOAD(msg), buf, 43);
#else
    mavlink_point_of_interest_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.timeout = timeout;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	memcpy(packet.name, name, sizeof(string)*26);
        memcpy(_MAV_PAYLOAD(msg), &packet, 43);
#endif

    msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 43, 95);
}
*/
/**
 * @brief Encode a point_of_interest struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param point_of_interest C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_point_of_interest_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_point_of_interest_t* point_of_interest)
{
    return mavlink_msg_point_of_interest_pack(system_id, component_id, msg, point_of_interest->type, point_of_interest->color, point_of_interest->coordinate_system, point_of_interest->timeout, point_of_interest->x, point_of_interest->y, point_of_interest->z, point_of_interest->name);
}
*/
/**
 * @brief Send a point_of_interest message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param name POI name
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_point_of_interest_send(mavlink_channel_t chan, byte public type, byte public color, byte public coordinate_system, UInt16 public timeout, Single public x, Single public y, Single public z, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 publicname)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[43];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_UInt16(buf, 12, timeout);
	_mav_put_byte(buf, 14, type);
	_mav_put_byte(buf, 15, color);
	_mav_put_byte(buf, 16, coordinate_system);
	_mav_put_string_array(buf, 17, name, 26);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST, buf, 43, 95);
#else
    mavlink_point_of_interest_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.timeout = timeout;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	memcpy(packet.name, name, sizeof(string)*26);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST, (const char *)&packet, 43, 95);
#endif
}

#endif
*/
// MESSAGE POINT_OF_INTEREST UNPACKING


/**
 * @brief Get field type from point_of_interest message
 *
 * @return 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 */
public static byte mavlink_msg_point_of_interest_get_type(byte[] msg)
{
    return getByte(msg,  14);
}

/**
 * @brief Get field color from point_of_interest message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
public static byte mavlink_msg_point_of_interest_get_color(byte[] msg)
{
    return getByte(msg,  15);
}

/**
 * @brief Get field coordinate_system from point_of_interest message
 *
 * @return 0: global, 1:local
 */
public static byte mavlink_msg_point_of_interest_get_coordinate_system(byte[] msg)
{
    return getByte(msg,  16);
}

/**
 * @brief Get field timeout from point_of_interest message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
public static UInt16 mavlink_msg_point_of_interest_get_timeout(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field x from point_of_interest message
 *
 * @return X Position
 */
public static Single mavlink_msg_point_of_interest_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from point_of_interest message
 *
 * @return Y Position
 */
public static Single mavlink_msg_point_of_interest_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from point_of_interest message
 *
 * @return Z Position
 */
public static Single mavlink_msg_point_of_interest_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field name from point_of_interest message
 *
 * @return POI name
 */
public static string mavlink_msg_point_of_interest_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,17,26); //(msg, 26,  17);
}

/**
 * @brief Decode a point_of_interest message into a struct
 *
 * @param msg The message to decode
 * @param point_of_interest C-struct to decode the message contents into
 */
public static void mavlink_msg_point_of_interest_decode(byte[] msg, ref mavlink_point_of_interest_t point_of_interest)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	point_of_interest.x = mavlink_msg_point_of_interest_get_x(msg);
	point_of_interest.y = mavlink_msg_point_of_interest_get_y(msg);
	point_of_interest.z = mavlink_msg_point_of_interest_get_z(msg);
	point_of_interest.timeout = mavlink_msg_point_of_interest_get_timeout(msg);
	point_of_interest.type = mavlink_msg_point_of_interest_get_type(msg);
	point_of_interest.color = mavlink_msg_point_of_interest_get_color(msg);
	point_of_interest.coordinate_system = mavlink_msg_point_of_interest_get_coordinate_system(msg);
	point_of_interest.name = mavlink_msg_point_of_interest_get_name(msg);
} else {
    int len = 43; //Marshal.SizeOf(point_of_interest);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    point_of_interest = (mavlink_point_of_interest_t)Marshal.PtrToStructure(i, ((object)point_of_interest).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
