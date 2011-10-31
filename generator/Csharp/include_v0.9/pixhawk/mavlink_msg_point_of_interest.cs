// MESSAGE POINT_OF_INTEREST PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POINT_OF_INTEREST = 191;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_point_of_interest_t
    {
        /// <summary>
        /// 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
        /// </summary>
        public  byte type;
            /// <summary>
        /// 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
        /// </summary>
        public  byte color;
            /// <summary>
        /// 0: global, 1:local
        /// </summary>
        public  byte coordinate_system;
            /// <summary>
        /// 0: no timeout, >1: timeout in seconds
        /// </summary>
        public  UInt16 timeout;
            /// <summary>
        /// X Position
        /// </summary>
        public  Single x;
            /// <summary>
        /// Y Position
        /// </summary>
        public  Single y;
            /// <summary>
        /// Z Position
        /// </summary>
        public  Single z;
            /// <summary>
        /// POI name
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 public string name;
    
    };

/// <summary>
/// * @brief Pack a point_of_interest message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
/// * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
/// * @param coordinate_system 0: global, 1:local
/// * @param timeout 0: no timeout, >1: timeout in seconds
/// * @param x X Position
/// * @param y Y Position
/// * @param z Z Position
/// * @param name POI name
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_point_of_interest_pack(byte system_id, byte component_id, byte[] msg,
                               byte type, byte color, byte coordinate_system, UInt16 timeout, Single x, Single y, Single z, string name)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(type),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(color),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(coordinate_system),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(timeout),0,msg,3,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(x),0,msg,5,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,9,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,13,sizeof(Single));
	Array.Copy(toArray(name),0,msg,17,26);
} else {
    mavlink_point_of_interest_t packet = new mavlink_point_of_interest_t();
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.name = name;
        
        int len = 43;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;
    //return mavlink_finalize_message(msg, system_id, component_id, 43);
    return 0;
}

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
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, color);
	_mav_put_byte(buf, 2, coordinate_system);
	_mav_put_UInt16(buf, 3, timeout);
	_mav_put_Single(buf, 5, x);
	_mav_put_Single(buf, 9, y);
	_mav_put_Single(buf, 13, z);
	_mav_put_string_array(buf, 17, name, 26);
        memcpy(_MAV_PAYLOAD(msg), buf, 43);
#else
    mavlink_point_of_interest_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	memcpy(packet.name, name, sizeof(string)*26);
        memcpy(_MAV_PAYLOAD(msg), &packet, 43);
#endif

    msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 43);
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
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, color);
	_mav_put_byte(buf, 2, coordinate_system);
	_mav_put_UInt16(buf, 3, timeout);
	_mav_put_Single(buf, 5, x);
	_mav_put_Single(buf, 9, y);
	_mav_put_Single(buf, 13, z);
	_mav_put_string_array(buf, 17, name, 26);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST, buf, 43);
#else
    mavlink_point_of_interest_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	memcpy(packet.name, name, sizeof(string)*26);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST, (const char *)&packet, 43);
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
    return getByte(msg,  0);
}

/**
 * @brief Get field color from point_of_interest message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
public static byte mavlink_msg_point_of_interest_get_color(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field coordinate_system from point_of_interest message
 *
 * @return 0: global, 1:local
 */
public static byte mavlink_msg_point_of_interest_get_coordinate_system(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field timeout from point_of_interest message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
public static UInt16 mavlink_msg_point_of_interest_get_timeout(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  3);
}

/**
 * @brief Get field x from point_of_interest message
 *
 * @return X Position
 */
public static Single mavlink_msg_point_of_interest_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  5);
}

/**
 * @brief Get field y from point_of_interest message
 *
 * @return Y Position
 */
public static Single mavlink_msg_point_of_interest_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  9);
}

/**
 * @brief Get field z from point_of_interest message
 *
 * @return Z Position
 */
public static Single mavlink_msg_point_of_interest_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  13);
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
    	point_of_interest.type = mavlink_msg_point_of_interest_get_type(msg);
    	point_of_interest.color = mavlink_msg_point_of_interest_get_color(msg);
    	point_of_interest.coordinate_system = mavlink_msg_point_of_interest_get_coordinate_system(msg);
    	point_of_interest.timeout = mavlink_msg_point_of_interest_get_timeout(msg);
    	point_of_interest.x = mavlink_msg_point_of_interest_get_x(msg);
    	point_of_interest.y = mavlink_msg_point_of_interest_get_y(msg);
    	point_of_interest.z = mavlink_msg_point_of_interest_get_z(msg);
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
