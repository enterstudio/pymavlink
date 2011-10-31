// MESSAGE POINT_OF_INTEREST_CONNECTION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION = 192;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_point_of_interest_connection_t
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
        /// X1 Position
        /// </summary>
        public  Single xp1;
            /// <summary>
        /// Y1 Position
        /// </summary>
        public  Single yp1;
            /// <summary>
        /// Z1 Position
        /// </summary>
        public  Single zp1;
            /// <summary>
        /// X2 Position
        /// </summary>
        public  Single xp2;
            /// <summary>
        /// Y2 Position
        /// </summary>
        public  Single yp2;
            /// <summary>
        /// Z2 Position
        /// </summary>
        public  Single zp2;
            /// <summary>
        /// POI connection name
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 public string name;
    
    };

/// <summary>
/// * @brief Pack a point_of_interest_connection message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
/// * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
/// * @param coordinate_system 0: global, 1:local
/// * @param timeout 0: no timeout, >1: timeout in seconds
/// * @param xp1 X1 Position
/// * @param yp1 Y1 Position
/// * @param zp1 Z1 Position
/// * @param xp2 X2 Position
/// * @param yp2 Y2 Position
/// * @param zp2 Z2 Position
/// * @param name POI connection name
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_point_of_interest_connection_pack(byte system_id, byte component_id, byte[] msg,
                               byte type, byte color, byte coordinate_system, UInt16 timeout, Single xp1, Single yp1, Single zp1, Single xp2, Single yp2, Single zp2, string name)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(type),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(color),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(coordinate_system),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(timeout),0,msg,3,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(xp1),0,msg,5,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yp1),0,msg,9,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(zp1),0,msg,13,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(xp2),0,msg,17,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yp2),0,msg,21,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(zp2),0,msg,25,sizeof(Single));
	Array.Copy(toArray(name),0,msg,29,26);
} else {
    mavlink_point_of_interest_connection_t packet = new mavlink_point_of_interest_connection_t();
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.xp1 = xp1;
	packet.yp1 = yp1;
	packet.zp1 = zp1;
	packet.xp2 = xp2;
	packet.yp2 = yp2;
	packet.zp2 = zp2;
	packet.name = name;
        
        int len = 55;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION;
    //return mavlink_finalize_message(msg, system_id, component_id, 55);
    return 0;
}

/**
 * @brief Pack a point_of_interest_connection message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param xp1 X1 Position
 * @param yp1 Y1 Position
 * @param zp1 Z1 Position
 * @param xp2 X2 Position
 * @param yp2 Y2 Position
 * @param zp2 Z2 Position
 * @param name POI connection name
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_point_of_interest_connection_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,byte public color,byte public coordinate_system,UInt16 public timeout,Single public xp1,Single public yp1,Single public zp1,Single public xp2,Single public yp2,Single public zp2,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 publicname)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[55];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, color);
	_mav_put_byte(buf, 2, coordinate_system);
	_mav_put_UInt16(buf, 3, timeout);
	_mav_put_Single(buf, 5, xp1);
	_mav_put_Single(buf, 9, yp1);
	_mav_put_Single(buf, 13, zp1);
	_mav_put_Single(buf, 17, xp2);
	_mav_put_Single(buf, 21, yp2);
	_mav_put_Single(buf, 25, zp2);
	_mav_put_string_array(buf, 29, name, 26);
        memcpy(_MAV_PAYLOAD(msg), buf, 55);
#else
    mavlink_point_of_interest_connection_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.xp1 = xp1;
	packet.yp1 = yp1;
	packet.zp1 = zp1;
	packet.xp2 = xp2;
	packet.yp2 = yp2;
	packet.zp2 = zp2;
	memcpy(packet.name, name, sizeof(string)*26);
        memcpy(_MAV_PAYLOAD(msg), &packet, 55);
#endif

    msg->msgid = MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 55);
}
*/
/**
 * @brief Encode a point_of_interest_connection struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param point_of_interest_connection C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_point_of_interest_connection_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_point_of_interest_connection_t* point_of_interest_connection)
{
    return mavlink_msg_point_of_interest_connection_pack(system_id, component_id, msg, point_of_interest_connection->type, point_of_interest_connection->color, point_of_interest_connection->coordinate_system, point_of_interest_connection->timeout, point_of_interest_connection->xp1, point_of_interest_connection->yp1, point_of_interest_connection->zp1, point_of_interest_connection->xp2, point_of_interest_connection->yp2, point_of_interest_connection->zp2, point_of_interest_connection->name);
}
*/
/**
 * @brief Send a point_of_interest_connection message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 * @param color 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 * @param coordinate_system 0: global, 1:local
 * @param timeout 0: no timeout, >1: timeout in seconds
 * @param xp1 X1 Position
 * @param yp1 Y1 Position
 * @param zp1 Z1 Position
 * @param xp2 X2 Position
 * @param yp2 Y2 Position
 * @param zp2 Z2 Position
 * @param name POI connection name
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_point_of_interest_connection_send(mavlink_channel_t chan, byte public type, byte public color, byte public coordinate_system, UInt16 public timeout, Single public xp1, Single public yp1, Single public zp1, Single public xp2, Single public yp2, Single public zp2, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=26)]
 publicname)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[55];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, color);
	_mav_put_byte(buf, 2, coordinate_system);
	_mav_put_UInt16(buf, 3, timeout);
	_mav_put_Single(buf, 5, xp1);
	_mav_put_Single(buf, 9, yp1);
	_mav_put_Single(buf, 13, zp1);
	_mav_put_Single(buf, 17, xp2);
	_mav_put_Single(buf, 21, yp2);
	_mav_put_Single(buf, 25, zp2);
	_mav_put_string_array(buf, 29, name, 26);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION, buf, 55);
#else
    mavlink_point_of_interest_connection_t packet;
	packet.type = type;
	packet.color = color;
	packet.coordinate_system = coordinate_system;
	packet.timeout = timeout;
	packet.xp1 = xp1;
	packet.yp1 = yp1;
	packet.zp1 = zp1;
	packet.xp2 = xp2;
	packet.yp2 = yp2;
	packet.zp2 = zp2;
	memcpy(packet.name, name, sizeof(string)*26);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POINT_OF_INTEREST_CONNECTION, (const char *)&packet, 55);
#endif
}

#endif
*/
// MESSAGE POINT_OF_INTEREST_CONNECTION UNPACKING


/**
 * @brief Get field type from point_of_interest_connection message
 *
 * @return 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
 */
public static byte mavlink_msg_point_of_interest_connection_get_type(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field color from point_of_interest_connection message
 *
 * @return 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
 */
public static byte mavlink_msg_point_of_interest_connection_get_color(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field coordinate_system from point_of_interest_connection message
 *
 * @return 0: global, 1:local
 */
public static byte mavlink_msg_point_of_interest_connection_get_coordinate_system(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field timeout from point_of_interest_connection message
 *
 * @return 0: no timeout, >1: timeout in seconds
 */
public static UInt16 mavlink_msg_point_of_interest_connection_get_timeout(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  3);
}

/**
 * @brief Get field xp1 from point_of_interest_connection message
 *
 * @return X1 Position
 */
public static Single mavlink_msg_point_of_interest_connection_get_xp1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  5);
}

/**
 * @brief Get field yp1 from point_of_interest_connection message
 *
 * @return Y1 Position
 */
public static Single mavlink_msg_point_of_interest_connection_get_yp1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  9);
}

/**
 * @brief Get field zp1 from point_of_interest_connection message
 *
 * @return Z1 Position
 */
public static Single mavlink_msg_point_of_interest_connection_get_zp1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  13);
}

/**
 * @brief Get field xp2 from point_of_interest_connection message
 *
 * @return X2 Position
 */
public static Single mavlink_msg_point_of_interest_connection_get_xp2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  17);
}

/**
 * @brief Get field yp2 from point_of_interest_connection message
 *
 * @return Y2 Position
 */
public static Single mavlink_msg_point_of_interest_connection_get_yp2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  21);
}

/**
 * @brief Get field zp2 from point_of_interest_connection message
 *
 * @return Z2 Position
 */
public static Single mavlink_msg_point_of_interest_connection_get_zp2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  25);
}

/**
 * @brief Get field name from point_of_interest_connection message
 *
 * @return POI connection name
 */
public static string mavlink_msg_point_of_interest_connection_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,29,26); //(msg, 26,  29);
}

/**
 * @brief Decode a point_of_interest_connection message into a struct
 *
 * @param msg The message to decode
 * @param point_of_interest_connection C-struct to decode the message contents into
 */
public static void mavlink_msg_point_of_interest_connection_decode(byte[] msg, ref mavlink_point_of_interest_connection_t point_of_interest_connection)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	point_of_interest_connection.type = mavlink_msg_point_of_interest_connection_get_type(msg);
    	point_of_interest_connection.color = mavlink_msg_point_of_interest_connection_get_color(msg);
    	point_of_interest_connection.coordinate_system = mavlink_msg_point_of_interest_connection_get_coordinate_system(msg);
    	point_of_interest_connection.timeout = mavlink_msg_point_of_interest_connection_get_timeout(msg);
    	point_of_interest_connection.xp1 = mavlink_msg_point_of_interest_connection_get_xp1(msg);
    	point_of_interest_connection.yp1 = mavlink_msg_point_of_interest_connection_get_yp1(msg);
    	point_of_interest_connection.zp1 = mavlink_msg_point_of_interest_connection_get_zp1(msg);
    	point_of_interest_connection.xp2 = mavlink_msg_point_of_interest_connection_get_xp2(msg);
    	point_of_interest_connection.yp2 = mavlink_msg_point_of_interest_connection_get_yp2(msg);
    	point_of_interest_connection.zp2 = mavlink_msg_point_of_interest_connection_get_zp2(msg);
    	point_of_interest_connection.name = mavlink_msg_point_of_interest_connection_get_name(msg);
    
    } else {
        int len = 55; //Marshal.SizeOf(point_of_interest_connection);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        point_of_interest_connection = (mavlink_point_of_interest_connection_t)Marshal.PtrToStructure(i, ((object)point_of_interest_connection).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
