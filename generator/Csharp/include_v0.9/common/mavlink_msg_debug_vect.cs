// MESSAGE DEBUG_VECT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_DEBUG_VECT = 251;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_debug_vect_t
    {
         [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 public string name; /// Name
     public  UInt64 usec; /// Timestamp
     public  Single x; /// x
     public  Single y; /// y
     public  Single z; /// z
    
    };

/**
 * @brief Pack a debug_vect message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name
 * @param usec Timestamp
 * @param x x
 * @param y y
 * @param z z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_debug_vect_pack(byte system_id, byte component_id, byte[] msg,
                               string name, UInt64 usec, Single x, Single y, Single z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(usec),0,msg,10,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(x),0,msg,18,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,22,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,26,sizeof(Single));
	//Array.Copy(name,0,msg,0,10);
} else {
    mavlink_debug_vect_t packet = new mavlink_debug_vect_t();
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.name = name;
        
        int len = 30;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_DEBUG_VECT;
    //return mavlink_finalize_message(msg, system_id, component_id, 30);
    return 0;
}

/**
 * @brief Pack a debug_vect message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name
 * @param usec Timestamp
 * @param x x
 * @param y y
 * @param z z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_debug_vect_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname,UInt64 public usec,Single public x,Single public y,Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[30];
	_mav_put_UInt64(buf, 10, usec);
	_mav_put_Single(buf, 18, x);
	_mav_put_Single(buf, 22, y);
	_mav_put_Single(buf, 26, z);
	_mav_put_string_array(buf, 0, name, 10);
        memcpy(_MAV_PAYLOAD(msg), buf, 30);
#else
    mavlink_debug_vect_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	memcpy(packet.name, name, sizeof(string)*10);
        memcpy(_MAV_PAYLOAD(msg), &packet, 30);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEBUG_VECT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 30);
}
*/
/**
 * @brief Encode a debug_vect struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug_vect C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_debug_vect_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_vect_t* debug_vect)
{
    return mavlink_msg_debug_vect_pack(system_id, component_id, msg, debug_vect->name, debug_vect->usec, debug_vect->x, debug_vect->y, debug_vect->z);
}
*/
/**
 * @brief Send a debug_vect message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name
 * @param usec Timestamp
 * @param x x
 * @param y y
 * @param z z
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_debug_vect_send(mavlink_channel_t chan, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname, UInt64 public usec, Single public x, Single public y, Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[30];
	_mav_put_UInt64(buf, 10, usec);
	_mav_put_Single(buf, 18, x);
	_mav_put_Single(buf, 22, y);
	_mav_put_Single(buf, 26, z);
	_mav_put_string_array(buf, 0, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, buf, 30);
#else
    mavlink_debug_vect_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	memcpy(packet.name, name, sizeof(string)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VECT, (const char *)&packet, 30);
#endif
}

#endif
*/
// MESSAGE DEBUG_VECT UNPACKING


/**
 * @brief Get field name from debug_vect message
 *
 * @return Name
 */
public static string mavlink_msg_debug_vect_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,0,10); //(msg, 10,  0);
}

/**
 * @brief Get field usec from debug_vect message
 *
 * @return Timestamp
 */
public static UInt64 mavlink_msg_debug_vect_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  10);
}

/**
 * @brief Get field x from debug_vect message
 *
 * @return x
 */
public static Single mavlink_msg_debug_vect_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  18);
}

/**
 * @brief Get field y from debug_vect message
 *
 * @return y
 */
public static Single mavlink_msg_debug_vect_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  22);
}

/**
 * @brief Get field z from debug_vect message
 *
 * @return z
 */
public static Single mavlink_msg_debug_vect_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  26);
}

/**
 * @brief Decode a debug_vect message into a struct
 *
 * @param msg The message to decode
 * @param debug_vect C-struct to decode the message contents into
 */
public static void mavlink_msg_debug_vect_decode(byte[] msg, ref mavlink_debug_vect_t debug_vect)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	debug_vect.name = mavlink_msg_debug_vect_get_name(msg);
    	debug_vect.usec = mavlink_msg_debug_vect_get_usec(msg);
    	debug_vect.x = mavlink_msg_debug_vect_get_x(msg);
    	debug_vect.y = mavlink_msg_debug_vect_get_y(msg);
    	debug_vect.z = mavlink_msg_debug_vect_get_z(msg);
    
    } else {
        int len = 30; //Marshal.SizeOf(debug_vect);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        debug_vect = (mavlink_debug_vect_t)Marshal.PtrToStructure(i, ((object)debug_vect).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
