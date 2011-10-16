// MESSAGE NAMED_VALUE_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 253;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_named_value_int_t
    {
         [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 public string name; /// Name of the debug variable
     public  Int32 value; /// Signed integer value
    
    };

/**
 * @brief Pack a named_value_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_named_value_int_pack(byte system_id, byte component_id, ref byte[] msg,
                               const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname, Int32 public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[14];
	_mav_put_Int32(buf, 10, value);
	_mav_put_string_array(buf, 0, name, 10);
        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_named_value_int_t packet;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;
    return mavlink_finalize_message(msg, system_id, component_id, 14);
}
*/
/**
 * @brief Pack a named_value_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_named_value_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname,Int32 public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_Int32(buf, 10, value);
	_mav_put_string_array(buf, 0, name, 10);
        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_named_value_int_t packet;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14);
}
*/
/**
 * @brief Encode a named_value_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_int C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_named_value_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_int_t* named_value_int)
{
    return mavlink_msg_named_value_int_pack(system_id, component_id, msg, named_value_int->name, named_value_int->value);
}
*/
/**
 * @brief Send a named_value_int message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the debug variable
 * @param value Signed integer value
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_int_send(mavlink_channel_t chan, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname, Int32 public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_Int32(buf, 10, value);
	_mav_put_string_array(buf, 0, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, buf, 14);
#else
    mavlink_named_value_int_t packet;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, (const char *)&packet, 14);
#endif
}

#endif
*/
// MESSAGE NAMED_VALUE_INT UNPACKING


/**
 * @brief Get field name from named_value_int message
 *
 * @return Name of the debug variable
 */
public static string mavlink_msg_named_value_int_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,0,10); //(msg, 10,  0);
}

/**
 * @brief Get field value from named_value_int message
 *
 * @return Signed integer value
 */
public static Int32 mavlink_msg_named_value_int_get_value(byte[] msg)
{
    return BitConverter.ToInt32(msg,  10);
}

/**
 * @brief Decode a named_value_int message into a struct
 *
 * @param msg The message to decode
 * @param named_value_int C-struct to decode the message contents into
 */
public static void mavlink_msg_named_value_int_decode(byte[] msg, ref mavlink_named_value_int_t named_value_int)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	named_value_int.name = mavlink_msg_named_value_int_get_name(msg);
	named_value_int.value = mavlink_msg_named_value_int_get_value(msg);
} else {
    int len = 14; //Marshal.SizeOf(named_value_int);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    named_value_int = (mavlink_named_value_int_t)Marshal.PtrToStructure(i, ((object)named_value_int).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
