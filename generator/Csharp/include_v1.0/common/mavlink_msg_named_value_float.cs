// MESSAGE NAMED_VALUE_FLOAT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_NAMED_VALUE_FLOAT = 252;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_named_value_float_t
    {
         public  Single value; /// Floating point value
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 public string name; /// Name of the debug variable
    
    };

/**
 * @brief Pack a named_value_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the debug variable
 * @param value Floating point value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_named_value_float_pack(byte system_id, byte component_id, ref byte[] msg,
                               const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname, Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[14];
	_mav_put_Single(buf, 0, value);
	_mav_put_string_array(buf, 4, name, 10);
        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_named_value_float_t packet;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
    return mavlink_finalize_message(msg, system_id, component_id, 14, 248);
}
*/
/**
 * @brief Pack a named_value_float message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the debug variable
 * @param value Floating point value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_named_value_float_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname,Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_Single(buf, 0, value);
	_mav_put_string_array(buf, 4, name, 10);
        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_named_value_float_t packet;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 248);
}
*/
/**
 * @brief Encode a named_value_float struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_value_float C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_named_value_float_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_value_float_t* named_value_float)
{
    return mavlink_msg_named_value_float_pack(system_id, component_id, msg, named_value_float->name, named_value_float->value);
}
*/
/**
 * @brief Send a named_value_float message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the debug variable
 * @param value Floating point value
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_float_send(mavlink_channel_t chan, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname, Single public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_Single(buf, 0, value);
	_mav_put_string_array(buf, 4, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, buf, 14, 248);
#else
    mavlink_named_value_float_t packet;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, (const char *)&packet, 14, 248);
#endif
}

#endif
*/
// MESSAGE NAMED_VALUE_FLOAT UNPACKING


/**
 * @brief Get field name from named_value_float message
 *
 * @return Name of the debug variable
 */
public static string mavlink_msg_named_value_float_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,4,10); //(msg, 10,  4);
}

/**
 * @brief Get field value from named_value_float message
 *
 * @return Floating point value
 */
public static Single mavlink_msg_named_value_float_get_value(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Decode a named_value_float message into a struct
 *
 * @param msg The message to decode
 * @param named_value_float C-struct to decode the message contents into
 */
public static void mavlink_msg_named_value_float_decode(byte[] msg, ref mavlink_named_value_float_t named_value_float)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	named_value_float.value = mavlink_msg_named_value_float_get_value(msg);
	named_value_float.name = mavlink_msg_named_value_float_get_name(msg);
} else {
    int len = 14; //Marshal.SizeOf(named_value_float);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    named_value_float = (mavlink_named_value_float_t)Marshal.PtrToStructure(i, ((object)named_value_float).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
