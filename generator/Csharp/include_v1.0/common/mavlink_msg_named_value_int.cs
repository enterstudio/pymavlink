// MESSAGE NAMED_VALUE_INT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_NAMED_VALUE_INT = 252;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_named_value_int_t
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// Signed integer value
        /// </summary>
        public  Int32 value;
            /// <summary>
        /// Name of the debug variable
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 public string name;
    
    };

/// <summary>
/// * @brief Pack a named_value_int message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp (milliseconds since system boot)
/// * @param name Name of the debug variable
/// * @param value Signed integer value
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_named_value_int_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, string name, Int32 value)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(value),0,msg,4,sizeof(Int32));
	Array.Copy(toArray(name),0,msg,8,10);
} else {
    mavlink_named_value_int_t packet = new mavlink_named_value_int_t();
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	packet.name = name;
        
        int len = 18;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;
    //return mavlink_finalize_message(msg, system_id, component_id, 18, 44);
    return 0;
}

/**
 * @brief Pack a named_value_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param name Name of the debug variable
 * @param value Signed integer value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_named_value_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname,Int32 public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, value);
	_mav_put_string_array(buf, 8, name, 10);
        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_named_value_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAMED_VALUE_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 44);
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
    return mavlink_msg_named_value_int_pack(system_id, component_id, msg, named_value_int->time_boot_ms, named_value_int->name, named_value_int->value);
}
*/
/**
 * @brief Send a named_value_int message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param name Name of the debug variable
 * @param value Signed integer value
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_value_int_send(mavlink_channel_t chan, UInt32 public time_boot_ms, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=10)]
 publicname, Int32 public value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int32(buf, 4, value);
	_mav_put_string_array(buf, 8, name, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, buf, 18, 44);
#else
    mavlink_named_value_int_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.value = value;
	memcpy(packet.name, name, sizeof(string)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_VALUE_INT, (const char *)&packet, 18, 44);
#endif
}

#endif
*/
// MESSAGE NAMED_VALUE_INT UNPACKING


/**
 * @brief Get field time_boot_ms from named_value_int message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_named_value_int_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field name from named_value_int message
 *
 * @return Name of the debug variable
 */
public static string mavlink_msg_named_value_int_get_name(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,8,10); //(msg, 10,  8);
}

/**
 * @brief Get field value from named_value_int message
 *
 * @return Signed integer value
 */
public static Int32 mavlink_msg_named_value_int_get_value(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
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
    	named_value_int.time_boot_ms = mavlink_msg_named_value_int_get_time_boot_ms(msg);
    	named_value_int.value = mavlink_msg_named_value_int_get_value(msg);
    	named_value_int.name = mavlink_msg_named_value_int_get_name(msg);
    
    } else {
        int len = 18; //Marshal.SizeOf(named_value_int);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        named_value_int = (mavlink_named_value_int_t)Marshal.PtrToStructure(i, ((object)named_value_int).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
