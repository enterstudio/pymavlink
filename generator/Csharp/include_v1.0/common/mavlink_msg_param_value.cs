// MESSAGE PARAM_VALUE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PARAM_VALUE = 22;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_param_value_t
    {
        /// <summary>
        /// Onboard parameter value
        /// </summary>
        public  Single param_value;
            /// <summary>
        /// Total number of onboard parameters
        /// </summary>
        public  UInt16 param_count;
            /// <summary>
        /// Index of this onboard parameter
        /// </summary>
        public  UInt16 param_index;
            /// <summary>
        /// Onboard parameter id
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 public string param_id;
            /// <summary>
        /// Onboard parameter type: see MAV_VAR enum
        /// </summary>
        public  byte param_type;
    
    };

/// <summary>
/// * @brief Pack a param_value message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param param_id Onboard parameter id
/// * @param param_value Onboard parameter value
/// * @param param_type Onboard parameter type: see MAV_VAR enum
/// * @param param_count Total number of onboard parameters
/// * @param param_index Index of this onboard parameter
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_param_value_pack(byte system_id, byte component_id, byte[] msg,
                               string param_id, Single param_value, byte param_type, UInt16 param_count, UInt16 param_index)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(param_value),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param_count),0,msg,4,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(param_index),0,msg,6,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(param_type),0,msg,24,sizeof(byte));
	Array.Copy(toArray(param_id),0,msg,8,16);
} else {
    mavlink_param_value_t packet = new mavlink_param_value_t();
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;
	packet.param_type = param_type;
	packet.param_id = param_id;
        
        int len = 25;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_PARAM_VALUE;
    //return mavlink_finalize_message(msg, system_id, component_id, 25, 220);
    return 0;
}

/**
 * @brief Pack a param_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_param_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 publicparam_id,Single public param_value,byte public param_type,UInt16 public param_count,UInt16 public param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[25];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_UInt16(buf, 4, param_count);
	_mav_put_UInt16(buf, 6, param_index);
	_mav_put_byte(buf, 24, param_type);
	_mav_put_string_array(buf, 8, param_id, 16);
        memcpy(_MAV_PAYLOAD(msg), buf, 25);
#else
    mavlink_param_value_t packet;
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;
	packet.param_type = param_type;
	memcpy(packet.param_id, param_id, sizeof(string)*16);
        memcpy(_MAV_PAYLOAD(msg), &packet, 25);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 25, 220);
}
*/
/**
 * @brief Encode a param_value struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_value C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_param_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_value_t* param_value)
{
    return mavlink_msg_param_value_pack(system_id, component_id, msg, param_value->param_id, param_value->param_value, param_value->param_type, param_value->param_count, param_value->param_index);
}
*/
/**
 * @brief Send a param_value message
 * @param chan MAVLink channel to send the message
 *
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_value_send(mavlink_channel_t chan, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 publicparam_id, Single public param_value, byte public param_type, UInt16 public param_count, UInt16 public param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[25];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_UInt16(buf, 4, param_count);
	_mav_put_UInt16(buf, 6, param_index);
	_mav_put_byte(buf, 24, param_type);
	_mav_put_string_array(buf, 8, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, buf, 25, 220);
#else
    mavlink_param_value_t packet;
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;
	packet.param_type = param_type;
	memcpy(packet.param_id, param_id, sizeof(string)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE, (const char *)&packet, 25, 220);
#endif
}

#endif
*/
// MESSAGE PARAM_VALUE UNPACKING


/**
 * @brief Get field param_id from param_value message
 *
 * @return Onboard parameter id
 */
public static string mavlink_msg_param_value_get_param_id(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,8,16); //(msg, 16,  8);
}

/**
 * @brief Get field param_value from param_value message
 *
 * @return Onboard parameter value
 */
public static Single mavlink_msg_param_value_get_param_value(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field param_type from param_value message
 *
 * @return Onboard parameter type: see MAV_VAR enum
 */
public static byte mavlink_msg_param_value_get_param_type(byte[] msg)
{
    return getByte(msg,  24);
}

/**
 * @brief Get field param_count from param_value message
 *
 * @return Total number of onboard parameters
 */
public static UInt16 mavlink_msg_param_value_get_param_count(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field param_index from param_value message
 *
 * @return Index of this onboard parameter
 */
public static UInt16 mavlink_msg_param_value_get_param_index(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Decode a param_value message into a struct
 *
 * @param msg The message to decode
 * @param param_value C-struct to decode the message contents into
 */
public static void mavlink_msg_param_value_decode(byte[] msg, ref mavlink_param_value_t param_value)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	param_value.param_value = mavlink_msg_param_value_get_param_value(msg);
    	param_value.param_count = mavlink_msg_param_value_get_param_count(msg);
    	param_value.param_index = mavlink_msg_param_value_get_param_index(msg);
    	param_value.param_id = mavlink_msg_param_value_get_param_id(msg);
    	param_value.param_type = mavlink_msg_param_value_get_param_type(msg);
    
    } else {
        int len = 25; //Marshal.SizeOf(param_value);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        param_value = (mavlink_param_value_t)Marshal.PtrToStructure(i, ((object)param_value).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
