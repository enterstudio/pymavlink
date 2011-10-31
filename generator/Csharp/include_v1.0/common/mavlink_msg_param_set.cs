// MESSAGE PARAM_SET PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PARAM_SET = 23;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_param_set_t
    {
        /// <summary>
        /// Onboard parameter value
        /// </summary>
        public  Single param_value;
            /// <summary>
        /// System ID
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component ID
        /// </summary>
        public  byte target_component;
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
/// * @brief Pack a param_set message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param param_id Onboard parameter id
/// * @param param_value Onboard parameter value
/// * @param param_type Onboard parameter type: see MAV_VAR enum
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_param_set_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, string param_id, Single param_value, byte param_type)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(param_value),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,4,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,5,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(param_type),0,msg,22,sizeof(byte));
	Array.Copy(toArray(param_id),0,msg,6,16);
} else {
    mavlink_param_set_t packet = new mavlink_param_set_t();
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_type = param_type;
	packet.param_id = param_id;
        
        int len = 23;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_PARAM_SET;
    //return mavlink_finalize_message(msg, system_id, component_id, 23, 168);
    return 0;
}

/**
 * @brief Pack a param_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_param_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 publicparam_id,Single public param_value,byte public param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[23];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_byte(buf, 22, param_type);
	_mav_put_string_array(buf, 6, param_id, 16);
        memcpy(_MAV_PAYLOAD(msg), buf, 23);
#else
    mavlink_param_set_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_type = param_type;
	memcpy(packet.param_id, param_id, sizeof(string)*16);
        memcpy(_MAV_PAYLOAD(msg), &packet, 23);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 23, 168);
}
*/
/**
 * @brief Encode a param_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_set C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_param_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_set_t* param_set)
{
    return mavlink_msg_param_set_pack(system_id, component_id, msg, param_set->target_system, param_set->target_component, param_set->param_id, param_set->param_value, param_set->param_type);
}
*/
/**
 * @brief Send a param_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see MAV_VAR enum
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_set_send(mavlink_channel_t chan, byte public target_system, byte public target_component, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 publicparam_id, Single public param_value, byte public param_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[23];
	_mav_put_Single(buf, 0, param_value);
	_mav_put_byte(buf, 4, target_system);
	_mav_put_byte(buf, 5, target_component);
	_mav_put_byte(buf, 22, param_type);
	_mav_put_string_array(buf, 6, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, buf, 23, 168);
#else
    mavlink_param_set_t packet;
	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_type = param_type;
	memcpy(packet.param_id, param_id, sizeof(string)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_SET, (const char *)&packet, 23, 168);
#endif
}

#endif
*/
// MESSAGE PARAM_SET UNPACKING


/**
 * @brief Get field target_system from param_set message
 *
 * @return System ID
 */
public static byte mavlink_msg_param_set_get_target_system(byte[] msg)
{
    return getByte(msg,  4);
}

/**
 * @brief Get field target_component from param_set message
 *
 * @return Component ID
 */
public static byte mavlink_msg_param_set_get_target_component(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Get field param_id from param_set message
 *
 * @return Onboard parameter id
 */
public static string mavlink_msg_param_set_get_param_id(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,6,16); //(msg, 16,  6);
}

/**
 * @brief Get field param_value from param_set message
 *
 * @return Onboard parameter value
 */
public static Single mavlink_msg_param_set_get_param_value(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field param_type from param_set message
 *
 * @return Onboard parameter type: see MAV_VAR enum
 */
public static byte mavlink_msg_param_set_get_param_type(byte[] msg)
{
    return getByte(msg,  22);
}

/**
 * @brief Decode a param_set message into a struct
 *
 * @param msg The message to decode
 * @param param_set C-struct to decode the message contents into
 */
public static void mavlink_msg_param_set_decode(byte[] msg, ref mavlink_param_set_t param_set)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	param_set.param_value = mavlink_msg_param_set_get_param_value(msg);
    	param_set.target_system = mavlink_msg_param_set_get_target_system(msg);
    	param_set.target_component = mavlink_msg_param_set_get_target_component(msg);
    	param_set.param_id = mavlink_msg_param_set_get_param_id(msg);
    	param_set.param_type = mavlink_msg_param_set_get_param_type(msg);
    
    } else {
        int len = 23; //Marshal.SizeOf(param_set);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        param_set = (mavlink_param_set_t)Marshal.PtrToStructure(i, ((object)param_set).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
