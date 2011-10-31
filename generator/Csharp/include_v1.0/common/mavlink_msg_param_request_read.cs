// MESSAGE PARAM_REQUEST_READ PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_param_request_read_t
    {
        /// <summary>
        /// Parameter index. Send -1 to use the param ID field as identifier
        /// </summary>
        public  Int16 param_index;
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
    
    };

/// <summary>
/// * @brief Pack a param_request_read message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param param_id Onboard parameter id
/// * @param param_index Parameter index. Send -1 to use the param ID field as identifier
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_param_request_read_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, string param_id, Int16 param_index)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(param_index),0,msg,0,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,3,sizeof(byte));
	Array.Copy(toArray(param_id),0,msg,4,16);
} else {
    mavlink_param_request_read_t packet = new mavlink_param_request_read_t();
	packet.param_index = param_index;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.param_id = param_id;
        
        int len = 20;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;
    //return mavlink_finalize_message(msg, system_id, component_id, 20, 214);
    return 0;
}

/**
 * @brief Pack a param_request_read message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_index Parameter index. Send -1 to use the param ID field as identifier
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_param_request_read_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 publicparam_id,Int16 public param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Int16(buf, 0, param_index);
	_mav_put_byte(buf, 2, target_system);
	_mav_put_byte(buf, 3, target_component);
	_mav_put_string_array(buf, 4, param_id, 16);
        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_param_request_read_t packet;
	packet.param_index = param_index;
	packet.target_system = target_system;
	packet.target_component = target_component;
	memcpy(packet.param_id, param_id, sizeof(string)*16);
        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 214);
}
*/
/**
 * @brief Encode a param_request_read struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_read C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_param_request_read_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_request_read_t* param_request_read)
{
    return mavlink_msg_param_request_read_pack(system_id, component_id, msg, param_request_read->target_system, param_request_read->target_component, param_request_read->param_id, param_request_read->param_index);
}
*/
/**
 * @brief Send a param_request_read message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param param_id Onboard parameter id
 * @param param_index Parameter index. Send -1 to use the param ID field as identifier
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_request_read_send(mavlink_channel_t chan, byte public target_system, byte public target_component, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=16)]
 publicparam_id, Int16 public param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Int16(buf, 0, param_index);
	_mav_put_byte(buf, 2, target_system);
	_mav_put_byte(buf, 3, target_component);
	_mav_put_string_array(buf, 4, param_id, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_READ, buf, 20, 214);
#else
    mavlink_param_request_read_t packet;
	packet.param_index = param_index;
	packet.target_system = target_system;
	packet.target_component = target_component;
	memcpy(packet.param_id, param_id, sizeof(string)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_READ, (const char *)&packet, 20, 214);
#endif
}

#endif
*/
// MESSAGE PARAM_REQUEST_READ UNPACKING


/**
 * @brief Get field target_system from param_request_read message
 *
 * @return System ID
 */
public static byte mavlink_msg_param_request_read_get_target_system(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field target_component from param_request_read message
 *
 * @return Component ID
 */
public static byte mavlink_msg_param_request_read_get_target_component(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field param_id from param_request_read message
 *
 * @return Onboard parameter id
 */
public static string mavlink_msg_param_request_read_get_param_id(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,4,16); //(msg, 16,  4);
}

/**
 * @brief Get field param_index from param_request_read message
 *
 * @return Parameter index. Send -1 to use the param ID field as identifier
 */
public static Int16 mavlink_msg_param_request_read_get_param_index(byte[] msg)
{
    return BitConverter.ToInt16(msg,  0);
}

/**
 * @brief Decode a param_request_read message into a struct
 *
 * @param msg The message to decode
 * @param param_request_read C-struct to decode the message contents into
 */
public static void mavlink_msg_param_request_read_decode(byte[] msg, ref mavlink_param_request_read_t param_request_read)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	param_request_read.param_index = mavlink_msg_param_request_read_get_param_index(msg);
    	param_request_read.target_system = mavlink_msg_param_request_read_get_target_system(msg);
    	param_request_read.target_component = mavlink_msg_param_request_read_get_target_component(msg);
    	param_request_read.param_id = mavlink_msg_param_request_read_get_param_id(msg);
    
    } else {
        int len = 20; //Marshal.SizeOf(param_request_read);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        param_request_read = (mavlink_param_request_read_t)Marshal.PtrToStructure(i, ((object)param_request_read).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
