// MESSAGE PARAM_REQUEST_LIST PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_param_request_list_t
    {
         public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
    
    };

/**
 * @brief Pack a param_request_list message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_param_request_list_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target_system, byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[2];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_param_request_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_LIST;
    return mavlink_finalize_message(msg, system_id, component_id, 2, 159);
}
*/
/**
 * @brief Pack a param_request_list message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_param_request_list_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_param_request_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_REQUEST_LIST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 159);
}
*/
/**
 * @brief Encode a param_request_list struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_request_list C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_param_request_list_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_request_list_t* param_request_list)
{
    return mavlink_msg_param_request_list_pack(system_id, component_id, msg, param_request_list->target_system, param_request_list->target_component);
}
*/
/**
 * @brief Send a param_request_list message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_request_list_send(mavlink_channel_t chan, byte public target_system, byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_LIST, buf, 2, 159);
#else
    mavlink_param_request_list_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_REQUEST_LIST, (const char *)&packet, 2, 159);
#endif
}

#endif
*/
// MESSAGE PARAM_REQUEST_LIST UNPACKING


/**
 * @brief Get field target_system from param_request_list message
 *
 * @return System ID
 */
public static byte mavlink_msg_param_request_list_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from param_request_list message
 *
 * @return Component ID
 */
public static byte mavlink_msg_param_request_list_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Decode a param_request_list message into a struct
 *
 * @param msg The message to decode
 * @param param_request_list C-struct to decode the message contents into
 */
public static void mavlink_msg_param_request_list_decode(byte[] msg, ref mavlink_param_request_list_t param_request_list)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	param_request_list.target_system = mavlink_msg_param_request_list_get_target_system(msg);
	param_request_list.target_component = mavlink_msg_param_request_list_get_target_component(msg);
} else {
    int len = 2; //Marshal.SizeOf(param_request_list);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    param_request_list = (mavlink_param_request_list_t)Marshal.PtrToStructure(i, ((object)param_request_list).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
