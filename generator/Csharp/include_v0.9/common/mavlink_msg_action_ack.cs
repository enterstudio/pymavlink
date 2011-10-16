// MESSAGE ACTION_ACK PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ACTION_ACK = 9;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_action_ack_t
    {
         public  byte action; /// The action id
     public  byte result; /// 0: Action DENIED, 1: Action executed
    
    };

/**
 * @brief Pack a action_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param action The action id
 * @param result 0: Action DENIED, 1: Action executed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_action_ack_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public action, byte public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[2];
	_mav_put_byte(buf, 0, action);
	_mav_put_byte(buf, 1, result);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_action_ack_t packet;
	packet.action = action;
	packet.result = result;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTION_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, 2);
}
*/
/**
 * @brief Pack a action_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param action The action id
 * @param result 0: Action DENIED, 1: Action executed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_action_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public action,byte public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, action);
	_mav_put_byte(buf, 1, result);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_action_ack_t packet;
	packet.action = action;
	packet.result = result;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTION_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2);
}
*/
/**
 * @brief Encode a action_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param action_ack C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_action_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_action_ack_t* action_ack)
{
    return mavlink_msg_action_ack_pack(system_id, component_id, msg, action_ack->action, action_ack->result);
}
*/
/**
 * @brief Send a action_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param action The action id
 * @param result 0: Action DENIED, 1: Action executed
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_action_ack_send(mavlink_channel_t chan, byte public action, byte public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, action);
	_mav_put_byte(buf, 1, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTION_ACK, buf, 2);
#else
    mavlink_action_ack_t packet;
	packet.action = action;
	packet.result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTION_ACK, (const char *)&packet, 2);
#endif
}

#endif
*/
// MESSAGE ACTION_ACK UNPACKING


/**
 * @brief Get field action from action_ack message
 *
 * @return The action id
 */
public static byte mavlink_msg_action_ack_get_action(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field result from action_ack message
 *
 * @return 0: Action DENIED, 1: Action executed
 */
public static byte mavlink_msg_action_ack_get_result(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Decode a action_ack message into a struct
 *
 * @param msg The message to decode
 * @param action_ack C-struct to decode the message contents into
 */
public static void mavlink_msg_action_ack_decode(byte[] msg, ref mavlink_action_ack_t action_ack)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	action_ack.action = mavlink_msg_action_ack_get_action(msg);
	action_ack.result = mavlink_msg_action_ack_get_result(msg);
} else {
    int len = 2; //Marshal.SizeOf(action_ack);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    action_ack = (mavlink_action_ack_t)Marshal.PtrToStructure(i, ((object)action_ack).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
