// MESSAGE COMMAND_ACK PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_COMMAND_ACK = 77;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_command_ack_t
    {
         public  Single command; /// Current airspeed in m/s
     public  Single result; /// 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
    
    };

/**
 * @brief Pack a command_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command Current airspeed in m/s
 * @param result 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_command_ack_pack(byte system_id, byte component_id, ref byte[] msg,
                               Single public command, Single public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[8];
	_mav_put_Single(buf, 0, command);
	_mav_put_Single(buf, 4, result);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_command_ack_t packet;
	packet.command = command;
	packet.result = result;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, 8, 8);
}
*/
/**
 * @brief Pack a command_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Current airspeed in m/s
 * @param result 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_command_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public command,Single public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_Single(buf, 0, command);
	_mav_put_Single(buf, 4, result);

        memcpy(_MAV_PAYLOAD(msg), buf, 8);
#else
    mavlink_command_ack_t packet;
	packet.command = command;
	packet.result = result;

        memcpy(_MAV_PAYLOAD(msg), &packet, 8);
#endif

    msg->msgid = MAVLINK_MSG_ID_COMMAND_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 8);
}
*/
/**
 * @brief Encode a command_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param command_ack C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_command_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_command_ack_t* command_ack)
{
    return mavlink_msg_command_ack_pack(system_id, component_id, msg, command_ack->command, command_ack->result);
}
*/
/**
 * @brief Send a command_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param command Current airspeed in m/s
 * @param result 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_command_ack_send(mavlink_channel_t chan, Single public command, Single public result)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[8];
	_mav_put_Single(buf, 0, command);
	_mav_put_Single(buf, 4, result);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_ACK, buf, 8, 8);
#else
    mavlink_command_ack_t packet;
	packet.command = command;
	packet.result = result;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COMMAND_ACK, (const char *)&packet, 8, 8);
#endif
}

#endif
*/
// MESSAGE COMMAND_ACK UNPACKING


/**
 * @brief Get field command from command_ack message
 *
 * @return Current airspeed in m/s
 */
public static Single mavlink_msg_command_ack_get_command(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field result from command_ack message
 *
 * @return 1: Action ACCEPTED and EXECUTED, 1: Action TEMPORARY REJECTED/DENIED, 2: Action PERMANENTLY DENIED, 3: Action UNKNOWN/UNSUPPORTED, 4: Requesting CONFIRMATION
 */
public static Single mavlink_msg_command_ack_get_result(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Decode a command_ack message into a struct
 *
 * @param msg The message to decode
 * @param command_ack C-struct to decode the message contents into
 */
public static void mavlink_msg_command_ack_decode(byte[] msg, ref mavlink_command_ack_t command_ack)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	command_ack.command = mavlink_msg_command_ack_get_command(msg);
	command_ack.result = mavlink_msg_command_ack_get_result(msg);
} else {
    int len = 8; //Marshal.SizeOf(command_ack);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    command_ack = (mavlink_command_ack_t)Marshal.PtrToStructure(i, ((object)command_ack).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
