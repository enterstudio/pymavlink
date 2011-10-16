// MESSAGE MISSION_CLEAR_ALL PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MISSION_CLEAR_ALL = 45;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_mission_clear_all_t
    {
         public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
    
    };

/**
 * @brief Pack a mission_clear_all message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_mission_clear_all_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target_system, byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[2];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_mission_clear_all_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
    return mavlink_finalize_message(msg, system_id, component_id, 2, 232);
}
*/
/**
 * @brief Pack a mission_clear_all message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_mission_clear_all_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 2);
#else
    mavlink_mission_clear_all_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 2);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 232);
}
*/
/**
 * @brief Encode a mission_clear_all struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_clear_all C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_mission_clear_all_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_clear_all_t* mission_clear_all)
{
    return mavlink_msg_mission_clear_all_pack(system_id, component_id, msg, mission_clear_all->target_system, mission_clear_all->target_component);
}
*/
/**
 * @brief Send a mission_clear_all message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_clear_all_send(mavlink_channel_t chan, byte public target_system, byte public target_component)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[2];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, buf, 2, 232);
#else
    mavlink_mission_clear_all_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, (const char *)&packet, 2, 232);
#endif
}

#endif
*/
// MESSAGE MISSION_CLEAR_ALL UNPACKING


/**
 * @brief Get field target_system from mission_clear_all message
 *
 * @return System ID
 */
public static byte mavlink_msg_mission_clear_all_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from mission_clear_all message
 *
 * @return Component ID
 */
public static byte mavlink_msg_mission_clear_all_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Decode a mission_clear_all message into a struct
 *
 * @param msg The message to decode
 * @param mission_clear_all C-struct to decode the message contents into
 */
public static void mavlink_msg_mission_clear_all_decode(byte[] msg, ref mavlink_mission_clear_all_t mission_clear_all)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	mission_clear_all.target_system = mavlink_msg_mission_clear_all_get_target_system(msg);
    	mission_clear_all.target_component = mavlink_msg_mission_clear_all_get_target_component(msg);
    
    } else {
        int len = 2; //Marshal.SizeOf(mission_clear_all);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        mission_clear_all = (mavlink_mission_clear_all_t)Marshal.PtrToStructure(i, ((object)mission_clear_all).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
