// MESSAGE MISSION_ACK PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MISSION_ACK = 47;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_mission_ack_t
    {
        /// <summary>
        /// System ID
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component ID
        /// </summary>
        public  byte target_component;
            /// <summary>
        /// See MAV_MISSION_RESULT enum
        /// </summary>
        public  byte type;
    
    };

/// <summary>
/// * @brief Pack a mission_ack message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param type See MAV_MISSION_RESULT enum
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_mission_ack_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, byte type)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(type),0,msg,2,sizeof(byte));

} else {
    mavlink_mission_ack_t packet = new mavlink_mission_ack_t();
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.type = type;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MISSION_ACK;
    //return mavlink_finalize_message(msg, system_id, component_id, 3, 153);
    return 0;
}

/**
 * @brief Pack a mission_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_mission_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, type);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_mission_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.type = type;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 153);
}
*/
/**
 * @brief Encode a mission_ack struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_ack C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_mission_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_ack_t* mission_ack)
{
    return mavlink_msg_mission_ack_pack(system_id, component_id, msg, mission_ack->target_system, mission_ack->target_component, mission_ack->type);
}
*/
/**
 * @brief Send a mission_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param type See MAV_MISSION_RESULT enum
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_ack_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, buf, 3, 153);
#else
    mavlink_mission_ack_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.type = type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ACK, (const char *)&packet, 3, 153);
#endif
}

#endif
*/
// MESSAGE MISSION_ACK UNPACKING


/**
 * @brief Get field target_system from mission_ack message
 *
 * @return System ID
 */
public static byte mavlink_msg_mission_ack_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from mission_ack message
 *
 * @return Component ID
 */
public static byte mavlink_msg_mission_ack_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field type from mission_ack message
 *
 * @return See MAV_MISSION_RESULT enum
 */
public static byte mavlink_msg_mission_ack_get_type(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Decode a mission_ack message into a struct
 *
 * @param msg The message to decode
 * @param mission_ack C-struct to decode the message contents into
 */
public static void mavlink_msg_mission_ack_decode(byte[] msg, ref mavlink_mission_ack_t mission_ack)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	mission_ack.target_system = mavlink_msg_mission_ack_get_target_system(msg);
    	mission_ack.target_component = mavlink_msg_mission_ack_get_target_component(msg);
    	mission_ack.type = mavlink_msg_mission_ack_get_type(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(mission_ack);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        mission_ack = (mavlink_mission_ack_t)Marshal.PtrToStructure(i, ((object)mission_ack).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
