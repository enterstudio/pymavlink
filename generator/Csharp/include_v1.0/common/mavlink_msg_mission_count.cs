// MESSAGE MISSION_COUNT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MISSION_COUNT = 44;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_mission_count_t
    {
        /// <summary>
        /// Number of MISSIONs in the Sequence
        /// </summary>
        public  UInt16 count;
            /// <summary>
        /// System ID
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component ID
        /// </summary>
        public  byte target_component;
    
    };

/// <summary>
/// * @brief Pack a mission_count message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param count Number of MISSIONs in the Sequence
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_mission_count_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, UInt16 count)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(count),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,3,sizeof(byte));

} else {
    mavlink_mission_count_t packet = new mavlink_mission_count_t();
	packet.count = count;
	packet.target_system = target_system;
	packet.target_component = target_component;

        
        int len = 4;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MISSION_COUNT;
    //return mavlink_finalize_message(msg, system_id, component_id, 4, 221);
    return 0;
}

/**
 * @brief Pack a mission_count message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param count Number of MISSIONs in the Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_mission_count_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,UInt16 public count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, count);
	_mav_put_byte(buf, 2, target_system);
	_mav_put_byte(buf, 3, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_mission_count_t packet;
	packet.count = count;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_COUNT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 221);
}
*/
/**
 * @brief Encode a mission_count struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_count C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_mission_count_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_count_t* mission_count)
{
    return mavlink_msg_mission_count_pack(system_id, component_id, msg, mission_count->target_system, mission_count->target_component, mission_count->count);
}
*/
/**
 * @brief Send a mission_count message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param count Number of MISSIONs in the Sequence
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_count_send(mavlink_channel_t chan, byte public target_system, byte public target_component, UInt16 public count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, count);
	_mav_put_byte(buf, 2, target_system);
	_mav_put_byte(buf, 3, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_COUNT, buf, 4, 221);
#else
    mavlink_mission_count_t packet;
	packet.count = count;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_COUNT, (const char *)&packet, 4, 221);
#endif
}

#endif
*/
// MESSAGE MISSION_COUNT UNPACKING


/**
 * @brief Get field target_system from mission_count message
 *
 * @return System ID
 */
public static byte mavlink_msg_mission_count_get_target_system(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field target_component from mission_count message
 *
 * @return Component ID
 */
public static byte mavlink_msg_mission_count_get_target_component(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field count from mission_count message
 *
 * @return Number of MISSIONs in the Sequence
 */
public static UInt16 mavlink_msg_mission_count_get_count(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Decode a mission_count message into a struct
 *
 * @param msg The message to decode
 * @param mission_count C-struct to decode the message contents into
 */
public static void mavlink_msg_mission_count_decode(byte[] msg, ref mavlink_mission_count_t mission_count)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	mission_count.count = mavlink_msg_mission_count_get_count(msg);
    	mission_count.target_system = mavlink_msg_mission_count_get_target_system(msg);
    	mission_count.target_component = mavlink_msg_mission_count_get_target_component(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(mission_count);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        mission_count = (mavlink_mission_count_t)Marshal.PtrToStructure(i, ((object)mission_count).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
