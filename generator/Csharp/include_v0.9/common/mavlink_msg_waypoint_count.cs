// MESSAGE WAYPOINT_COUNT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_WAYPOINT_COUNT = 44;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_waypoint_count_t
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
        /// Number of Waypoints in the Sequence
        /// </summary>
        public  UInt16 count;
    
    };

/// <summary>
/// * @brief Pack a waypoint_count message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param count Number of Waypoints in the Sequence
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_waypoint_count_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, UInt16 count)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(count),0,msg,2,sizeof(UInt16));

} else {
    mavlink_waypoint_count_t packet = new mavlink_waypoint_count_t();
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.count = count;

        
        int len = 4;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_WAYPOINT_COUNT;
    //return mavlink_finalize_message(msg, system_id, component_id, 4);
    return 0;
}

/**
 * @brief Pack a waypoint_count message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param count Number of Waypoints in the Sequence
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_waypoint_count_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,UInt16 public count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, count);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_waypoint_count_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.count = count;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_WAYPOINT_COUNT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}
*/
/**
 * @brief Encode a waypoint_count struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_count C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_waypoint_count_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_count_t* waypoint_count)
{
    return mavlink_msg_waypoint_count_pack(system_id, component_id, msg, waypoint_count->target_system, waypoint_count->target_component, waypoint_count->count);
}
*/
/**
 * @brief Send a waypoint_count message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param count Number of Waypoints in the Sequence
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_count_send(mavlink_channel_t chan, byte public target_system, byte public target_component, UInt16 public count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_COUNT, buf, 4);
#else
    mavlink_waypoint_count_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.count = count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_COUNT, (const char *)&packet, 4);
#endif
}

#endif
*/
// MESSAGE WAYPOINT_COUNT UNPACKING


/**
 * @brief Get field target_system from waypoint_count message
 *
 * @return System ID
 */
public static byte mavlink_msg_waypoint_count_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from waypoint_count message
 *
 * @return Component ID
 */
public static byte mavlink_msg_waypoint_count_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field count from waypoint_count message
 *
 * @return Number of Waypoints in the Sequence
 */
public static UInt16 mavlink_msg_waypoint_count_get_count(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Decode a waypoint_count message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_count C-struct to decode the message contents into
 */
public static void mavlink_msg_waypoint_count_decode(byte[] msg, ref mavlink_waypoint_count_t waypoint_count)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	waypoint_count.target_system = mavlink_msg_waypoint_count_get_target_system(msg);
    	waypoint_count.target_component = mavlink_msg_waypoint_count_get_target_component(msg);
    	waypoint_count.count = mavlink_msg_waypoint_count_get_count(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(waypoint_count);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        waypoint_count = (mavlink_waypoint_count_t)Marshal.PtrToStructure(i, ((object)waypoint_count).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
