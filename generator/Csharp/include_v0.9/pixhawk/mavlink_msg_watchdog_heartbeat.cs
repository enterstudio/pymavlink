// MESSAGE WATCHDOG_HEARTBEAT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT = 180;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_watchdog_heartbeat_t
    {
        /// <summary>
        /// Watchdog ID
        /// </summary>
        public  UInt16 watchdog_id;
            /// <summary>
        /// Number of processes
        /// </summary>
        public  UInt16 process_count;
    
    };

/// <summary>
/// * @brief Pack a watchdog_heartbeat message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param watchdog_id Watchdog ID
/// * @param process_count Number of processes
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_watchdog_heartbeat_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 watchdog_id, UInt16 process_count)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(watchdog_id),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(process_count),0,msg,2,sizeof(UInt16));

} else {
    mavlink_watchdog_heartbeat_t packet = new mavlink_watchdog_heartbeat_t();
	packet.watchdog_id = watchdog_id;
	packet.process_count = process_count;

        
        int len = 4;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT;
    //return mavlink_finalize_message(msg, system_id, component_id, 4);
    return 0;
}

/**
 * @brief Pack a watchdog_heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_id Watchdog ID
 * @param process_count Number of processes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_watchdog_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public watchdog_id,UInt16 public process_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, watchdog_id);
	_mav_put_UInt16(buf, 2, process_count);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_watchdog_heartbeat_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_count = process_count;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}
*/
/**
 * @brief Encode a watchdog_heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_heartbeat C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_watchdog_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_watchdog_heartbeat_t* watchdog_heartbeat)
{
    return mavlink_msg_watchdog_heartbeat_pack(system_id, component_id, msg, watchdog_heartbeat->watchdog_id, watchdog_heartbeat->process_count);
}
*/
/**
 * @brief Send a watchdog_heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param watchdog_id Watchdog ID
 * @param process_count Number of processes
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_watchdog_heartbeat_send(mavlink_channel_t chan, UInt16 public watchdog_id, UInt16 public process_count)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, watchdog_id);
	_mav_put_UInt16(buf, 2, process_count);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT, buf, 4);
#else
    mavlink_watchdog_heartbeat_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_count = process_count;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_HEARTBEAT, (const char *)&packet, 4);
#endif
}

#endif
*/
// MESSAGE WATCHDOG_HEARTBEAT UNPACKING


/**
 * @brief Get field watchdog_id from watchdog_heartbeat message
 *
 * @return Watchdog ID
 */
public static UInt16 mavlink_msg_watchdog_heartbeat_get_watchdog_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field process_count from watchdog_heartbeat message
 *
 * @return Number of processes
 */
public static UInt16 mavlink_msg_watchdog_heartbeat_get_process_count(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Decode a watchdog_heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param watchdog_heartbeat C-struct to decode the message contents into
 */
public static void mavlink_msg_watchdog_heartbeat_decode(byte[] msg, ref mavlink_watchdog_heartbeat_t watchdog_heartbeat)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	watchdog_heartbeat.watchdog_id = mavlink_msg_watchdog_heartbeat_get_watchdog_id(msg);
    	watchdog_heartbeat.process_count = mavlink_msg_watchdog_heartbeat_get_process_count(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(watchdog_heartbeat);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        watchdog_heartbeat = (mavlink_watchdog_heartbeat_t)Marshal.PtrToStructure(i, ((object)watchdog_heartbeat).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
