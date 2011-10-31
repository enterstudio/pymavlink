// MESSAGE WATCHDOG_COMMAND PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_WATCHDOG_COMMAND = 183;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_watchdog_command_t
    {
        /// <summary>
        /// Watchdog ID
        /// </summary>
        public  UInt16 watchdog_id;
            /// <summary>
        /// Process ID
        /// </summary>
        public  UInt16 process_id;
            /// <summary>
        /// Target system ID
        /// </summary>
        public  byte target_system_id;
            /// <summary>
        /// Command ID
        /// </summary>
        public  byte command_id;
    
    };

/// <summary>
/// * @brief Pack a watchdog_command message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system_id Target system ID
/// * @param watchdog_id Watchdog ID
/// * @param process_id Process ID
/// * @param command_id Command ID
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_watchdog_command_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system_id, UInt16 watchdog_id, UInt16 process_id, byte command_id)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(watchdog_id),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(process_id),0,msg,2,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(target_system_id),0,msg,4,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(command_id),0,msg,5,sizeof(byte));

} else {
    mavlink_watchdog_command_t packet = new mavlink_watchdog_command_t();
	packet.watchdog_id = watchdog_id;
	packet.process_id = process_id;
	packet.target_system_id = target_system_id;
	packet.command_id = command_id;

        
        int len = 6;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_WATCHDOG_COMMAND;
    //return mavlink_finalize_message(msg, system_id, component_id, 6, 162);
    return 0;
}

/**
 * @brief Pack a watchdog_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system_id Target system ID
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param command_id Command ID
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_watchdog_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system_id,UInt16 public watchdog_id,UInt16 public process_id,byte public command_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[6];
	_mav_put_UInt16(buf, 0, watchdog_id);
	_mav_put_UInt16(buf, 2, process_id);
	_mav_put_byte(buf, 4, target_system_id);
	_mav_put_byte(buf, 5, command_id);

        memcpy(_MAV_PAYLOAD(msg), buf, 6);
#else
    mavlink_watchdog_command_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_id = process_id;
	packet.target_system_id = target_system_id;
	packet.command_id = command_id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 6);
#endif

    msg->msgid = MAVLINK_MSG_ID_WATCHDOG_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 162);
}
*/
/**
 * @brief Encode a watchdog_command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param watchdog_command C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_watchdog_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_watchdog_command_t* watchdog_command)
{
    return mavlink_msg_watchdog_command_pack(system_id, component_id, msg, watchdog_command->target_system_id, watchdog_command->watchdog_id, watchdog_command->process_id, watchdog_command->command_id);
}
*/
/**
 * @brief Send a watchdog_command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system_id Target system ID
 * @param watchdog_id Watchdog ID
 * @param process_id Process ID
 * @param command_id Command ID
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_watchdog_command_send(mavlink_channel_t chan, byte public target_system_id, UInt16 public watchdog_id, UInt16 public process_id, byte public command_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[6];
	_mav_put_UInt16(buf, 0, watchdog_id);
	_mav_put_UInt16(buf, 2, process_id);
	_mav_put_byte(buf, 4, target_system_id);
	_mav_put_byte(buf, 5, command_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_COMMAND, buf, 6, 162);
#else
    mavlink_watchdog_command_t packet;
	packet.watchdog_id = watchdog_id;
	packet.process_id = process_id;
	packet.target_system_id = target_system_id;
	packet.command_id = command_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATCHDOG_COMMAND, (const char *)&packet, 6, 162);
#endif
}

#endif
*/
// MESSAGE WATCHDOG_COMMAND UNPACKING


/**
 * @brief Get field target_system_id from watchdog_command message
 *
 * @return Target system ID
 */
public static byte mavlink_msg_watchdog_command_get_target_system_id(byte[] msg)
{
    return getByte(msg,  4);
}

/**
 * @brief Get field watchdog_id from watchdog_command message
 *
 * @return Watchdog ID
 */
public static UInt16 mavlink_msg_watchdog_command_get_watchdog_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field process_id from watchdog_command message
 *
 * @return Process ID
 */
public static UInt16 mavlink_msg_watchdog_command_get_process_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field command_id from watchdog_command message
 *
 * @return Command ID
 */
public static byte mavlink_msg_watchdog_command_get_command_id(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Decode a watchdog_command message into a struct
 *
 * @param msg The message to decode
 * @param watchdog_command C-struct to decode the message contents into
 */
public static void mavlink_msg_watchdog_command_decode(byte[] msg, ref mavlink_watchdog_command_t watchdog_command)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	watchdog_command.watchdog_id = mavlink_msg_watchdog_command_get_watchdog_id(msg);
    	watchdog_command.process_id = mavlink_msg_watchdog_command_get_process_id(msg);
    	watchdog_command.target_system_id = mavlink_msg_watchdog_command_get_target_system_id(msg);
    	watchdog_command.command_id = mavlink_msg_watchdog_command_get_command_id(msg);
    
    } else {
        int len = 6; //Marshal.SizeOf(watchdog_command);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        watchdog_command = (mavlink_watchdog_command_t)Marshal.PtrToStructure(i, ((object)watchdog_command).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
