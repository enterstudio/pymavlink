// MESSAGE BOOT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_BOOT = 1;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_boot_t
    {
        /// <summary>
        /// The onboard software version
        /// </summary>
        public  UInt32 version;
    
    };

/// <summary>
/// * @brief Pack a boot message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param version The onboard software version
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_boot_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 version)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(version),0,msg,0,sizeof(UInt32));

} else {
    mavlink_boot_t packet = new mavlink_boot_t();
	packet.version = version;

        
        int len = 4;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_BOOT;
    //return mavlink_finalize_message(msg, system_id, component_id, 4);
    return 0;
}

/**
 * @brief Pack a boot message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_boot_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt32(buf, 0, version);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_boot_t packet;
	packet.version = version;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_BOOT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}
*/
/**
 * @brief Encode a boot struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param boot C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_boot_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boot_t* boot)
{
    return mavlink_msg_boot_pack(system_id, component_id, msg, boot->version);
}
*/
/**
 * @brief Send a boot message
 * @param chan MAVLink channel to send the message
 *
 * @param version The onboard software version
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_boot_send(mavlink_channel_t chan, UInt32 public version)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt32(buf, 0, version);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, buf, 4);
#else
    mavlink_boot_t packet;
	packet.version = version;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BOOT, (const char *)&packet, 4);
#endif
}

#endif
*/
// MESSAGE BOOT UNPACKING


/**
 * @brief Get field version from boot message
 *
 * @return The onboard software version
 */
public static UInt32 mavlink_msg_boot_get_version(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Decode a boot message into a struct
 *
 * @param msg The message to decode
 * @param boot C-struct to decode the message contents into
 */
public static void mavlink_msg_boot_decode(byte[] msg, ref mavlink_boot_t boot)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	boot.version = mavlink_msg_boot_get_version(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(boot);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        boot = (mavlink_boot_t)Marshal.PtrToStructure(i, ((object)boot).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
