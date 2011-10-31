// MESSAGE RC_CHANNELS_OVERRIDE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE = 70;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_rc_channels_override_t
    {
        /// <summary>
        /// RC channel 1 value, in microseconds
        /// </summary>
        public  UInt16 chan1_raw;
            /// <summary>
        /// RC channel 2 value, in microseconds
        /// </summary>
        public  UInt16 chan2_raw;
            /// <summary>
        /// RC channel 3 value, in microseconds
        /// </summary>
        public  UInt16 chan3_raw;
            /// <summary>
        /// RC channel 4 value, in microseconds
        /// </summary>
        public  UInt16 chan4_raw;
            /// <summary>
        /// RC channel 5 value, in microseconds
        /// </summary>
        public  UInt16 chan5_raw;
            /// <summary>
        /// RC channel 6 value, in microseconds
        /// </summary>
        public  UInt16 chan6_raw;
            /// <summary>
        /// RC channel 7 value, in microseconds
        /// </summary>
        public  UInt16 chan7_raw;
            /// <summary>
        /// RC channel 8 value, in microseconds
        /// </summary>
        public  UInt16 chan8_raw;
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
/// * @brief Pack a rc_channels_override message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param chan1_raw RC channel 1 value, in microseconds
/// * @param chan2_raw RC channel 2 value, in microseconds
/// * @param chan3_raw RC channel 3 value, in microseconds
/// * @param chan4_raw RC channel 4 value, in microseconds
/// * @param chan5_raw RC channel 5 value, in microseconds
/// * @param chan6_raw RC channel 6 value, in microseconds
/// * @param chan7_raw RC channel 7 value, in microseconds
/// * @param chan8_raw RC channel 8 value, in microseconds
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_rc_channels_override_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, UInt16 chan1_raw, UInt16 chan2_raw, UInt16 chan3_raw, UInt16 chan4_raw, UInt16 chan5_raw, UInt16 chan6_raw, UInt16 chan7_raw, UInt16 chan8_raw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(chan1_raw),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan2_raw),0,msg,2,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan3_raw),0,msg,4,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan4_raw),0,msg,6,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan5_raw),0,msg,8,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan6_raw),0,msg,10,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan7_raw),0,msg,12,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan8_raw),0,msg,14,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,16,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,17,sizeof(byte));

} else {
    mavlink_rc_channels_override_t packet = new mavlink_rc_channels_override_t();
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        
        int len = 18;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
    //return mavlink_finalize_message(msg, system_id, component_id, 18, 124);
    return 0;
}

/**
 * @brief Pack a rc_channels_override message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_rc_channels_override_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,UInt16 public chan1_raw,UInt16 public chan2_raw,UInt16 public chan3_raw,UInt16 public chan4_raw,UInt16 public chan5_raw,UInt16 public chan6_raw,UInt16 public chan7_raw,UInt16 public chan8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt16(buf, 0, chan1_raw);
	_mav_put_UInt16(buf, 2, chan2_raw);
	_mav_put_UInt16(buf, 4, chan3_raw);
	_mav_put_UInt16(buf, 6, chan4_raw);
	_mav_put_UInt16(buf, 8, chan5_raw);
	_mav_put_UInt16(buf, 10, chan6_raw);
	_mav_put_UInt16(buf, 12, chan7_raw);
	_mav_put_UInt16(buf, 14, chan8_raw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_rc_channels_override_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 124);
}
*/
/**
 * @brief Encode a rc_channels_override struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_override C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_rc_channels_override_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_override_t* rc_channels_override)
{
    return mavlink_msg_rc_channels_override_pack(system_id, component_id, msg, rc_channels_override->target_system, rc_channels_override->target_component, rc_channels_override->chan1_raw, rc_channels_override->chan2_raw, rc_channels_override->chan3_raw, rc_channels_override->chan4_raw, rc_channels_override->chan5_raw, rc_channels_override->chan6_raw, rc_channels_override->chan7_raw, rc_channels_override->chan8_raw);
}
*/
/**
 * @brief Send a rc_channels_override message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_override_send(mavlink_channel_t chan, byte public target_system, byte public target_component, UInt16 public chan1_raw, UInt16 public chan2_raw, UInt16 public chan3_raw, UInt16 public chan4_raw, UInt16 public chan5_raw, UInt16 public chan6_raw, UInt16 public chan7_raw, UInt16 public chan8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt16(buf, 0, chan1_raw);
	_mav_put_UInt16(buf, 2, chan2_raw);
	_mav_put_UInt16(buf, 4, chan3_raw);
	_mav_put_UInt16(buf, 6, chan4_raw);
	_mav_put_UInt16(buf, 8, chan5_raw);
	_mav_put_UInt16(buf, 10, chan6_raw);
	_mav_put_UInt16(buf, 12, chan7_raw);
	_mav_put_UInt16(buf, 14, chan8_raw);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, buf, 18, 124);
#else
    mavlink_rc_channels_override_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, (const char *)&packet, 18, 124);
#endif
}

#endif
*/
// MESSAGE RC_CHANNELS_OVERRIDE UNPACKING


/**
 * @brief Get field target_system from rc_channels_override message
 *
 * @return System ID
 */
public static byte mavlink_msg_rc_channels_override_get_target_system(byte[] msg)
{
    return getByte(msg,  16);
}

/**
 * @brief Get field target_component from rc_channels_override message
 *
 * @return Component ID
 */
public static byte mavlink_msg_rc_channels_override_get_target_component(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field chan1_raw from rc_channels_override message
 *
 * @return RC channel 1 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan1_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field chan2_raw from rc_channels_override message
 *
 * @return RC channel 2 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan2_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field chan3_raw from rc_channels_override message
 *
 * @return RC channel 3 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan3_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field chan4_raw from rc_channels_override message
 *
 * @return RC channel 4 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan4_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Get field chan5_raw from rc_channels_override message
 *
 * @return RC channel 5 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan5_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field chan6_raw from rc_channels_override message
 *
 * @return RC channel 6 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan6_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  10);
}

/**
 * @brief Get field chan7_raw from rc_channels_override message
 *
 * @return RC channel 7 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan7_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field chan8_raw from rc_channels_override message
 *
 * @return RC channel 8 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_override_get_chan8_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  14);
}

/**
 * @brief Decode a rc_channels_override message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_override C-struct to decode the message contents into
 */
public static void mavlink_msg_rc_channels_override_decode(byte[] msg, ref mavlink_rc_channels_override_t rc_channels_override)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	rc_channels_override.chan1_raw = mavlink_msg_rc_channels_override_get_chan1_raw(msg);
    	rc_channels_override.chan2_raw = mavlink_msg_rc_channels_override_get_chan2_raw(msg);
    	rc_channels_override.chan3_raw = mavlink_msg_rc_channels_override_get_chan3_raw(msg);
    	rc_channels_override.chan4_raw = mavlink_msg_rc_channels_override_get_chan4_raw(msg);
    	rc_channels_override.chan5_raw = mavlink_msg_rc_channels_override_get_chan5_raw(msg);
    	rc_channels_override.chan6_raw = mavlink_msg_rc_channels_override_get_chan6_raw(msg);
    	rc_channels_override.chan7_raw = mavlink_msg_rc_channels_override_get_chan7_raw(msg);
    	rc_channels_override.chan8_raw = mavlink_msg_rc_channels_override_get_chan8_raw(msg);
    	rc_channels_override.target_system = mavlink_msg_rc_channels_override_get_target_system(msg);
    	rc_channels_override.target_component = mavlink_msg_rc_channels_override_get_target_component(msg);
    
    } else {
        int len = 18; //Marshal.SizeOf(rc_channels_override);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        rc_channels_override = (mavlink_rc_channels_override_t)Marshal.PtrToStructure(i, ((object)rc_channels_override).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}