// MESSAGE RC_CHANNELS_RAW PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_rc_channels_raw_t
    {
         public  UInt16 chan1_raw; /// RC channel 1 value, in microseconds
     public  UInt16 chan2_raw; /// RC channel 2 value, in microseconds
     public  UInt16 chan3_raw; /// RC channel 3 value, in microseconds
     public  UInt16 chan4_raw; /// RC channel 4 value, in microseconds
     public  UInt16 chan5_raw; /// RC channel 5 value, in microseconds
     public  UInt16 chan6_raw; /// RC channel 6 value, in microseconds
     public  UInt16 chan7_raw; /// RC channel 7 value, in microseconds
     public  UInt16 chan8_raw; /// RC channel 8 value, in microseconds
     public  byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
    
    };

/**
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_rc_channels_raw_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt16 public chan1_raw, UInt16 public chan2_raw, UInt16 public chan3_raw, UInt16 public chan4_raw, UInt16 public chan5_raw, UInt16 public chan6_raw, UInt16 public chan7_raw, UInt16 public chan8_raw, byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[17];
	_mav_put_UInt16(buf, 0, chan1_raw);
	_mav_put_UInt16(buf, 2, chan2_raw);
	_mav_put_UInt16(buf, 4, chan3_raw);
	_mav_put_UInt16(buf, 6, chan4_raw);
	_mav_put_UInt16(buf, 8, chan5_raw);
	_mav_put_UInt16(buf, 10, chan6_raw);
	_mav_put_UInt16(buf, 12, chan7_raw);
	_mav_put_UInt16(buf, 14, chan8_raw);
	_mav_put_byte(buf, 16, rssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 17);
#else
    mavlink_rc_channels_raw_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 17);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, 17);
}
*/
/**
 * @brief Pack a rc_channels_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_rc_channels_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public chan1_raw,UInt16 public chan2_raw,UInt16 public chan3_raw,UInt16 public chan4_raw,UInt16 public chan5_raw,UInt16 public chan6_raw,UInt16 public chan7_raw,UInt16 public chan8_raw,byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[17];
	_mav_put_UInt16(buf, 0, chan1_raw);
	_mav_put_UInt16(buf, 2, chan2_raw);
	_mav_put_UInt16(buf, 4, chan3_raw);
	_mav_put_UInt16(buf, 6, chan4_raw);
	_mav_put_UInt16(buf, 8, chan5_raw);
	_mav_put_UInt16(buf, 10, chan6_raw);
	_mav_put_UInt16(buf, 12, chan7_raw);
	_mav_put_UInt16(buf, 14, chan8_raw);
	_mav_put_byte(buf, 16, rssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 17);
#else
    mavlink_rc_channels_raw_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 17);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 17);
}
*/
/**
 * @brief Encode a rc_channels_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_raw C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_rc_channels_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_raw_t* rc_channels_raw)
{
    return mavlink_msg_rc_channels_raw_pack(system_id, component_id, msg, rc_channels_raw->chan1_raw, rc_channels_raw->chan2_raw, rc_channels_raw->chan3_raw, rc_channels_raw->chan4_raw, rc_channels_raw->chan5_raw, rc_channels_raw->chan6_raw, rc_channels_raw->chan7_raw, rc_channels_raw->chan8_raw, rc_channels_raw->rssi);
}
*/
/**
 * @brief Send a rc_channels_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_raw_send(mavlink_channel_t chan, UInt16 public chan1_raw, UInt16 public chan2_raw, UInt16 public chan3_raw, UInt16 public chan4_raw, UInt16 public chan5_raw, UInt16 public chan6_raw, UInt16 public chan7_raw, UInt16 public chan8_raw, byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[17];
	_mav_put_UInt16(buf, 0, chan1_raw);
	_mav_put_UInt16(buf, 2, chan2_raw);
	_mav_put_UInt16(buf, 4, chan3_raw);
	_mav_put_UInt16(buf, 6, chan4_raw);
	_mav_put_UInt16(buf, 8, chan5_raw);
	_mav_put_UInt16(buf, 10, chan6_raw);
	_mav_put_UInt16(buf, 12, chan7_raw);
	_mav_put_UInt16(buf, 14, chan8_raw);
	_mav_put_byte(buf, 16, rssi);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, buf, 17);
#else
    mavlink_rc_channels_raw_t packet;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.rssi = rssi;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_RAW, (const char *)&packet, 17);
#endif
}

#endif
*/
// MESSAGE RC_CHANNELS_RAW UNPACKING


/**
 * @brief Get field chan1_raw from rc_channels_raw message
 *
 * @return RC channel 1 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan1_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field chan2_raw from rc_channels_raw message
 *
 * @return RC channel 2 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan2_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field chan3_raw from rc_channels_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan3_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field chan4_raw from rc_channels_raw message
 *
 * @return RC channel 4 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan4_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Get field chan5_raw from rc_channels_raw message
 *
 * @return RC channel 5 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan5_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field chan6_raw from rc_channels_raw message
 *
 * @return RC channel 6 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan6_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  10);
}

/**
 * @brief Get field chan7_raw from rc_channels_raw message
 *
 * @return RC channel 7 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan7_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field chan8_raw from rc_channels_raw message
 *
 * @return RC channel 8 value, in microseconds
 */
public static UInt16 mavlink_msg_rc_channels_raw_get_chan8_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  14);
}

/**
 * @brief Get field rssi from rc_channels_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
public static byte mavlink_msg_rc_channels_raw_get_rssi(byte[] msg)
{
    return getByte(msg,  16);
}

/**
 * @brief Decode a rc_channels_raw message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_raw C-struct to decode the message contents into
 */
public static void mavlink_msg_rc_channels_raw_decode(byte[] msg, ref mavlink_rc_channels_raw_t rc_channels_raw)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	rc_channels_raw.chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(msg);
	rc_channels_raw.chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(msg);
	rc_channels_raw.chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(msg);
	rc_channels_raw.chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(msg);
	rc_channels_raw.chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(msg);
	rc_channels_raw.chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(msg);
	rc_channels_raw.chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(msg);
	rc_channels_raw.chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(msg);
	rc_channels_raw.rssi = mavlink_msg_rc_channels_raw_get_rssi(msg);
} else {
    int len = 17; //Marshal.SizeOf(rc_channels_raw);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    rc_channels_raw = (mavlink_rc_channels_raw_t)Marshal.PtrToStructure(i, ((object)rc_channels_raw).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
