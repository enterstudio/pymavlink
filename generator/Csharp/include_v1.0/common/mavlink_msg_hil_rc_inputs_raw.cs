// MESSAGE HIL_RC_INPUTS_RAW PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW = 92;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_hil_rc_inputs_raw_t
    {
         public  UInt64 time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     public  UInt16 chan1_raw; /// RC channel 1 value, in microseconds
     public  UInt16 chan2_raw; /// RC channel 2 value, in microseconds
     public  UInt16 chan3_raw; /// RC channel 3 value, in microseconds
     public  UInt16 chan4_raw; /// RC channel 4 value, in microseconds
     public  UInt16 chan5_raw; /// RC channel 5 value, in microseconds
     public  UInt16 chan6_raw; /// RC channel 6 value, in microseconds
     public  UInt16 chan7_raw; /// RC channel 7 value, in microseconds
     public  UInt16 chan8_raw; /// RC channel 8 value, in microseconds
     public  UInt16 chan9_raw; /// RC channel 9 value, in microseconds
     public  UInt16 chan10_raw; /// RC channel 10 value, in microseconds
     public  UInt16 chan11_raw; /// RC channel 11 value, in microseconds
     public  UInt16 chan12_raw; /// RC channel 12 value, in microseconds
     public  byte rssi; /// Receive signal strength indicator, 0: 0%, 255: 100%
    
    };

/**
 * @brief Pack a hil_rc_inputs_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_hil_rc_inputs_raw_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec, UInt16 chan1_raw, UInt16 chan2_raw, UInt16 chan3_raw, UInt16 chan4_raw, UInt16 chan5_raw, UInt16 chan6_raw, UInt16 chan7_raw, UInt16 chan8_raw, UInt16 chan9_raw, UInt16 chan10_raw, UInt16 chan11_raw, UInt16 chan12_raw, byte rssi)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(chan1_raw),0,msg,8,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan2_raw),0,msg,10,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan3_raw),0,msg,12,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan4_raw),0,msg,14,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan5_raw),0,msg,16,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan6_raw),0,msg,18,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan7_raw),0,msg,20,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan8_raw),0,msg,22,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan9_raw),0,msg,24,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan10_raw),0,msg,26,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan11_raw),0,msg,28,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(chan12_raw),0,msg,30,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(rssi),0,msg,32,sizeof(byte));

} else {
    mavlink_hil_rc_inputs_raw_t packet = new mavlink_hil_rc_inputs_raw_t();
	packet.time_usec = time_usec;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.rssi = rssi;

        
        int len = 33;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
    //return mavlink_finalize_message(msg, system_id, component_id, 33, 54);
    return 0;
}

/**
 * @brief Pack a hil_rc_inputs_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec,UInt16 public chan1_raw,UInt16 public chan2_raw,UInt16 public chan3_raw,UInt16 public chan4_raw,UInt16 public chan5_raw,UInt16 public chan6_raw,UInt16 public chan7_raw,UInt16 public chan8_raw,UInt16 public chan9_raw,UInt16 public chan10_raw,UInt16 public chan11_raw,UInt16 public chan12_raw,byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[33];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_UInt16(buf, 8, chan1_raw);
	_mav_put_UInt16(buf, 10, chan2_raw);
	_mav_put_UInt16(buf, 12, chan3_raw);
	_mav_put_UInt16(buf, 14, chan4_raw);
	_mav_put_UInt16(buf, 16, chan5_raw);
	_mav_put_UInt16(buf, 18, chan6_raw);
	_mav_put_UInt16(buf, 20, chan7_raw);
	_mav_put_UInt16(buf, 22, chan8_raw);
	_mav_put_UInt16(buf, 24, chan9_raw);
	_mav_put_UInt16(buf, 26, chan10_raw);
	_mav_put_UInt16(buf, 28, chan11_raw);
	_mav_put_UInt16(buf, 30, chan12_raw);
	_mav_put_byte(buf, 32, rssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 33);
#else
    mavlink_hil_rc_inputs_raw_t packet;
	packet.time_usec = time_usec;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 33);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 33, 54);
}
*/
/**
 * @brief Encode a hil_rc_inputs_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_rc_inputs_raw C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_hil_rc_inputs_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_rc_inputs_raw_t* hil_rc_inputs_raw)
{
    return mavlink_msg_hil_rc_inputs_raw_pack(system_id, component_id, msg, hil_rc_inputs_raw->time_usec, hil_rc_inputs_raw->chan1_raw, hil_rc_inputs_raw->chan2_raw, hil_rc_inputs_raw->chan3_raw, hil_rc_inputs_raw->chan4_raw, hil_rc_inputs_raw->chan5_raw, hil_rc_inputs_raw->chan6_raw, hil_rc_inputs_raw->chan7_raw, hil_rc_inputs_raw->chan8_raw, hil_rc_inputs_raw->chan9_raw, hil_rc_inputs_raw->chan10_raw, hil_rc_inputs_raw->chan11_raw, hil_rc_inputs_raw->chan12_raw, hil_rc_inputs_raw->rssi);
}
*/
/**
 * @brief Send a hil_rc_inputs_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param chan1_raw RC channel 1 value, in microseconds
 * @param chan2_raw RC channel 2 value, in microseconds
 * @param chan3_raw RC channel 3 value, in microseconds
 * @param chan4_raw RC channel 4 value, in microseconds
 * @param chan5_raw RC channel 5 value, in microseconds
 * @param chan6_raw RC channel 6 value, in microseconds
 * @param chan7_raw RC channel 7 value, in microseconds
 * @param chan8_raw RC channel 8 value, in microseconds
 * @param chan9_raw RC channel 9 value, in microseconds
 * @param chan10_raw RC channel 10 value, in microseconds
 * @param chan11_raw RC channel 11 value, in microseconds
 * @param chan12_raw RC channel 12 value, in microseconds
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_rc_inputs_raw_send(mavlink_channel_t chan, UInt64 public time_usec, UInt16 public chan1_raw, UInt16 public chan2_raw, UInt16 public chan3_raw, UInt16 public chan4_raw, UInt16 public chan5_raw, UInt16 public chan6_raw, UInt16 public chan7_raw, UInt16 public chan8_raw, UInt16 public chan9_raw, UInt16 public chan10_raw, UInt16 public chan11_raw, UInt16 public chan12_raw, byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[33];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_UInt16(buf, 8, chan1_raw);
	_mav_put_UInt16(buf, 10, chan2_raw);
	_mav_put_UInt16(buf, 12, chan3_raw);
	_mav_put_UInt16(buf, 14, chan4_raw);
	_mav_put_UInt16(buf, 16, chan5_raw);
	_mav_put_UInt16(buf, 18, chan6_raw);
	_mav_put_UInt16(buf, 20, chan7_raw);
	_mav_put_UInt16(buf, 22, chan8_raw);
	_mav_put_UInt16(buf, 24, chan9_raw);
	_mav_put_UInt16(buf, 26, chan10_raw);
	_mav_put_UInt16(buf, 28, chan11_raw);
	_mav_put_UInt16(buf, 30, chan12_raw);
	_mav_put_byte(buf, 32, rssi);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW, buf, 33, 54);
#else
    mavlink_hil_rc_inputs_raw_t packet;
	packet.time_usec = time_usec;
	packet.chan1_raw = chan1_raw;
	packet.chan2_raw = chan2_raw;
	packet.chan3_raw = chan3_raw;
	packet.chan4_raw = chan4_raw;
	packet.chan5_raw = chan5_raw;
	packet.chan6_raw = chan6_raw;
	packet.chan7_raw = chan7_raw;
	packet.chan8_raw = chan8_raw;
	packet.chan9_raw = chan9_raw;
	packet.chan10_raw = chan10_raw;
	packet.chan11_raw = chan11_raw;
	packet.chan12_raw = chan12_raw;
	packet.rssi = rssi;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW, (const char *)&packet, 33, 54);
#endif
}

#endif
*/
// MESSAGE HIL_RC_INPUTS_RAW UNPACKING


/**
 * @brief Get field time_usec from hil_rc_inputs_raw message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_hil_rc_inputs_raw_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field chan1_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 1 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan1_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field chan2_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 2 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan2_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  10);
}

/**
 * @brief Get field chan3_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 3 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan3_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field chan4_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 4 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan4_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  14);
}

/**
 * @brief Get field chan5_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 5 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan5_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  16);
}

/**
 * @brief Get field chan6_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 6 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan6_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  18);
}

/**
 * @brief Get field chan7_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 7 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan7_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  20);
}

/**
 * @brief Get field chan8_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 8 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan8_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  22);
}

/**
 * @brief Get field chan9_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 9 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan9_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  24);
}

/**
 * @brief Get field chan10_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 10 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan10_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  26);
}

/**
 * @brief Get field chan11_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 11 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan11_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  28);
}

/**
 * @brief Get field chan12_raw from hil_rc_inputs_raw message
 *
 * @return RC channel 12 value, in microseconds
 */
public static UInt16 mavlink_msg_hil_rc_inputs_raw_get_chan12_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  30);
}

/**
 * @brief Get field rssi from hil_rc_inputs_raw message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
public static byte mavlink_msg_hil_rc_inputs_raw_get_rssi(byte[] msg)
{
    return getByte(msg,  32);
}

/**
 * @brief Decode a hil_rc_inputs_raw message into a struct
 *
 * @param msg The message to decode
 * @param hil_rc_inputs_raw C-struct to decode the message contents into
 */
public static void mavlink_msg_hil_rc_inputs_raw_decode(byte[] msg, ref mavlink_hil_rc_inputs_raw_t hil_rc_inputs_raw)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	hil_rc_inputs_raw.time_usec = mavlink_msg_hil_rc_inputs_raw_get_time_usec(msg);
    	hil_rc_inputs_raw.chan1_raw = mavlink_msg_hil_rc_inputs_raw_get_chan1_raw(msg);
    	hil_rc_inputs_raw.chan2_raw = mavlink_msg_hil_rc_inputs_raw_get_chan2_raw(msg);
    	hil_rc_inputs_raw.chan3_raw = mavlink_msg_hil_rc_inputs_raw_get_chan3_raw(msg);
    	hil_rc_inputs_raw.chan4_raw = mavlink_msg_hil_rc_inputs_raw_get_chan4_raw(msg);
    	hil_rc_inputs_raw.chan5_raw = mavlink_msg_hil_rc_inputs_raw_get_chan5_raw(msg);
    	hil_rc_inputs_raw.chan6_raw = mavlink_msg_hil_rc_inputs_raw_get_chan6_raw(msg);
    	hil_rc_inputs_raw.chan7_raw = mavlink_msg_hil_rc_inputs_raw_get_chan7_raw(msg);
    	hil_rc_inputs_raw.chan8_raw = mavlink_msg_hil_rc_inputs_raw_get_chan8_raw(msg);
    	hil_rc_inputs_raw.chan9_raw = mavlink_msg_hil_rc_inputs_raw_get_chan9_raw(msg);
    	hil_rc_inputs_raw.chan10_raw = mavlink_msg_hil_rc_inputs_raw_get_chan10_raw(msg);
    	hil_rc_inputs_raw.chan11_raw = mavlink_msg_hil_rc_inputs_raw_get_chan11_raw(msg);
    	hil_rc_inputs_raw.chan12_raw = mavlink_msg_hil_rc_inputs_raw_get_chan12_raw(msg);
    	hil_rc_inputs_raw.rssi = mavlink_msg_hil_rc_inputs_raw_get_rssi(msg);
    
    } else {
        int len = 33; //Marshal.SizeOf(hil_rc_inputs_raw);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        hil_rc_inputs_raw = (mavlink_hil_rc_inputs_raw_t)Marshal.PtrToStructure(i, ((object)hil_rc_inputs_raw).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
