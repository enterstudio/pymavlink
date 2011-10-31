// MESSAGE RC_CHANNELS_SCALED PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 36;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_rc_channels_scaled_t
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan1_scaled;
            /// <summary>
        /// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan2_scaled;
            /// <summary>
        /// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan3_scaled;
            /// <summary>
        /// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan4_scaled;
            /// <summary>
        /// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan5_scaled;
            /// <summary>
        /// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan6_scaled;
            /// <summary>
        /// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan7_scaled;
            /// <summary>
        /// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
        /// </summary>
        public  Int16 chan8_scaled;
            /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        /// </summary>
        public  byte port;
            /// <summary>
        /// Receive signal strength indicator, 0: 0%, 255: 100%
        /// </summary>
        public  byte rssi;
    
    };

/// <summary>
/// * @brief Pack a rc_channels_scaled message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp (milliseconds since system boot)
/// * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
/// * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
/// * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_rc_channels_scaled_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, byte port, Int16 chan1_scaled, Int16 chan2_scaled, Int16 chan3_scaled, Int16 chan4_scaled, Int16 chan5_scaled, Int16 chan6_scaled, Int16 chan7_scaled, Int16 chan8_scaled, byte rssi)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(chan1_scaled),0,msg,4,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan2_scaled),0,msg,6,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan3_scaled),0,msg,8,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan4_scaled),0,msg,10,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan5_scaled),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan6_scaled),0,msg,14,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan7_scaled),0,msg,16,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(chan8_scaled),0,msg,18,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(port),0,msg,20,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(rssi),0,msg,21,sizeof(byte));

} else {
    mavlink_rc_channels_scaled_t packet = new mavlink_rc_channels_scaled_t();
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.port = port;
	packet.rssi = rssi;

        
        int len = 22;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_RC_CHANNELS_SCALED;
    //return mavlink_finalize_message(msg, system_id, component_id, 22, 237);
    return 0;
}

/**
 * @brief Pack a rc_channels_scaled message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_rc_channels_scaled_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,byte public port,Int16 public chan1_scaled,Int16 public chan2_scaled,Int16 public chan3_scaled,Int16 public chan4_scaled,Int16 public chan5_scaled,Int16 public chan6_scaled,Int16 public chan7_scaled,Int16 public chan8_scaled,byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[22];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int16(buf, 4, chan1_scaled);
	_mav_put_Int16(buf, 6, chan2_scaled);
	_mav_put_Int16(buf, 8, chan3_scaled);
	_mav_put_Int16(buf, 10, chan4_scaled);
	_mav_put_Int16(buf, 12, chan5_scaled);
	_mav_put_Int16(buf, 14, chan6_scaled);
	_mav_put_Int16(buf, 16, chan7_scaled);
	_mav_put_Int16(buf, 18, chan8_scaled);
	_mav_put_byte(buf, 20, port);
	_mav_put_byte(buf, 21, rssi);

        memcpy(_MAV_PAYLOAD(msg), buf, 22);
#else
    mavlink_rc_channels_scaled_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.port = port;
	packet.rssi = rssi;

        memcpy(_MAV_PAYLOAD(msg), &packet, 22);
#endif

    msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_SCALED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 22, 237);
}
*/
/**
 * @brief Encode a rc_channels_scaled struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_scaled C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_rc_channels_scaled_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_scaled_t* rc_channels_scaled)
{
    return mavlink_msg_rc_channels_scaled_pack(system_id, component_id, msg, rc_channels_scaled->time_boot_ms, rc_channels_scaled->port, rc_channels_scaled->chan1_scaled, rc_channels_scaled->chan2_scaled, rc_channels_scaled->chan3_scaled, rc_channels_scaled->chan4_scaled, rc_channels_scaled->chan5_scaled, rc_channels_scaled->chan6_scaled, rc_channels_scaled->chan7_scaled, rc_channels_scaled->chan8_scaled, rc_channels_scaled->rssi);
}
*/
/**
 * @brief Send a rc_channels_scaled message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param chan1_scaled RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan2_scaled RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan3_scaled RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan4_scaled RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan5_scaled RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan6_scaled RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan7_scaled RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param chan8_scaled RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 * @param rssi Receive signal strength indicator, 0: 0%, 255: 100%
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_scaled_send(mavlink_channel_t chan, UInt32 public time_boot_ms, byte public port, Int16 public chan1_scaled, Int16 public chan2_scaled, Int16 public chan3_scaled, Int16 public chan4_scaled, Int16 public chan5_scaled, Int16 public chan6_scaled, Int16 public chan7_scaled, Int16 public chan8_scaled, byte public rssi)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[22];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int16(buf, 4, chan1_scaled);
	_mav_put_Int16(buf, 6, chan2_scaled);
	_mav_put_Int16(buf, 8, chan3_scaled);
	_mav_put_Int16(buf, 10, chan4_scaled);
	_mav_put_Int16(buf, 12, chan5_scaled);
	_mav_put_Int16(buf, 14, chan6_scaled);
	_mav_put_Int16(buf, 16, chan7_scaled);
	_mav_put_Int16(buf, 18, chan8_scaled);
	_mav_put_byte(buf, 20, port);
	_mav_put_byte(buf, 21, rssi);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, buf, 22, 237);
#else
    mavlink_rc_channels_scaled_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.chan1_scaled = chan1_scaled;
	packet.chan2_scaled = chan2_scaled;
	packet.chan3_scaled = chan3_scaled;
	packet.chan4_scaled = chan4_scaled;
	packet.chan5_scaled = chan5_scaled;
	packet.chan6_scaled = chan6_scaled;
	packet.chan7_scaled = chan7_scaled;
	packet.chan8_scaled = chan8_scaled;
	packet.port = port;
	packet.rssi = rssi;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_CHANNELS_SCALED, (const char *)&packet, 22, 237);
#endif
}

#endif
*/
// MESSAGE RC_CHANNELS_SCALED UNPACKING


/**
 * @brief Get field time_boot_ms from rc_channels_scaled message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_rc_channels_scaled_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field port from rc_channels_scaled message
 *
 * @return Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 */
public static byte mavlink_msg_rc_channels_scaled_get_port(byte[] msg)
{
    return getByte(msg,  20);
}

/**
 * @brief Get field chan1_scaled from rc_channels_scaled message
 *
 * @return RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan1_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  4);
}

/**
 * @brief Get field chan2_scaled from rc_channels_scaled message
 *
 * @return RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan2_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  6);
}

/**
 * @brief Get field chan3_scaled from rc_channels_scaled message
 *
 * @return RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan3_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  8);
}

/**
 * @brief Get field chan4_scaled from rc_channels_scaled message
 *
 * @return RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan4_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  10);
}

/**
 * @brief Get field chan5_scaled from rc_channels_scaled message
 *
 * @return RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan5_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field chan6_scaled from rc_channels_scaled message
 *
 * @return RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan6_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Get field chan7_scaled from rc_channels_scaled message
 *
 * @return RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan7_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Get field chan8_scaled from rc_channels_scaled message
 *
 * @return RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
 */
public static Int16 mavlink_msg_rc_channels_scaled_get_chan8_scaled(byte[] msg)
{
    return BitConverter.ToInt16(msg,  18);
}

/**
 * @brief Get field rssi from rc_channels_scaled message
 *
 * @return Receive signal strength indicator, 0: 0%, 255: 100%
 */
public static byte mavlink_msg_rc_channels_scaled_get_rssi(byte[] msg)
{
    return getByte(msg,  21);
}

/**
 * @brief Decode a rc_channels_scaled message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_scaled C-struct to decode the message contents into
 */
public static void mavlink_msg_rc_channels_scaled_decode(byte[] msg, ref mavlink_rc_channels_scaled_t rc_channels_scaled)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	rc_channels_scaled.time_boot_ms = mavlink_msg_rc_channels_scaled_get_time_boot_ms(msg);
    	rc_channels_scaled.chan1_scaled = mavlink_msg_rc_channels_scaled_get_chan1_scaled(msg);
    	rc_channels_scaled.chan2_scaled = mavlink_msg_rc_channels_scaled_get_chan2_scaled(msg);
    	rc_channels_scaled.chan3_scaled = mavlink_msg_rc_channels_scaled_get_chan3_scaled(msg);
    	rc_channels_scaled.chan4_scaled = mavlink_msg_rc_channels_scaled_get_chan4_scaled(msg);
    	rc_channels_scaled.chan5_scaled = mavlink_msg_rc_channels_scaled_get_chan5_scaled(msg);
    	rc_channels_scaled.chan6_scaled = mavlink_msg_rc_channels_scaled_get_chan6_scaled(msg);
    	rc_channels_scaled.chan7_scaled = mavlink_msg_rc_channels_scaled_get_chan7_scaled(msg);
    	rc_channels_scaled.chan8_scaled = mavlink_msg_rc_channels_scaled_get_chan8_scaled(msg);
    	rc_channels_scaled.port = mavlink_msg_rc_channels_scaled_get_port(msg);
    	rc_channels_scaled.rssi = mavlink_msg_rc_channels_scaled_get_rssi(msg);
    
    } else {
        int len = 22; //Marshal.SizeOf(rc_channels_scaled);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        rc_channels_scaled = (mavlink_rc_channels_scaled_t)Marshal.PtrToStructure(i, ((object)rc_channels_scaled).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
