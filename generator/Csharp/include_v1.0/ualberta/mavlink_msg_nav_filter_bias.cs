// MESSAGE NAV_FILTER_BIAS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_NAV_FILTER_BIAS = 220;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_nav_filter_bias_t
    {
         public  UInt64 usec; /// Timestamp (microseconds)
     public  Single accel_0; /// b_f[0]
     public  Single accel_1; /// b_f[1]
     public  Single accel_2; /// b_f[2]
     public  Single gyro_0; /// b_f[0]
     public  Single gyro_1; /// b_f[1]
     public  Single gyro_2; /// b_f[2]
    
    };

/**
 * @brief Pack a nav_filter_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds)
 * @param accel_0 b_f[0]
 * @param accel_1 b_f[1]
 * @param accel_2 b_f[2]
 * @param gyro_0 b_f[0]
 * @param gyro_1 b_f[1]
 * @param gyro_2 b_f[2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_nav_filter_bias_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt64 public usec, Single public accel_0, Single public accel_1, Single public accel_2, Single public gyro_0, Single public gyro_1, Single public gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, accel_0);
	_mav_put_Single(buf, 12, accel_1);
	_mav_put_Single(buf, 16, accel_2);
	_mav_put_Single(buf, 20, gyro_0);
	_mav_put_Single(buf, 24, gyro_1);
	_mav_put_Single(buf, 28, gyro_2);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_nav_filter_bias_t packet;
	packet.usec = usec;
	packet.accel_0 = accel_0;
	packet.accel_1 = accel_1;
	packet.accel_2 = accel_2;
	packet.gyro_0 = gyro_0;
	packet.gyro_1 = gyro_1;
	packet.gyro_2 = gyro_2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_FILTER_BIAS;
    return mavlink_finalize_message(msg, system_id, component_id, 32, 34);
}
*/
/**
 * @brief Pack a nav_filter_bias message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds)
 * @param accel_0 b_f[0]
 * @param accel_1 b_f[1]
 * @param accel_2 b_f[2]
 * @param gyro_0 b_f[0]
 * @param gyro_1 b_f[1]
 * @param gyro_2 b_f[2]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_nav_filter_bias_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public accel_0,Single public accel_1,Single public accel_2,Single public gyro_0,Single public gyro_1,Single public gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, accel_0);
	_mav_put_Single(buf, 12, accel_1);
	_mav_put_Single(buf, 16, accel_2);
	_mav_put_Single(buf, 20, gyro_0);
	_mav_put_Single(buf, 24, gyro_1);
	_mav_put_Single(buf, 28, gyro_2);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_nav_filter_bias_t packet;
	packet.usec = usec;
	packet.accel_0 = accel_0;
	packet.accel_1 = accel_1;
	packet.accel_2 = accel_2;
	packet.gyro_0 = gyro_0;
	packet.gyro_1 = gyro_1;
	packet.gyro_2 = gyro_2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_FILTER_BIAS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32, 34);
}
*/
/**
 * @brief Encode a nav_filter_bias struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nav_filter_bias C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_nav_filter_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_filter_bias_t* nav_filter_bias)
{
    return mavlink_msg_nav_filter_bias_pack(system_id, component_id, msg, nav_filter_bias->usec, nav_filter_bias->accel_0, nav_filter_bias->accel_1, nav_filter_bias->accel_2, nav_filter_bias->gyro_0, nav_filter_bias->gyro_1, nav_filter_bias->gyro_2);
}
*/
/**
 * @brief Send a nav_filter_bias message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds)
 * @param accel_0 b_f[0]
 * @param accel_1 b_f[1]
 * @param accel_2 b_f[2]
 * @param gyro_0 b_f[0]
 * @param gyro_1 b_f[1]
 * @param gyro_2 b_f[2]
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_filter_bias_send(mavlink_channel_t chan, UInt64 public usec, Single public accel_0, Single public accel_1, Single public accel_2, Single public gyro_0, Single public gyro_1, Single public gyro_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, accel_0);
	_mav_put_Single(buf, 12, accel_1);
	_mav_put_Single(buf, 16, accel_2);
	_mav_put_Single(buf, 20, gyro_0);
	_mav_put_Single(buf, 24, gyro_1);
	_mav_put_Single(buf, 28, gyro_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, buf, 32, 34);
#else
    mavlink_nav_filter_bias_t packet;
	packet.usec = usec;
	packet.accel_0 = accel_0;
	packet.accel_1 = accel_1;
	packet.accel_2 = accel_2;
	packet.gyro_0 = gyro_0;
	packet.gyro_1 = gyro_1;
	packet.gyro_2 = gyro_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_FILTER_BIAS, (const char *)&packet, 32, 34);
#endif
}

#endif
*/
// MESSAGE NAV_FILTER_BIAS UNPACKING


/**
 * @brief Get field usec from nav_filter_bias message
 *
 * @return Timestamp (microseconds)
 */
public static UInt64 mavlink_msg_nav_filter_bias_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field accel_0 from nav_filter_bias message
 *
 * @return b_f[0]
 */
public static Single mavlink_msg_nav_filter_bias_get_accel_0(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field accel_1 from nav_filter_bias message
 *
 * @return b_f[1]
 */
public static Single mavlink_msg_nav_filter_bias_get_accel_1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field accel_2 from nav_filter_bias message
 *
 * @return b_f[2]
 */
public static Single mavlink_msg_nav_filter_bias_get_accel_2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field gyro_0 from nav_filter_bias message
 *
 * @return b_f[0]
 */
public static Single mavlink_msg_nav_filter_bias_get_gyro_0(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field gyro_1 from nav_filter_bias message
 *
 * @return b_f[1]
 */
public static Single mavlink_msg_nav_filter_bias_get_gyro_1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field gyro_2 from nav_filter_bias message
 *
 * @return b_f[2]
 */
public static Single mavlink_msg_nav_filter_bias_get_gyro_2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a nav_filter_bias message into a struct
 *
 * @param msg The message to decode
 * @param nav_filter_bias C-struct to decode the message contents into
 */
public static void mavlink_msg_nav_filter_bias_decode(byte[] msg, ref mavlink_nav_filter_bias_t nav_filter_bias)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	nav_filter_bias.usec = mavlink_msg_nav_filter_bias_get_usec(msg);
	nav_filter_bias.accel_0 = mavlink_msg_nav_filter_bias_get_accel_0(msg);
	nav_filter_bias.accel_1 = mavlink_msg_nav_filter_bias_get_accel_1(msg);
	nav_filter_bias.accel_2 = mavlink_msg_nav_filter_bias_get_accel_2(msg);
	nav_filter_bias.gyro_0 = mavlink_msg_nav_filter_bias_get_gyro_0(msg);
	nav_filter_bias.gyro_1 = mavlink_msg_nav_filter_bias_get_gyro_1(msg);
	nav_filter_bias.gyro_2 = mavlink_msg_nav_filter_bias_get_gyro_2(msg);
} else {
    int len = 32; //Marshal.SizeOf(nav_filter_bias);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    nav_filter_bias = (mavlink_nav_filter_bias_t)Marshal.PtrToStructure(i, ((object)nav_filter_bias).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
