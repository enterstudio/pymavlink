// MESSAGE ATTITUDE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ATTITUDE = 30;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_attitude_t
    {
         public  UInt64 usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     public  Single roll; /// Roll angle (rad)
     public  Single pitch; /// Pitch angle (rad)
     public  Single yaw; /// Yaw angle (rad)
     public  Single rollspeed; /// Roll angular speed (rad/s)
     public  Single pitchspeed; /// Pitch angular speed (rad/s)
     public  Single yawspeed; /// Yaw angular speed (rad/s)
    
    };

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_attitude_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt64 public usec, Single public roll, Single public pitch, Single public yaw, Single public rollspeed, Single public pitchspeed, Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, roll);
	_mav_put_Single(buf, 12, pitch);
	_mav_put_Single(buf, 16, yaw);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_attitude_t packet;
	packet.usec = usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, 32);
}
*/
/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public roll,Single public pitch,Single public yaw,Single public rollspeed,Single public pitchspeed,Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, roll);
	_mav_put_Single(buf, 12, pitch);
	_mav_put_Single(buf, 16, yaw);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_attitude_t packet;
	packet.usec = usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}
*/
/**
 * @brief Encode a attitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->usec, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
}
*/
/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, UInt64 public usec, Single public roll, Single public pitch, Single public yaw, Single public rollspeed, Single public pitchspeed, Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, roll);
	_mav_put_Single(buf, 12, pitch);
	_mav_put_Single(buf, 16, yaw);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, 32);
#else
    mavlink_attitude_t packet;
	packet.usec = usec;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)&packet, 32);
#endif
}

#endif
*/
// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field usec from attitude message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_attitude_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field roll from attitude message
 *
 * @return Roll angle (rad)
 */
public static Single mavlink_msg_attitude_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return Pitch angle (rad)
 */
public static Single mavlink_msg_attitude_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return Yaw angle (rad)
 */
public static Single mavlink_msg_attitude_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field rollspeed from attitude message
 *
 * @return Roll angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_get_rollspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field pitchspeed from attitude message
 *
 * @return Pitch angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_get_pitchspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field yawspeed from attitude message
 *
 * @return Yaw angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_get_yawspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
public static void mavlink_msg_attitude_decode(byte[] msg, ref mavlink_attitude_t attitude)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	attitude.usec = mavlink_msg_attitude_get_usec(msg);
	attitude.roll = mavlink_msg_attitude_get_roll(msg);
	attitude.pitch = mavlink_msg_attitude_get_pitch(msg);
	attitude.yaw = mavlink_msg_attitude_get_yaw(msg);
	attitude.rollspeed = mavlink_msg_attitude_get_rollspeed(msg);
	attitude.pitchspeed = mavlink_msg_attitude_get_pitchspeed(msg);
	attitude.yawspeed = mavlink_msg_attitude_get_yawspeed(msg);
} else {
    int len = 32; //Marshal.SizeOf(attitude);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    attitude = (mavlink_attitude_t)Marshal.PtrToStructure(i, ((object)attitude).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
