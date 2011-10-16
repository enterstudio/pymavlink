// MESSAGE ROLL_PITCH_YAW_THRUST_SETPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT = 57;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_roll_pitch_yaw_thrust_setpoint_t
    {
         public  UInt32 time_ms; /// Timestamp in milliseconds since system boot
     public  Single roll; /// Desired roll angle in radians
     public  Single pitch; /// Desired pitch angle in radians
     public  Single yaw; /// Desired yaw angle in radians
     public  Single thrust; /// Collective thrust, normalized to 0 .. 1
    
    };

/**
 * @brief Pack a roll_pitch_yaw_thrust_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_ms Timestamp in milliseconds since system boot
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt32 public time_ms, Single public roll, Single public pitch, Single public yaw, Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[20];
	_mav_put_UInt32(buf, 0, time_ms);
	_mav_put_Single(buf, 4, roll);
	_mav_put_Single(buf, 8, pitch);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_Single(buf, 16, thrust);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_roll_pitch_yaw_thrust_setpoint_t packet;
	packet.time_ms = time_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, 20, 141);
}
*/
/**
 * @brief Pack a roll_pitch_yaw_thrust_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_ms Timestamp in milliseconds since system boot
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_ms,Single public roll,Single public pitch,Single public yaw,Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_UInt32(buf, 0, time_ms);
	_mav_put_Single(buf, 4, roll);
	_mav_put_Single(buf, 8, pitch);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_Single(buf, 16, thrust);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_roll_pitch_yaw_thrust_setpoint_t packet;
	packet.time_ms = time_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 141);
}
*/
/**
 * @brief Encode a roll_pitch_yaw_thrust_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_yaw_thrust_setpoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_roll_pitch_yaw_thrust_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_yaw_thrust_setpoint_t* roll_pitch_yaw_thrust_setpoint)
{
    return mavlink_msg_roll_pitch_yaw_thrust_setpoint_pack(system_id, component_id, msg, roll_pitch_yaw_thrust_setpoint->time_ms, roll_pitch_yaw_thrust_setpoint->roll, roll_pitch_yaw_thrust_setpoint->pitch, roll_pitch_yaw_thrust_setpoint->yaw, roll_pitch_yaw_thrust_setpoint->thrust);
}
*/
/**
 * @brief Send a roll_pitch_yaw_thrust_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_ms Timestamp in milliseconds since system boot
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(mavlink_channel_t chan, UInt32 public time_ms, Single public roll, Single public pitch, Single public yaw, Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_UInt32(buf, 0, time_ms);
	_mav_put_Single(buf, 4, roll);
	_mav_put_Single(buf, 8, pitch);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_Single(buf, 16, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT, buf, 20, 141);
#else
    mavlink_roll_pitch_yaw_thrust_setpoint_t packet;
	packet.time_ms = time_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT, (const char *)&packet, 20, 141);
#endif
}

#endif
*/
// MESSAGE ROLL_PITCH_YAW_THRUST_SETPOINT UNPACKING


/**
 * @brief Get field time_ms from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Timestamp in milliseconds since system boot
 */
public static UInt32 mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_time_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field roll from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Desired roll angle in radians
 */
public static Single mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field pitch from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Desired pitch angle in radians
 */
public static Single mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Desired yaw angle in radians
 */
public static Single mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field thrust from roll_pitch_yaw_thrust_setpoint message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
public static Single mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_thrust(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Decode a roll_pitch_yaw_thrust_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_yaw_thrust_setpoint C-struct to decode the message contents into
 */
public static void mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(byte[] msg, ref mavlink_roll_pitch_yaw_thrust_setpoint_t roll_pitch_yaw_thrust_setpoint)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	roll_pitch_yaw_thrust_setpoint.time_ms = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_time_ms(msg);
	roll_pitch_yaw_thrust_setpoint.roll = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_roll(msg);
	roll_pitch_yaw_thrust_setpoint.pitch = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_pitch(msg);
	roll_pitch_yaw_thrust_setpoint.yaw = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_yaw(msg);
	roll_pitch_yaw_thrust_setpoint.thrust = mavlink_msg_roll_pitch_yaw_thrust_setpoint_get_thrust(msg);
} else {
    int len = 20; //Marshal.SizeOf(roll_pitch_yaw_thrust_setpoint);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    roll_pitch_yaw_thrust_setpoint = (mavlink_roll_pitch_yaw_thrust_setpoint_t)Marshal.PtrToStructure(i, ((object)roll_pitch_yaw_thrust_setpoint).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
