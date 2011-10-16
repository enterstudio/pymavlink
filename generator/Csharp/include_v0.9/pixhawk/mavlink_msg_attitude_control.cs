// MESSAGE ATTITUDE_CONTROL PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ATTITUDE_CONTROL = 200;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_attitude_control_t
    {
         public  byte target; /// The system to be controlled
     public  Single roll; /// roll
     public  Single pitch; /// pitch
     public  Single yaw; /// yaw
     public  Single thrust; /// thrust
     public  byte roll_manual; /// roll control enabled auto:0, manual:1
     public  byte pitch_manual; /// pitch auto:0, manual:1
     public  byte yaw_manual; /// yaw auto:0, manual:1
     public  byte thrust_manual; /// thrust auto:0, manual:1
    
    };

/**
 * @brief Pack a attitude_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_attitude_control_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target, Single public roll, Single public pitch, Single public yaw, Single public thrust, byte public roll_manual, byte public pitch_manual, byte public yaw_manual, byte public thrust_manual)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[21];
	_mav_put_byte(buf, 0, target);
	_mav_put_Single(buf, 1, roll);
	_mav_put_Single(buf, 5, pitch);
	_mav_put_Single(buf, 9, yaw);
	_mav_put_Single(buf, 13, thrust);
	_mav_put_byte(buf, 17, roll_manual);
	_mav_put_byte(buf, 18, pitch_manual);
	_mav_put_byte(buf, 19, yaw_manual);
	_mav_put_byte(buf, 20, thrust_manual);

        memcpy(_MAV_PAYLOAD(msg), buf, 21);
#else
    mavlink_attitude_control_t packet;
	packet.target = target;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

        memcpy(_MAV_PAYLOAD(msg), &packet, 21);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, 21);
}
*/
/**
 * @brief Pack a attitude_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_attitude_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,Single public roll,Single public pitch,Single public yaw,Single public thrust,byte public roll_manual,byte public pitch_manual,byte public yaw_manual,byte public thrust_manual)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[21];
	_mav_put_byte(buf, 0, target);
	_mav_put_Single(buf, 1, roll);
	_mav_put_Single(buf, 5, pitch);
	_mav_put_Single(buf, 9, yaw);
	_mav_put_Single(buf, 13, thrust);
	_mav_put_byte(buf, 17, roll_manual);
	_mav_put_byte(buf, 18, pitch_manual);
	_mav_put_byte(buf, 19, yaw_manual);
	_mav_put_byte(buf, 20, thrust_manual);

        memcpy(_MAV_PAYLOAD(msg), buf, 21);
#else
    mavlink_attitude_control_t packet;
	packet.target = target;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

        memcpy(_MAV_PAYLOAD(msg), &packet, 21);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 21);
}
*/
/**
 * @brief Encode a attitude_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_control C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_attitude_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_control_t* attitude_control)
{
    return mavlink_msg_attitude_control_pack(system_id, component_id, msg, attitude_control->target, attitude_control->roll, attitude_control->pitch, attitude_control->yaw, attitude_control->thrust, attitude_control->roll_manual, attitude_control->pitch_manual, attitude_control->yaw_manual, attitude_control->thrust_manual);
}
*/
/**
 * @brief Send a attitude_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system to be controlled
 * @param roll roll
 * @param pitch pitch
 * @param yaw yaw
 * @param thrust thrust
 * @param roll_manual roll control enabled auto:0, manual:1
 * @param pitch_manual pitch auto:0, manual:1
 * @param yaw_manual yaw auto:0, manual:1
 * @param thrust_manual thrust auto:0, manual:1
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_control_send(mavlink_channel_t chan, byte public target, Single public roll, Single public pitch, Single public yaw, Single public thrust, byte public roll_manual, byte public pitch_manual, byte public yaw_manual, byte public thrust_manual)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[21];
	_mav_put_byte(buf, 0, target);
	_mav_put_Single(buf, 1, roll);
	_mav_put_Single(buf, 5, pitch);
	_mav_put_Single(buf, 9, yaw);
	_mav_put_Single(buf, 13, thrust);
	_mav_put_byte(buf, 17, roll_manual);
	_mav_put_byte(buf, 18, pitch_manual);
	_mav_put_byte(buf, 19, yaw_manual);
	_mav_put_byte(buf, 20, thrust_manual);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL, buf, 21);
#else
    mavlink_attitude_control_t packet;
	packet.target = target;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_CONTROL, (const char *)&packet, 21);
#endif
}

#endif
*/
// MESSAGE ATTITUDE_CONTROL UNPACKING


/**
 * @brief Get field target from attitude_control message
 *
 * @return The system to be controlled
 */
public static byte mavlink_msg_attitude_control_get_target(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field roll from attitude_control message
 *
 * @return roll
 */
public static Single mavlink_msg_attitude_control_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  1);
}

/**
 * @brief Get field pitch from attitude_control message
 *
 * @return pitch
 */
public static Single mavlink_msg_attitude_control_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  5);
}

/**
 * @brief Get field yaw from attitude_control message
 *
 * @return yaw
 */
public static Single mavlink_msg_attitude_control_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  9);
}

/**
 * @brief Get field thrust from attitude_control message
 *
 * @return thrust
 */
public static Single mavlink_msg_attitude_control_get_thrust(byte[] msg)
{
    return BitConverter.ToSingle(msg,  13);
}

/**
 * @brief Get field roll_manual from attitude_control message
 *
 * @return roll control enabled auto:0, manual:1
 */
public static byte mavlink_msg_attitude_control_get_roll_manual(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field pitch_manual from attitude_control message
 *
 * @return pitch auto:0, manual:1
 */
public static byte mavlink_msg_attitude_control_get_pitch_manual(byte[] msg)
{
    return getByte(msg,  18);
}

/**
 * @brief Get field yaw_manual from attitude_control message
 *
 * @return yaw auto:0, manual:1
 */
public static byte mavlink_msg_attitude_control_get_yaw_manual(byte[] msg)
{
    return getByte(msg,  19);
}

/**
 * @brief Get field thrust_manual from attitude_control message
 *
 * @return thrust auto:0, manual:1
 */
public static byte mavlink_msg_attitude_control_get_thrust_manual(byte[] msg)
{
    return getByte(msg,  20);
}

/**
 * @brief Decode a attitude_control message into a struct
 *
 * @param msg The message to decode
 * @param attitude_control C-struct to decode the message contents into
 */
public static void mavlink_msg_attitude_control_decode(byte[] msg, ref mavlink_attitude_control_t attitude_control)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	attitude_control.target = mavlink_msg_attitude_control_get_target(msg);
	attitude_control.roll = mavlink_msg_attitude_control_get_roll(msg);
	attitude_control.pitch = mavlink_msg_attitude_control_get_pitch(msg);
	attitude_control.yaw = mavlink_msg_attitude_control_get_yaw(msg);
	attitude_control.thrust = mavlink_msg_attitude_control_get_thrust(msg);
	attitude_control.roll_manual = mavlink_msg_attitude_control_get_roll_manual(msg);
	attitude_control.pitch_manual = mavlink_msg_attitude_control_get_pitch_manual(msg);
	attitude_control.yaw_manual = mavlink_msg_attitude_control_get_yaw_manual(msg);
	attitude_control.thrust_manual = mavlink_msg_attitude_control_get_thrust_manual(msg);
} else {
    int len = 21; //Marshal.SizeOf(attitude_control);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    attitude_control = (mavlink_attitude_control_t)Marshal.PtrToStructure(i, ((object)attitude_control).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
