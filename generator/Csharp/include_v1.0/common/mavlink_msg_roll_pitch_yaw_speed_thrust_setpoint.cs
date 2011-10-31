// MESSAGE ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT = 59;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_roll_pitch_yaw_speed_thrust_setpoint_t
    {
        /// <summary>
        /// Timestamp in milliseconds since system boot
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// Desired roll angular speed in rad/s
        /// </summary>
        public  Single roll_speed;
            /// <summary>
        /// Desired pitch angular speed in rad/s
        /// </summary>
        public  Single pitch_speed;
            /// <summary>
        /// Desired yaw angular speed in rad/s
        /// </summary>
        public  Single yaw_speed;
            /// <summary>
        /// Collective thrust, normalized to 0 .. 1
        /// </summary>
        public  Single thrust;
    
    };

/// <summary>
/// * @brief Pack a roll_pitch_yaw_speed_thrust_setpoint message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp in milliseconds since system boot
/// * @param roll_speed Desired roll angular speed in rad/s
/// * @param pitch_speed Desired pitch angular speed in rad/s
/// * @param yaw_speed Desired yaw angular speed in rad/s
/// * @param thrust Collective thrust, normalized to 0 .. 1
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Single roll_speed, Single pitch_speed, Single yaw_speed, Single thrust)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(roll_speed),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch_speed),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw_speed),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(thrust),0,msg,16,sizeof(Single));

} else {
    mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet = new mavlink_roll_pitch_yaw_speed_thrust_setpoint_t();
	packet.time_boot_ms = time_boot_ms;
	packet.roll_speed = roll_speed;
	packet.pitch_speed = pitch_speed;
	packet.yaw_speed = yaw_speed;
	packet.thrust = thrust;

        
        int len = 20;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT;
    //return mavlink_finalize_message(msg, system_id, component_id, 20, 238);
    return 0;
}

/**
 * @brief Pack a roll_pitch_yaw_speed_thrust_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param roll_speed Desired roll angular speed in rad/s
 * @param pitch_speed Desired pitch angular speed in rad/s
 * @param yaw_speed Desired yaw angular speed in rad/s
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,Single public roll_speed,Single public pitch_speed,Single public yaw_speed,Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, roll_speed);
	_mav_put_Single(buf, 8, pitch_speed);
	_mav_put_Single(buf, 12, yaw_speed);
	_mav_put_Single(buf, 16, thrust);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll_speed = roll_speed;
	packet.pitch_speed = pitch_speed;
	packet.yaw_speed = yaw_speed;
	packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 238);
}
*/
/**
 * @brief Encode a roll_pitch_yaw_speed_thrust_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_pitch_yaw_speed_thrust_setpoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_pitch_yaw_speed_thrust_setpoint_t* roll_pitch_yaw_speed_thrust_setpoint)
{
    return mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(system_id, component_id, msg, roll_pitch_yaw_speed_thrust_setpoint->time_boot_ms, roll_pitch_yaw_speed_thrust_setpoint->roll_speed, roll_pitch_yaw_speed_thrust_setpoint->pitch_speed, roll_pitch_yaw_speed_thrust_setpoint->yaw_speed, roll_pitch_yaw_speed_thrust_setpoint->thrust);
}
*/
/**
 * @brief Send a roll_pitch_yaw_speed_thrust_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp in milliseconds since system boot
 * @param roll_speed Desired roll angular speed in rad/s
 * @param pitch_speed Desired pitch angular speed in rad/s
 * @param yaw_speed Desired yaw angular speed in rad/s
 * @param thrust Collective thrust, normalized to 0 .. 1
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Single public roll_speed, Single public pitch_speed, Single public yaw_speed, Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, roll_speed);
	_mav_put_Single(buf, 8, pitch_speed);
	_mav_put_Single(buf, 12, yaw_speed);
	_mav_put_Single(buf, 16, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT, buf, 20, 238);
#else
    mavlink_roll_pitch_yaw_speed_thrust_setpoint_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll_speed = roll_speed;
	packet.pitch_speed = pitch_speed;
	packet.yaw_speed = yaw_speed;
	packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT, (const char *)&packet, 20, 238);
#endif
}

#endif
*/
// MESSAGE ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT UNPACKING


/**
 * @brief Get field time_boot_ms from roll_pitch_yaw_speed_thrust_setpoint message
 *
 * @return Timestamp in milliseconds since system boot
 */
public static UInt32 mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field roll_speed from roll_pitch_yaw_speed_thrust_setpoint message
 *
 * @return Desired roll angular speed in rad/s
 */
public static Single mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_roll_speed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field pitch_speed from roll_pitch_yaw_speed_thrust_setpoint message
 *
 * @return Desired pitch angular speed in rad/s
 */
public static Single mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_pitch_speed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw_speed from roll_pitch_yaw_speed_thrust_setpoint message
 *
 * @return Desired yaw angular speed in rad/s
 */
public static Single mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_yaw_speed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field thrust from roll_pitch_yaw_speed_thrust_setpoint message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
public static Single mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_thrust(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Decode a roll_pitch_yaw_speed_thrust_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param roll_pitch_yaw_speed_thrust_setpoint C-struct to decode the message contents into
 */
public static void mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(byte[] msg, ref mavlink_roll_pitch_yaw_speed_thrust_setpoint_t roll_pitch_yaw_speed_thrust_setpoint)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	roll_pitch_yaw_speed_thrust_setpoint.time_boot_ms = mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_time_boot_ms(msg);
    	roll_pitch_yaw_speed_thrust_setpoint.roll_speed = mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_roll_speed(msg);
    	roll_pitch_yaw_speed_thrust_setpoint.pitch_speed = mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_pitch_speed(msg);
    	roll_pitch_yaw_speed_thrust_setpoint.yaw_speed = mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_yaw_speed(msg);
    	roll_pitch_yaw_speed_thrust_setpoint.thrust = mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_get_thrust(msg);
    
    } else {
        int len = 20; //Marshal.SizeOf(roll_pitch_yaw_speed_thrust_setpoint);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        roll_pitch_yaw_speed_thrust_setpoint = (mavlink_roll_pitch_yaw_speed_thrust_setpoint_t)Marshal.PtrToStructure(i, ((object)roll_pitch_yaw_speed_thrust_setpoint).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
