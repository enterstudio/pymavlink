// MESSAGE SET_ROLL_PITCH_YAW_THRUST PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST = 56;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_roll_pitch_yaw_thrust_t
    {
         public  Single roll; /// Desired roll angle in radians
     public  Single pitch; /// Desired pitch angle in radians
     public  Single yaw; /// Desired yaw angle in radians
     public  Single thrust; /// Collective thrust, normalized to 0 .. 1
     public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
    
    };

/**
 * @brief Pack a set_roll_pitch_yaw_thrust message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_set_roll_pitch_yaw_thrust_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, Single roll, Single pitch, Single yaw, Single thrust)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(roll),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(thrust),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,16,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,17,sizeof(byte));

} else {
    mavlink_set_roll_pitch_yaw_thrust_t packet = new mavlink_set_roll_pitch_yaw_thrust_t();
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;

        
        int len = 18;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST;
    //return mavlink_finalize_message(msg, system_id, component_id, 18, 100);
    return 0;
}

/**
 * @brief Pack a set_roll_pitch_yaw_thrust message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_thrust_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,Single public roll,Single public pitch,Single public yaw,Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_Single(buf, 0, roll);
	_mav_put_Single(buf, 4, pitch);
	_mav_put_Single(buf, 8, yaw);
	_mav_put_Single(buf, 12, thrust);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_set_roll_pitch_yaw_thrust_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 100);
}
*/
/**
 * @brief Encode a set_roll_pitch_yaw_thrust struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_roll_pitch_yaw_thrust C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_roll_pitch_yaw_thrust_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_roll_pitch_yaw_thrust_t* set_roll_pitch_yaw_thrust)
{
    return mavlink_msg_set_roll_pitch_yaw_thrust_pack(system_id, component_id, msg, set_roll_pitch_yaw_thrust->target_system, set_roll_pitch_yaw_thrust->target_component, set_roll_pitch_yaw_thrust->roll, set_roll_pitch_yaw_thrust->pitch, set_roll_pitch_yaw_thrust->yaw, set_roll_pitch_yaw_thrust->thrust);
}
*/
/**
 * @brief Send a set_roll_pitch_yaw_thrust message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param roll Desired roll angle in radians
 * @param pitch Desired pitch angle in radians
 * @param yaw Desired yaw angle in radians
 * @param thrust Collective thrust, normalized to 0 .. 1
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_roll_pitch_yaw_thrust_send(mavlink_channel_t chan, byte public target_system, byte public target_component, Single public roll, Single public pitch, Single public yaw, Single public thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_Single(buf, 0, roll);
	_mav_put_Single(buf, 4, pitch);
	_mav_put_Single(buf, 8, yaw);
	_mav_put_Single(buf, 12, thrust);
	_mav_put_byte(buf, 16, target_system);
	_mav_put_byte(buf, 17, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST, buf, 18, 100);
#else
    mavlink_set_roll_pitch_yaw_thrust_t packet;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ROLL_PITCH_YAW_THRUST, (const char *)&packet, 18, 100);
#endif
}

#endif
*/
// MESSAGE SET_ROLL_PITCH_YAW_THRUST UNPACKING


/**
 * @brief Get field target_system from set_roll_pitch_yaw_thrust message
 *
 * @return System ID
 */
public static byte mavlink_msg_set_roll_pitch_yaw_thrust_get_target_system(byte[] msg)
{
    return getByte(msg,  16);
}

/**
 * @brief Get field target_component from set_roll_pitch_yaw_thrust message
 *
 * @return Component ID
 */
public static byte mavlink_msg_set_roll_pitch_yaw_thrust_get_target_component(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field roll from set_roll_pitch_yaw_thrust message
 *
 * @return Desired roll angle in radians
 */
public static Single mavlink_msg_set_roll_pitch_yaw_thrust_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field pitch from set_roll_pitch_yaw_thrust message
 *
 * @return Desired pitch angle in radians
 */
public static Single mavlink_msg_set_roll_pitch_yaw_thrust_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field yaw from set_roll_pitch_yaw_thrust message
 *
 * @return Desired yaw angle in radians
 */
public static Single mavlink_msg_set_roll_pitch_yaw_thrust_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field thrust from set_roll_pitch_yaw_thrust message
 *
 * @return Collective thrust, normalized to 0 .. 1
 */
public static Single mavlink_msg_set_roll_pitch_yaw_thrust_get_thrust(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a set_roll_pitch_yaw_thrust message into a struct
 *
 * @param msg The message to decode
 * @param set_roll_pitch_yaw_thrust C-struct to decode the message contents into
 */
public static void mavlink_msg_set_roll_pitch_yaw_thrust_decode(byte[] msg, ref mavlink_set_roll_pitch_yaw_thrust_t set_roll_pitch_yaw_thrust)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	set_roll_pitch_yaw_thrust.roll = mavlink_msg_set_roll_pitch_yaw_thrust_get_roll(msg);
    	set_roll_pitch_yaw_thrust.pitch = mavlink_msg_set_roll_pitch_yaw_thrust_get_pitch(msg);
    	set_roll_pitch_yaw_thrust.yaw = mavlink_msg_set_roll_pitch_yaw_thrust_get_yaw(msg);
    	set_roll_pitch_yaw_thrust.thrust = mavlink_msg_set_roll_pitch_yaw_thrust_get_thrust(msg);
    	set_roll_pitch_yaw_thrust.target_system = mavlink_msg_set_roll_pitch_yaw_thrust_get_target_system(msg);
    	set_roll_pitch_yaw_thrust.target_component = mavlink_msg_set_roll_pitch_yaw_thrust_get_target_component(msg);
    
    } else {
        int len = 18; //Marshal.SizeOf(set_roll_pitch_yaw_thrust);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        set_roll_pitch_yaw_thrust = (mavlink_set_roll_pitch_yaw_thrust_t)Marshal.PtrToStructure(i, ((object)set_roll_pitch_yaw_thrust).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
