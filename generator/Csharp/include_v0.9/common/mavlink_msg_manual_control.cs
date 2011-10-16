// MESSAGE MANUAL_CONTROL PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MANUAL_CONTROL = 69;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_manual_control_t
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
 * @brief Pack a manual_control message
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
 
public static UInt16 mavlink_msg_manual_control_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, Single roll, Single pitch, Single yaw, Single thrust, byte roll_manual, byte pitch_manual, byte yaw_manual, byte thrust_manual)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(roll),0,msg,1,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,5,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,9,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(thrust),0,msg,13,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(roll_manual),0,msg,17,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(pitch_manual),0,msg,18,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(yaw_manual),0,msg,19,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(thrust_manual),0,msg,20,sizeof(byte));

} else {
    mavlink_manual_control_t packet = new mavlink_manual_control_t();
	packet.target = target;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

        
        int len = 21;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
    //return mavlink_finalize_message(msg, system_id, component_id, 21);
    return 0;
}

/**
 * @brief Pack a manual_control message on a channel
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
static inline uint16_t mavlink_msg_manual_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
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
    mavlink_manual_control_t packet;
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

    msg->msgid = MAVLINK_MSG_ID_MANUAL_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 21);
}
*/
/**
 * @brief Encode a manual_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param manual_control C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_manual_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_manual_control_t* manual_control)
{
    return mavlink_msg_manual_control_pack(system_id, component_id, msg, manual_control->target, manual_control->roll, manual_control->pitch, manual_control->yaw, manual_control->thrust, manual_control->roll_manual, manual_control->pitch_manual, manual_control->yaw_manual, manual_control->thrust_manual);
}
*/
/**
 * @brief Send a manual_control message
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

static inline void mavlink_msg_manual_control_send(mavlink_channel_t chan, byte public target, Single public roll, Single public pitch, Single public yaw, Single public thrust, byte public roll_manual, byte public pitch_manual, byte public yaw_manual, byte public thrust_manual)
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

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, buf, 21);
#else
    mavlink_manual_control_t packet;
	packet.target = target;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.thrust = thrust;
	packet.roll_manual = roll_manual;
	packet.pitch_manual = pitch_manual;
	packet.yaw_manual = yaw_manual;
	packet.thrust_manual = thrust_manual;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MANUAL_CONTROL, (const char *)&packet, 21);
#endif
}

#endif
*/
// MESSAGE MANUAL_CONTROL UNPACKING


/**
 * @brief Get field target from manual_control message
 *
 * @return The system to be controlled
 */
public static byte mavlink_msg_manual_control_get_target(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field roll from manual_control message
 *
 * @return roll
 */
public static Single mavlink_msg_manual_control_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  1);
}

/**
 * @brief Get field pitch from manual_control message
 *
 * @return pitch
 */
public static Single mavlink_msg_manual_control_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  5);
}

/**
 * @brief Get field yaw from manual_control message
 *
 * @return yaw
 */
public static Single mavlink_msg_manual_control_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  9);
}

/**
 * @brief Get field thrust from manual_control message
 *
 * @return thrust
 */
public static Single mavlink_msg_manual_control_get_thrust(byte[] msg)
{
    return BitConverter.ToSingle(msg,  13);
}

/**
 * @brief Get field roll_manual from manual_control message
 *
 * @return roll control enabled auto:0, manual:1
 */
public static byte mavlink_msg_manual_control_get_roll_manual(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field pitch_manual from manual_control message
 *
 * @return pitch auto:0, manual:1
 */
public static byte mavlink_msg_manual_control_get_pitch_manual(byte[] msg)
{
    return getByte(msg,  18);
}

/**
 * @brief Get field yaw_manual from manual_control message
 *
 * @return yaw auto:0, manual:1
 */
public static byte mavlink_msg_manual_control_get_yaw_manual(byte[] msg)
{
    return getByte(msg,  19);
}

/**
 * @brief Get field thrust_manual from manual_control message
 *
 * @return thrust auto:0, manual:1
 */
public static byte mavlink_msg_manual_control_get_thrust_manual(byte[] msg)
{
    return getByte(msg,  20);
}

/**
 * @brief Decode a manual_control message into a struct
 *
 * @param msg The message to decode
 * @param manual_control C-struct to decode the message contents into
 */
public static void mavlink_msg_manual_control_decode(byte[] msg, ref mavlink_manual_control_t manual_control)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	manual_control.target = mavlink_msg_manual_control_get_target(msg);
    	manual_control.roll = mavlink_msg_manual_control_get_roll(msg);
    	manual_control.pitch = mavlink_msg_manual_control_get_pitch(msg);
    	manual_control.yaw = mavlink_msg_manual_control_get_yaw(msg);
    	manual_control.thrust = mavlink_msg_manual_control_get_thrust(msg);
    	manual_control.roll_manual = mavlink_msg_manual_control_get_roll_manual(msg);
    	manual_control.pitch_manual = mavlink_msg_manual_control_get_pitch_manual(msg);
    	manual_control.yaw_manual = mavlink_msg_manual_control_get_yaw_manual(msg);
    	manual_control.thrust_manual = mavlink_msg_manual_control_get_thrust_manual(msg);
    
    } else {
        int len = 21; //Marshal.SizeOf(manual_control);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        manual_control = (mavlink_manual_control_t)Marshal.PtrToStructure(i, ((object)manual_control).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
