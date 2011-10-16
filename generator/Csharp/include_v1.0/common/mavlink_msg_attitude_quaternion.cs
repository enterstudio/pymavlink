// MESSAGE ATTITUDE_QUATERNION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_attitude_quaternion_t
    {
         public  UInt32 time_boot_ms; /// Timestamp (milliseconds since system boot)
     public  Single q1; /// Quaternion component 1
     public  Single q2; /// Quaternion component 2
     public  Single q3; /// Quaternion component 3
     public  Single q4; /// Quaternion component 4
     public  Single rollspeed; /// Roll angular speed (rad/s)
     public  Single pitchspeed; /// Pitch angular speed (rad/s)
     public  Single yawspeed; /// Yaw angular speed (rad/s)
    
    };

/**
 * @brief Pack a attitude_quaternion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1
 * @param q2 Quaternion component 2
 * @param q3 Quaternion component 3
 * @param q4 Quaternion component 4
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_attitude_quaternion_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Single q1, Single q2, Single q3, Single q4, Single rollspeed, Single pitchspeed, Single yawspeed)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(q1),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(q2),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(q3),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(q4),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(rollspeed),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitchspeed),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yawspeed),0,msg,28,sizeof(Single));

} else {
    mavlink_attitude_quaternion_t packet = new mavlink_attitude_quaternion_t();
	packet.time_boot_ms = time_boot_ms;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        
        int len = 32;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    //return mavlink_finalize_message(msg, system_id, component_id, 32, 246);
    return 0;
}

/**
 * @brief Pack a attitude_quaternion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1
 * @param q2 Quaternion component 2
 * @param q3 Quaternion component 3
 * @param q4 Quaternion component 4
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_attitude_quaternion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,Single public q1,Single public q2,Single public q3,Single public q4,Single public rollspeed,Single public pitchspeed,Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, q1);
	_mav_put_Single(buf, 8, q2);
	_mav_put_Single(buf, 12, q3);
	_mav_put_Single(buf, 16, q4);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_attitude_quaternion_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32, 246);
}
*/
/**
 * @brief Encode a attitude_quaternion struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_quaternion C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_attitude_quaternion_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_quaternion_t* attitude_quaternion)
{
    return mavlink_msg_attitude_quaternion_pack(system_id, component_id, msg, attitude_quaternion->time_boot_ms, attitude_quaternion->q1, attitude_quaternion->q2, attitude_quaternion->q3, attitude_quaternion->q4, attitude_quaternion->rollspeed, attitude_quaternion->pitchspeed, attitude_quaternion->yawspeed);
}
*/
/**
 * @brief Send a attitude_quaternion message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param q1 Quaternion component 1
 * @param q2 Quaternion component 2
 * @param q3 Quaternion component 3
 * @param q4 Quaternion component 4
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_quaternion_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Single public q1, Single public q2, Single public q3, Single public q4, Single public rollspeed, Single public pitchspeed, Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, q1);
	_mav_put_Single(buf, 8, q2);
	_mav_put_Single(buf, 12, q3);
	_mav_put_Single(buf, 16, q4);
	_mav_put_Single(buf, 20, rollspeed);
	_mav_put_Single(buf, 24, pitchspeed);
	_mav_put_Single(buf, 28, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, buf, 32, 246);
#else
    mavlink_attitude_quaternion_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.q1 = q1;
	packet.q2 = q2;
	packet.q3 = q3;
	packet.q4 = q4;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, (const char *)&packet, 32, 246);
#endif
}

#endif
*/
// MESSAGE ATTITUDE_QUATERNION UNPACKING


/**
 * @brief Get field time_boot_ms from attitude_quaternion message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_attitude_quaternion_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field q1 from attitude_quaternion message
 *
 * @return Quaternion component 1
 */
public static Single mavlink_msg_attitude_quaternion_get_q1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field q2 from attitude_quaternion message
 *
 * @return Quaternion component 2
 */
public static Single mavlink_msg_attitude_quaternion_get_q2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field q3 from attitude_quaternion message
 *
 * @return Quaternion component 3
 */
public static Single mavlink_msg_attitude_quaternion_get_q3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field q4 from attitude_quaternion message
 *
 * @return Quaternion component 4
 */
public static Single mavlink_msg_attitude_quaternion_get_q4(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field rollspeed from attitude_quaternion message
 *
 * @return Roll angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_quaternion_get_rollspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field pitchspeed from attitude_quaternion message
 *
 * @return Pitch angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_quaternion_get_pitchspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field yawspeed from attitude_quaternion message
 *
 * @return Yaw angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_quaternion_get_yawspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a attitude_quaternion message into a struct
 *
 * @param msg The message to decode
 * @param attitude_quaternion C-struct to decode the message contents into
 */
public static void mavlink_msg_attitude_quaternion_decode(byte[] msg, ref mavlink_attitude_quaternion_t attitude_quaternion)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	attitude_quaternion.time_boot_ms = mavlink_msg_attitude_quaternion_get_time_boot_ms(msg);
    	attitude_quaternion.q1 = mavlink_msg_attitude_quaternion_get_q1(msg);
    	attitude_quaternion.q2 = mavlink_msg_attitude_quaternion_get_q2(msg);
    	attitude_quaternion.q3 = mavlink_msg_attitude_quaternion_get_q3(msg);
    	attitude_quaternion.q4 = mavlink_msg_attitude_quaternion_get_q4(msg);
    	attitude_quaternion.rollspeed = mavlink_msg_attitude_quaternion_get_rollspeed(msg);
    	attitude_quaternion.pitchspeed = mavlink_msg_attitude_quaternion_get_pitchspeed(msg);
    	attitude_quaternion.yawspeed = mavlink_msg_attitude_quaternion_get_yawspeed(msg);
    
    } else {
        int len = 32; //Marshal.SizeOf(attitude_quaternion);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        attitude_quaternion = (mavlink_attitude_quaternion_t)Marshal.PtrToStructure(i, ((object)attitude_quaternion).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
