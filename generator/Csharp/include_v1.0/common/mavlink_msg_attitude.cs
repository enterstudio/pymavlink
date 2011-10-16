// MESSAGE ATTITUDE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ATTITUDE = 30;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_attitude_t
    {
         public  UInt32 time_boot_ms; /// Timestamp (milliseconds since system boot)
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
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_attitude_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Single roll, Single pitch, Single yaw, Single rollspeed, Single pitchspeed, Single yawspeed)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(roll),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(rollspeed),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitchspeed),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yawspeed),0,msg,24,sizeof(Single));

} else {
    mavlink_attitude_t packet = new mavlink_attitude_t();
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        
        int len = 28;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_ATTITUDE;
    //return mavlink_finalize_message(msg, system_id, component_id, 28, 39);
    return 0;
}

/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
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
                                   UInt32 public time_boot_ms,Single public roll,Single public pitch,Single public yaw,Single public rollspeed,Single public pitchspeed,Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, roll);
	_mav_put_Single(buf, 8, pitch);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_Single(buf, 16, rollspeed);
	_mav_put_Single(buf, 20, pitchspeed);
	_mav_put_Single(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
    mavlink_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 39);
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
    return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->time_boot_ms, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
}
*/
/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad)
 * @param pitch Pitch angle (rad)
 * @param yaw Yaw angle (rad)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Single public roll, Single public pitch, Single public yaw, Single public rollspeed, Single public pitchspeed, Single public yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, roll);
	_mav_put_Single(buf, 8, pitch);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_Single(buf, 16, rollspeed);
	_mav_put_Single(buf, 20, pitchspeed);
	_mav_put_Single(buf, 24, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, 28, 39);
#else
    mavlink_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)&packet, 28, 39);
#endif
}

#endif
*/
// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field time_boot_ms from attitude message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_attitude_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field roll from attitude message
 *
 * @return Roll angle (rad)
 */
public static Single mavlink_msg_attitude_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return Pitch angle (rad)
 */
public static Single mavlink_msg_attitude_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return Yaw angle (rad)
 */
public static Single mavlink_msg_attitude_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field rollspeed from attitude message
 *
 * @return Roll angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_get_rollspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field pitchspeed from attitude message
 *
 * @return Pitch angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_get_pitchspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field yawspeed from attitude message
 *
 * @return Yaw angular speed (rad/s)
 */
public static Single mavlink_msg_attitude_get_yawspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
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
    	attitude.time_boot_ms = mavlink_msg_attitude_get_time_boot_ms(msg);
    	attitude.roll = mavlink_msg_attitude_get_roll(msg);
    	attitude.pitch = mavlink_msg_attitude_get_pitch(msg);
    	attitude.yaw = mavlink_msg_attitude_get_yaw(msg);
    	attitude.rollspeed = mavlink_msg_attitude_get_rollspeed(msg);
    	attitude.pitchspeed = mavlink_msg_attitude_get_pitchspeed(msg);
    	attitude.yawspeed = mavlink_msg_attitude_get_yawspeed(msg);
    
    } else {
        int len = 28; //Marshal.SizeOf(attitude);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        attitude = (mavlink_attitude_t)Marshal.PtrToStructure(i, ((object)attitude).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
