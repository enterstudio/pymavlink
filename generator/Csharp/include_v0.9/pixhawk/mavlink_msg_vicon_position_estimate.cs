// MESSAGE VICON_POSITION_ESTIMATE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE = 157;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_vicon_position_estimate_t
    {
        /// <summary>
        /// Timestamp (milliseconds)
        /// </summary>
        public  UInt64 usec;
            /// <summary>
        /// Global X position
        /// </summary>
        public  Single x;
            /// <summary>
        /// Global Y position
        /// </summary>
        public  Single y;
            /// <summary>
        /// Global Z position
        /// </summary>
        public  Single z;
            /// <summary>
        /// Roll angle in rad
        /// </summary>
        public  Single roll;
            /// <summary>
        /// Pitch angle in rad
        /// </summary>
        public  Single pitch;
            /// <summary>
        /// Yaw angle in rad
        /// </summary>
        public  Single yaw;
    
    };

/// <summary>
/// * @brief Pack a vicon_position_estimate message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param usec Timestamp (milliseconds)
/// * @param x Global X position
/// * @param y Global Y position
/// * @param z Global Z position
/// * @param roll Roll angle in rad
/// * @param pitch Pitch angle in rad
/// * @param yaw Yaw angle in rad
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_vicon_position_estimate_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 usec, Single x, Single y, Single z, Single roll, Single pitch, Single yaw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(x),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(roll),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,28,sizeof(Single));

} else {
    mavlink_vicon_position_estimate_t packet = new mavlink_vicon_position_estimate_t();
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        
        int len = 32;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
    //return mavlink_finalize_message(msg, system_id, component_id, 32);
    return 0;
}

/**
 * @brief Pack a vicon_position_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (milliseconds)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_vicon_position_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public x,Single public y,Single public z,Single public roll,Single public pitch,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, roll);
	_mav_put_Single(buf, 24, pitch);
	_mav_put_Single(buf, 28, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_vicon_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}
*/
/**
 * @brief Encode a vicon_position_estimate struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vicon_position_estimate C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_vicon_position_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vicon_position_estimate_t* vicon_position_estimate)
{
    return mavlink_msg_vicon_position_estimate_pack(system_id, component_id, msg, vicon_position_estimate->usec, vicon_position_estimate->x, vicon_position_estimate->y, vicon_position_estimate->z, vicon_position_estimate->roll, vicon_position_estimate->pitch, vicon_position_estimate->yaw);
}
*/
/**
 * @brief Send a vicon_position_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (milliseconds)
 * @param x Global X position
 * @param y Global Y position
 * @param z Global Z position
 * @param roll Roll angle in rad
 * @param pitch Pitch angle in rad
 * @param yaw Yaw angle in rad
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vicon_position_estimate_send(mavlink_channel_t chan, UInt64 public usec, Single public x, Single public y, Single public z, Single public roll, Single public pitch, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, roll);
	_mav_put_Single(buf, 24, pitch);
	_mav_put_Single(buf, 28, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE, buf, 32);
#else
    mavlink_vicon_position_estimate_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE, (const char *)&packet, 32);
#endif
}

#endif
*/
// MESSAGE VICON_POSITION_ESTIMATE UNPACKING


/**
 * @brief Get field usec from vicon_position_estimate message
 *
 * @return Timestamp (milliseconds)
 */
public static UInt64 mavlink_msg_vicon_position_estimate_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field x from vicon_position_estimate message
 *
 * @return Global X position
 */
public static Single mavlink_msg_vicon_position_estimate_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field y from vicon_position_estimate message
 *
 * @return Global Y position
 */
public static Single mavlink_msg_vicon_position_estimate_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field z from vicon_position_estimate message
 *
 * @return Global Z position
 */
public static Single mavlink_msg_vicon_position_estimate_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field roll from vicon_position_estimate message
 *
 * @return Roll angle in rad
 */
public static Single mavlink_msg_vicon_position_estimate_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field pitch from vicon_position_estimate message
 *
 * @return Pitch angle in rad
 */
public static Single mavlink_msg_vicon_position_estimate_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field yaw from vicon_position_estimate message
 *
 * @return Yaw angle in rad
 */
public static Single mavlink_msg_vicon_position_estimate_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a vicon_position_estimate message into a struct
 *
 * @param msg The message to decode
 * @param vicon_position_estimate C-struct to decode the message contents into
 */
public static void mavlink_msg_vicon_position_estimate_decode(byte[] msg, ref mavlink_vicon_position_estimate_t vicon_position_estimate)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	vicon_position_estimate.usec = mavlink_msg_vicon_position_estimate_get_usec(msg);
    	vicon_position_estimate.x = mavlink_msg_vicon_position_estimate_get_x(msg);
    	vicon_position_estimate.y = mavlink_msg_vicon_position_estimate_get_y(msg);
    	vicon_position_estimate.z = mavlink_msg_vicon_position_estimate_get_z(msg);
    	vicon_position_estimate.roll = mavlink_msg_vicon_position_estimate_get_roll(msg);
    	vicon_position_estimate.pitch = mavlink_msg_vicon_position_estimate_get_pitch(msg);
    	vicon_position_estimate.yaw = mavlink_msg_vicon_position_estimate_get_yaw(msg);
    
    } else {
        int len = 32; //Marshal.SizeOf(vicon_position_estimate);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        vicon_position_estimate = (mavlink_vicon_position_estimate_t)Marshal.PtrToStructure(i, ((object)vicon_position_estimate).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
