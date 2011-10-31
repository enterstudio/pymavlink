// MESSAGE LOCAL_POSITION_SETPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT = 51;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_local_position_setpoint_t
    {
        /// <summary>
        /// x position
        /// </summary>
        public  Single x;
            /// <summary>
        /// y position
        /// </summary>
        public  Single y;
            /// <summary>
        /// z position
        /// </summary>
        public  Single z;
            /// <summary>
        /// Desired yaw angle
        /// </summary>
        public  Single yaw;
    
    };

/// <summary>
/// * @brief Pack a local_position_setpoint message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param x x position
/// * @param y y position
/// * @param z z position
/// * @param yaw Desired yaw angle
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_local_position_setpoint_pack(byte system_id, byte component_id, byte[] msg,
                               Single x, Single y, Single z, Single yaw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(x),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,12,sizeof(Single));

} else {
    mavlink_local_position_setpoint_t packet = new mavlink_local_position_setpoint_t();
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        
        int len = 16;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT;
    //return mavlink_finalize_message(msg, system_id, component_id, 16);
    return 0;
}

/**
 * @brief Pack a local_position_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_local_position_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
    mavlink_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16);
}
*/
/**
 * @brief Encode a local_position_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_setpoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_local_position_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_setpoint_t* local_position_setpoint)
{
    return mavlink_msg_local_position_setpoint_pack(system_id, component_id, msg, local_position_setpoint->x, local_position_setpoint->y, local_position_setpoint->z, local_position_setpoint->yaw);
}
*/
/**
 * @brief Send a local_position_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw Desired yaw angle
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_setpoint_send(mavlink_channel_t chan, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT, buf, 16);
#else
    mavlink_local_position_setpoint_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_SETPOINT, (const char *)&packet, 16);
#endif
}

#endif
*/
// MESSAGE LOCAL_POSITION_SETPOINT UNPACKING


/**
 * @brief Get field x from local_position_setpoint message
 *
 * @return x position
 */
public static Single mavlink_msg_local_position_setpoint_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from local_position_setpoint message
 *
 * @return y position
 */
public static Single mavlink_msg_local_position_setpoint_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from local_position_setpoint message
 *
 * @return z position
 */
public static Single mavlink_msg_local_position_setpoint_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from local_position_setpoint message
 *
 * @return Desired yaw angle
 */
public static Single mavlink_msg_local_position_setpoint_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a local_position_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param local_position_setpoint C-struct to decode the message contents into
 */
public static void mavlink_msg_local_position_setpoint_decode(byte[] msg, ref mavlink_local_position_setpoint_t local_position_setpoint)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	local_position_setpoint.x = mavlink_msg_local_position_setpoint_get_x(msg);
    	local_position_setpoint.y = mavlink_msg_local_position_setpoint_get_y(msg);
    	local_position_setpoint.z = mavlink_msg_local_position_setpoint_get_z(msg);
    	local_position_setpoint.yaw = mavlink_msg_local_position_setpoint_get_yaw(msg);
    
    } else {
        int len = 16; //Marshal.SizeOf(local_position_setpoint);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        local_position_setpoint = (mavlink_local_position_setpoint_t)Marshal.PtrToStructure(i, ((object)local_position_setpoint).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
