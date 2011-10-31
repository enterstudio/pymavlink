// MESSAGE POSITION_CONTROL_SETPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT = 170;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_position_control_setpoint_t
    {
        /// <summary>
        /// ID of waypoint, 0 for plain position
        /// </summary>
        public  UInt16 id;
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
        /// yaw orientation in radians, 0 = NORTH
        /// </summary>
        public  Single yaw;
    
    };

/// <summary>
/// * @brief Pack a position_control_setpoint message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param id ID of waypoint, 0 for plain position
/// * @param x x position
/// * @param y y position
/// * @param z z position
/// * @param yaw yaw orientation in radians, 0 = NORTH
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_position_control_setpoint_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 id, Single x, Single y, Single z, Single yaw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(id),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(x),0,msg,2,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,6,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,10,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,14,sizeof(Single));

} else {
    mavlink_position_control_setpoint_t packet = new mavlink_position_control_setpoint_t();
	packet.id = id;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        
        int len = 18;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;
    //return mavlink_finalize_message(msg, system_id, component_id, 18);
    return 0;
}

/**
 * @brief Pack a position_control_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_position_control_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public id,Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt16(buf, 0, id);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, yaw);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_position_control_setpoint_t packet;
	packet.id = id;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18);
}
*/
/**
 * @brief Encode a position_control_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_control_setpoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_position_control_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_setpoint_t* position_control_setpoint)
{
    return mavlink_msg_position_control_setpoint_pack(system_id, component_id, msg, position_control_setpoint->id, position_control_setpoint->x, position_control_setpoint->y, position_control_setpoint->z, position_control_setpoint->yaw);
}
*/
/**
 * @brief Send a position_control_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_setpoint_send(mavlink_channel_t chan, UInt16 public id, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt16(buf, 0, id);
	_mav_put_Single(buf, 2, x);
	_mav_put_Single(buf, 6, y);
	_mav_put_Single(buf, 10, z);
	_mav_put_Single(buf, 14, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT, buf, 18);
#else
    mavlink_position_control_setpoint_t packet;
	packet.id = id;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT, (const char *)&packet, 18);
#endif
}

#endif
*/
// MESSAGE POSITION_CONTROL_SETPOINT UNPACKING


/**
 * @brief Get field id from position_control_setpoint message
 *
 * @return ID of waypoint, 0 for plain position
 */
public static UInt16 mavlink_msg_position_control_setpoint_get_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field x from position_control_setpoint message
 *
 * @return x position
 */
public static Single mavlink_msg_position_control_setpoint_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  2);
}

/**
 * @brief Get field y from position_control_setpoint message
 *
 * @return y position
 */
public static Single mavlink_msg_position_control_setpoint_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  6);
}

/**
 * @brief Get field z from position_control_setpoint message
 *
 * @return z position
 */
public static Single mavlink_msg_position_control_setpoint_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  10);
}

/**
 * @brief Get field yaw from position_control_setpoint message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
public static Single mavlink_msg_position_control_setpoint_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  14);
}

/**
 * @brief Decode a position_control_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param position_control_setpoint C-struct to decode the message contents into
 */
public static void mavlink_msg_position_control_setpoint_decode(byte[] msg, ref mavlink_position_control_setpoint_t position_control_setpoint)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	position_control_setpoint.id = mavlink_msg_position_control_setpoint_get_id(msg);
    	position_control_setpoint.x = mavlink_msg_position_control_setpoint_get_x(msg);
    	position_control_setpoint.y = mavlink_msg_position_control_setpoint_get_y(msg);
    	position_control_setpoint.z = mavlink_msg_position_control_setpoint_get_z(msg);
    	position_control_setpoint.yaw = mavlink_msg_position_control_setpoint_get_yaw(msg);
    
    } else {
        int len = 18; //Marshal.SizeOf(position_control_setpoint);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        position_control_setpoint = (mavlink_position_control_setpoint_t)Marshal.PtrToStructure(i, ((object)position_control_setpoint).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
