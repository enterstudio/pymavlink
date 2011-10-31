// MESSAGE POSITION_CONTROL_SETPOINT_SET PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET = 159;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_position_control_setpoint_set_t
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
        /// yaw orientation in radians, 0 = NORTH
        /// </summary>
        public  Single yaw;
            /// <summary>
        /// ID of waypoint, 0 for plain position
        /// </summary>
        public  UInt16 id;
            /// <summary>
        /// System ID
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component ID
        /// </summary>
        public  byte target_component;
    
    };

/// <summary>
/// * @brief Pack a position_control_setpoint_set message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param id ID of waypoint, 0 for plain position
/// * @param x x position
/// * @param y y position
/// * @param z z position
/// * @param yaw yaw orientation in radians, 0 = NORTH
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_position_control_setpoint_set_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, UInt16 id, Single x, Single y, Single z, Single yaw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(x),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(id),0,msg,16,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,18,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,19,sizeof(byte));

} else {
    mavlink_position_control_setpoint_set_t packet = new mavlink_position_control_setpoint_set_t();
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;
	packet.target_system = target_system;
	packet.target_component = target_component;

        
        int len = 20;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET;
    //return mavlink_finalize_message(msg, system_id, component_id, 20, 11);
    return 0;
}

/**
 * @brief Pack a position_control_setpoint_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_position_control_setpoint_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,UInt16 public id,Single public x,Single public y,Single public z,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_UInt16(buf, 16, id);
	_mav_put_byte(buf, 18, target_system);
	_mav_put_byte(buf, 19, target_component);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_position_control_setpoint_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 11);
}
*/
/**
 * @brief Encode a position_control_setpoint_set struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param position_control_setpoint_set C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_position_control_setpoint_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_position_control_setpoint_set_t* position_control_setpoint_set)
{
    return mavlink_msg_position_control_setpoint_set_pack(system_id, component_id, msg, position_control_setpoint_set->target_system, position_control_setpoint_set->target_component, position_control_setpoint_set->id, position_control_setpoint_set->x, position_control_setpoint_set->y, position_control_setpoint_set->z, position_control_setpoint_set->yaw);
}
*/
/**
 * @brief Send a position_control_setpoint_set message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param id ID of waypoint, 0 for plain position
 * @param x x position
 * @param y y position
 * @param z z position
 * @param yaw yaw orientation in radians, 0 = NORTH
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_position_control_setpoint_set_send(mavlink_channel_t chan, byte public target_system, byte public target_component, UInt16 public id, Single public x, Single public y, Single public z, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, yaw);
	_mav_put_UInt16(buf, 16, id);
	_mav_put_byte(buf, 18, target_system);
	_mav_put_byte(buf, 19, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET, buf, 20, 11);
#else
    mavlink_position_control_setpoint_set_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.yaw = yaw;
	packet.id = id;
	packet.target_system = target_system;
	packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSITION_CONTROL_SETPOINT_SET, (const char *)&packet, 20, 11);
#endif
}

#endif
*/
// MESSAGE POSITION_CONTROL_SETPOINT_SET UNPACKING


/**
 * @brief Get field target_system from position_control_setpoint_set message
 *
 * @return System ID
 */
public static byte mavlink_msg_position_control_setpoint_set_get_target_system(byte[] msg)
{
    return getByte(msg,  18);
}

/**
 * @brief Get field target_component from position_control_setpoint_set message
 *
 * @return Component ID
 */
public static byte mavlink_msg_position_control_setpoint_set_get_target_component(byte[] msg)
{
    return getByte(msg,  19);
}

/**
 * @brief Get field id from position_control_setpoint_set message
 *
 * @return ID of waypoint, 0 for plain position
 */
public static UInt16 mavlink_msg_position_control_setpoint_set_get_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  16);
}

/**
 * @brief Get field x from position_control_setpoint_set message
 *
 * @return x position
 */
public static Single mavlink_msg_position_control_setpoint_set_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from position_control_setpoint_set message
 *
 * @return y position
 */
public static Single mavlink_msg_position_control_setpoint_set_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from position_control_setpoint_set message
 *
 * @return z position
 */
public static Single mavlink_msg_position_control_setpoint_set_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field yaw from position_control_setpoint_set message
 *
 * @return yaw orientation in radians, 0 = NORTH
 */
public static Single mavlink_msg_position_control_setpoint_set_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a position_control_setpoint_set message into a struct
 *
 * @param msg The message to decode
 * @param position_control_setpoint_set C-struct to decode the message contents into
 */
public static void mavlink_msg_position_control_setpoint_set_decode(byte[] msg, ref mavlink_position_control_setpoint_set_t position_control_setpoint_set)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	position_control_setpoint_set.x = mavlink_msg_position_control_setpoint_set_get_x(msg);
    	position_control_setpoint_set.y = mavlink_msg_position_control_setpoint_set_get_y(msg);
    	position_control_setpoint_set.z = mavlink_msg_position_control_setpoint_set_get_z(msg);
    	position_control_setpoint_set.yaw = mavlink_msg_position_control_setpoint_set_get_yaw(msg);
    	position_control_setpoint_set.id = mavlink_msg_position_control_setpoint_set_get_id(msg);
    	position_control_setpoint_set.target_system = mavlink_msg_position_control_setpoint_set_get_target_system(msg);
    	position_control_setpoint_set.target_component = mavlink_msg_position_control_setpoint_set_get_target_component(msg);
    
    } else {
        int len = 20; //Marshal.SizeOf(position_control_setpoint_set);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        position_control_setpoint_set = (mavlink_position_control_setpoint_set_t)Marshal.PtrToStructure(i, ((object)position_control_setpoint_set).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
