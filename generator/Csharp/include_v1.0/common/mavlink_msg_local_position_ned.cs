// MESSAGE LOCAL_POSITION_NED PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_local_position_ned_t
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// X Position
        /// </summary>
        public  Single x;
            /// <summary>
        /// Y Position
        /// </summary>
        public  Single y;
            /// <summary>
        /// Z Position
        /// </summary>
        public  Single z;
            /// <summary>
        /// X Speed
        /// </summary>
        public  Single vx;
            /// <summary>
        /// Y Speed
        /// </summary>
        public  Single vy;
            /// <summary>
        /// Z Speed
        /// </summary>
        public  Single vz;
    
    };

/// <summary>
/// * @brief Pack a local_position_ned message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp (milliseconds since system boot)
/// * @param x X Position
/// * @param y Y Position
/// * @param z Z Position
/// * @param vx X Speed
/// * @param vy Y Speed
/// * @param vz Z Speed
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_local_position_ned_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Single x, Single y, Single z, Single vx, Single vy, Single vz)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(x),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vx),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vy),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vz),0,msg,24,sizeof(Single));

} else {
    mavlink_local_position_ned_t packet = new mavlink_local_position_ned_t();
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        
        int len = 28;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
    //return mavlink_finalize_message(msg, system_id, component_id, 28, 185);
    return 0;
}

/**
 * @brief Pack a local_position_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_local_position_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,Single public x,Single public y,Single public z,Single public vx,Single public vy,Single public vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, x);
	_mav_put_Single(buf, 8, y);
	_mav_put_Single(buf, 12, z);
	_mav_put_Single(buf, 16, vx);
	_mav_put_Single(buf, 20, vy);
	_mav_put_Single(buf, 24, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
    mavlink_local_position_ned_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 185);
}
*/
/**
 * @brief Encode a local_position_ned struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_local_position_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_ned_t* local_position_ned)
{
    return mavlink_msg_local_position_ned_pack(system_id, component_id, msg, local_position_ned->time_boot_ms, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->vx, local_position_ned->vy, local_position_ned->vz);
}
*/
/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_ned_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Single public x, Single public y, Single public z, Single public vx, Single public vy, Single public vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, x);
	_mav_put_Single(buf, 8, y);
	_mav_put_Single(buf, 12, z);
	_mav_put_Single(buf, 16, vx);
	_mav_put_Single(buf, 20, vy);
	_mav_put_Single(buf, 24, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, buf, 28, 185);
#else
    mavlink_local_position_ned_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)&packet, 28, 185);
#endif
}

#endif
*/
// MESSAGE LOCAL_POSITION_NED UNPACKING


/**
 * @brief Get field time_boot_ms from local_position_ned message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_local_position_ned_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field x from local_position_ned message
 *
 * @return X Position
 */
public static Single mavlink_msg_local_position_ned_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field y from local_position_ned message
 *
 * @return Y Position
 */
public static Single mavlink_msg_local_position_ned_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field z from local_position_ned message
 *
 * @return Z Position
 */
public static Single mavlink_msg_local_position_ned_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field vx from local_position_ned message
 *
 * @return X Speed
 */
public static Single mavlink_msg_local_position_ned_get_vx(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field vy from local_position_ned message
 *
 * @return Y Speed
 */
public static Single mavlink_msg_local_position_ned_get_vy(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field vz from local_position_ned message
 *
 * @return Z Speed
 */
public static Single mavlink_msg_local_position_ned_get_vz(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Decode a local_position_ned message into a struct
 *
 * @param msg The message to decode
 * @param local_position_ned C-struct to decode the message contents into
 */
public static void mavlink_msg_local_position_ned_decode(byte[] msg, ref mavlink_local_position_ned_t local_position_ned)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	local_position_ned.time_boot_ms = mavlink_msg_local_position_ned_get_time_boot_ms(msg);
    	local_position_ned.x = mavlink_msg_local_position_ned_get_x(msg);
    	local_position_ned.y = mavlink_msg_local_position_ned_get_y(msg);
    	local_position_ned.z = mavlink_msg_local_position_ned_get_z(msg);
    	local_position_ned.vx = mavlink_msg_local_position_ned_get_vx(msg);
    	local_position_ned.vy = mavlink_msg_local_position_ned_get_vy(msg);
    	local_position_ned.vz = mavlink_msg_local_position_ned_get_vz(msg);
    
    } else {
        int len = 28; //Marshal.SizeOf(local_position_ned);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        local_position_ned = (mavlink_local_position_ned_t)Marshal.PtrToStructure(i, ((object)local_position_ned).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}