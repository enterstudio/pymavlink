// MESSAGE LOCAL_POSITION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_LOCAL_POSITION = 31;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_local_position_t
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public  UInt64 usec;
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
/// * @brief Pack a local_position message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
/// * @param x X Position
/// * @param y Y Position
/// * @param z Z Position
/// * @param vx X Speed
/// * @param vy Y Speed
/// * @param vz Z Speed
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_local_position_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 usec, Single x, Single y, Single z, Single vx, Single vy, Single vz)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(x),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vx),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vy),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(vz),0,msg,28,sizeof(Single));

} else {
    mavlink_local_position_t packet = new mavlink_local_position_t();
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        
        int len = 32;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_LOCAL_POSITION;
    //return mavlink_finalize_message(msg, system_id, component_id, 32);
    return 0;
}

/**
 * @brief Pack a local_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_local_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public usec,Single public x,Single public y,Single public z,Single public vx,Single public vy,Single public vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, vx);
	_mav_put_Single(buf, 24, vy);
	_mav_put_Single(buf, 28, vz);

        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_local_position_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}
*/
/**
 * @brief Encode a local_position struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_local_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_t* local_position)
{
    return mavlink_msg_local_position_pack(system_id, component_id, msg, local_position->usec, local_position->x, local_position->y, local_position->z, local_position->vx, local_position->vy, local_position->vz);
}
*/
/**
 * @brief Send a local_position message
 * @param chan MAVLink channel to send the message
 *
 * @param usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param x X Position
 * @param y Y Position
 * @param z Z Position
 * @param vx X Speed
 * @param vy Y Speed
 * @param vz Z Speed
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_send(mavlink_channel_t chan, UInt64 public usec, Single public x, Single public y, Single public z, Single public vx, Single public vy, Single public vz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];
	_mav_put_UInt64(buf, 0, usec);
	_mav_put_Single(buf, 8, x);
	_mav_put_Single(buf, 12, y);
	_mav_put_Single(buf, 16, z);
	_mav_put_Single(buf, 20, vx);
	_mav_put_Single(buf, 24, vy);
	_mav_put_Single(buf, 28, vz);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION, buf, 32);
#else
    mavlink_local_position_t packet;
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.vx = vx;
	packet.vy = vy;
	packet.vz = vz;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION, (const char *)&packet, 32);
#endif
}

#endif
*/
// MESSAGE LOCAL_POSITION UNPACKING


/**
 * @brief Get field usec from local_position message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_local_position_get_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field x from local_position message
 *
 * @return X Position
 */
public static Single mavlink_msg_local_position_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field y from local_position message
 *
 * @return Y Position
 */
public static Single mavlink_msg_local_position_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field z from local_position message
 *
 * @return Z Position
 */
public static Single mavlink_msg_local_position_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field vx from local_position message
 *
 * @return X Speed
 */
public static Single mavlink_msg_local_position_get_vx(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field vy from local_position message
 *
 * @return Y Speed
 */
public static Single mavlink_msg_local_position_get_vy(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field vz from local_position message
 *
 * @return Z Speed
 */
public static Single mavlink_msg_local_position_get_vz(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Decode a local_position message into a struct
 *
 * @param msg The message to decode
 * @param local_position C-struct to decode the message contents into
 */
public static void mavlink_msg_local_position_decode(byte[] msg, ref mavlink_local_position_t local_position)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	local_position.usec = mavlink_msg_local_position_get_usec(msg);
    	local_position.x = mavlink_msg_local_position_get_x(msg);
    	local_position.y = mavlink_msg_local_position_get_y(msg);
    	local_position.z = mavlink_msg_local_position_get_z(msg);
    	local_position.vx = mavlink_msg_local_position_get_vx(msg);
    	local_position.vy = mavlink_msg_local_position_get_vy(msg);
    	local_position.vz = mavlink_msg_local_position_get_vz(msg);
    
    } else {
        int len = 32; //Marshal.SizeOf(local_position);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        local_position = (mavlink_local_position_t)Marshal.PtrToStructure(i, ((object)local_position).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
