// MESSAGE MARKER PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MARKER = 171;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_marker_t
    {
         public  Single x; /// x position
     public  Single y; /// y position
     public  Single z; /// z position
     public  Single roll; /// roll orientation
     public  Single pitch; /// pitch orientation
     public  Single yaw; /// yaw orientation
     public  UInt16 id; /// ID
    
    };

/**
 * @brief Pack a marker message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param roll roll orientation
 * @param pitch pitch orientation
 * @param yaw yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_marker_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 id, Single x, Single y, Single z, Single roll, Single pitch, Single yaw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(x),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(roll),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(id),0,msg,24,sizeof(UInt16));

} else {
    mavlink_marker_t packet = new mavlink_marker_t();
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.id = id;

        
        int len = 26;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MARKER;
    //return mavlink_finalize_message(msg, system_id, component_id, 26, 249);
    return 0;
}

/**
 * @brief Pack a marker message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param id ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param roll roll orientation
 * @param pitch pitch orientation
 * @param yaw yaw orientation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_marker_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public id,Single public x,Single public y,Single public z,Single public roll,Single public pitch,Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, roll);
	_mav_put_Single(buf, 16, pitch);
	_mav_put_Single(buf, 20, yaw);
	_mav_put_UInt16(buf, 24, id);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_marker_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.id = id;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_MARKER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26, 249);
}
*/
/**
 * @brief Encode a marker struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param marker C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_marker_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_marker_t* marker)
{
    return mavlink_msg_marker_pack(system_id, component_id, msg, marker->id, marker->x, marker->y, marker->z, marker->roll, marker->pitch, marker->yaw);
}
*/
/**
 * @brief Send a marker message
 * @param chan MAVLink channel to send the message
 *
 * @param id ID
 * @param x x position
 * @param y y position
 * @param z z position
 * @param roll roll orientation
 * @param pitch pitch orientation
 * @param yaw yaw orientation
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_marker_send(mavlink_channel_t chan, UInt16 public id, Single public x, Single public y, Single public z, Single public roll, Single public pitch, Single public yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_Single(buf, 12, roll);
	_mav_put_Single(buf, 16, pitch);
	_mav_put_Single(buf, 20, yaw);
	_mav_put_UInt16(buf, 24, id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER, buf, 26, 249);
#else
    mavlink_marker_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.id = id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MARKER, (const char *)&packet, 26, 249);
#endif
}

#endif
*/
// MESSAGE MARKER UNPACKING


/**
 * @brief Get field id from marker message
 *
 * @return ID
 */
public static UInt16 mavlink_msg_marker_get_id(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  24);
}

/**
 * @brief Get field x from marker message
 *
 * @return x position
 */
public static Single mavlink_msg_marker_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from marker message
 *
 * @return y position
 */
public static Single mavlink_msg_marker_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from marker message
 *
 * @return z position
 */
public static Single mavlink_msg_marker_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field roll from marker message
 *
 * @return roll orientation
 */
public static Single mavlink_msg_marker_get_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field pitch from marker message
 *
 * @return pitch orientation
 */
public static Single mavlink_msg_marker_get_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field yaw from marker message
 *
 * @return yaw orientation
 */
public static Single mavlink_msg_marker_get_yaw(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Decode a marker message into a struct
 *
 * @param msg The message to decode
 * @param marker C-struct to decode the message contents into
 */
public static void mavlink_msg_marker_decode(byte[] msg, ref mavlink_marker_t marker)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	marker.x = mavlink_msg_marker_get_x(msg);
    	marker.y = mavlink_msg_marker_get_y(msg);
    	marker.z = mavlink_msg_marker_get_z(msg);
    	marker.roll = mavlink_msg_marker_get_roll(msg);
    	marker.pitch = mavlink_msg_marker_get_pitch(msg);
    	marker.yaw = mavlink_msg_marker_get_yaw(msg);
    	marker.id = mavlink_msg_marker_get_id(msg);
    
    } else {
        int len = 26; //Marshal.SizeOf(marker);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        marker = (mavlink_marker_t)Marshal.PtrToStructure(i, ((object)marker).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
