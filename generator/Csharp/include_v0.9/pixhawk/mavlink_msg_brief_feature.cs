// MESSAGE BRIEF_FEATURE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_BRIEF_FEATURE = 195;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_brief_feature_t
    {
         public  Single x; /// x position in m
     public  Single y; /// y position in m
     public  Single z; /// z position in m
     public  byte orientation_assignment; /// Orientation assignment 0: false, 1:true
     public  UInt16 size; /// Size in pixels
     public  UInt16 orientation; /// Orientation
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 public byte[] descriptor; /// Descriptor
     public  Single response; /// Harris operator response at this location
    
    };

/**
 * @brief Pack a brief_feature message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x x position in m
 * @param y y position in m
 * @param z z position in m
 * @param orientation_assignment Orientation assignment 0: false, 1:true
 * @param size Size in pixels
 * @param orientation Orientation
 * @param descriptor Descriptor
 * @param response Harris operator response at this location
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_brief_feature_pack(byte system_id, byte component_id, byte[] msg,
                               Single x, Single y, Single z, byte orientation_assignment, UInt16 size, UInt16 orientation, byte[] descriptor, Single response)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(x),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(orientation_assignment),0,msg,12,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(size),0,msg,13,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(orientation),0,msg,15,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(response),0,msg,49,sizeof(Single));
	//Array.Copy(descriptor,0,msg,17,32);
} else {
    mavlink_brief_feature_t packet = new mavlink_brief_feature_t();
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.orientation_assignment = orientation_assignment;
	packet.size = size;
	packet.orientation = orientation;
	packet.response = response;
	packet.descriptor = descriptor;
        
        int len = 53;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_BRIEF_FEATURE;
    //return mavlink_finalize_message(msg, system_id, component_id, 53);
    return 0;
}

/**
 * @brief Pack a brief_feature message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param x x position in m
 * @param y y position in m
 * @param z z position in m
 * @param orientation_assignment Orientation assignment 0: false, 1:true
 * @param size Size in pixels
 * @param orientation Orientation
 * @param descriptor Descriptor
 * @param response Harris operator response at this location
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_brief_feature_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public x,Single public y,Single public z,byte public orientation_assignment,UInt16 public size,UInt16 public orientation,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 publicdescriptor,Single public response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[53];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_byte(buf, 12, orientation_assignment);
	_mav_put_UInt16(buf, 13, size);
	_mav_put_UInt16(buf, 15, orientation);
	_mav_put_Single(buf, 49, response);
	_mav_put_byte[]_array(buf, 17, descriptor, 32);
        memcpy(_MAV_PAYLOAD(msg), buf, 53);
#else
    mavlink_brief_feature_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.orientation_assignment = orientation_assignment;
	packet.size = size;
	packet.orientation = orientation;
	packet.response = response;
	memcpy(packet.descriptor, descriptor, sizeof(byte[])*32);
        memcpy(_MAV_PAYLOAD(msg), &packet, 53);
#endif

    msg->msgid = MAVLINK_MSG_ID_BRIEF_FEATURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 53);
}
*/
/**
 * @brief Encode a brief_feature struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param brief_feature C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_brief_feature_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_brief_feature_t* brief_feature)
{
    return mavlink_msg_brief_feature_pack(system_id, component_id, msg, brief_feature->x, brief_feature->y, brief_feature->z, brief_feature->orientation_assignment, brief_feature->size, brief_feature->orientation, brief_feature->descriptor, brief_feature->response);
}
*/
/**
 * @brief Send a brief_feature message
 * @param chan MAVLink channel to send the message
 *
 * @param x x position in m
 * @param y y position in m
 * @param z z position in m
 * @param orientation_assignment Orientation assignment 0: false, 1:true
 * @param size Size in pixels
 * @param orientation Orientation
 * @param descriptor Descriptor
 * @param response Harris operator response at this location
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_brief_feature_send(mavlink_channel_t chan, Single public x, Single public y, Single public z, byte public orientation_assignment, UInt16 public size, UInt16 public orientation, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 publicdescriptor, Single public response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[53];
	_mav_put_Single(buf, 0, x);
	_mav_put_Single(buf, 4, y);
	_mav_put_Single(buf, 8, z);
	_mav_put_byte(buf, 12, orientation_assignment);
	_mav_put_UInt16(buf, 13, size);
	_mav_put_UInt16(buf, 15, orientation);
	_mav_put_Single(buf, 49, response);
	_mav_put_byte[]_array(buf, 17, descriptor, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BRIEF_FEATURE, buf, 53);
#else
    mavlink_brief_feature_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.orientation_assignment = orientation_assignment;
	packet.size = size;
	packet.orientation = orientation;
	packet.response = response;
	memcpy(packet.descriptor, descriptor, sizeof(byte[])*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BRIEF_FEATURE, (const char *)&packet, 53);
#endif
}

#endif
*/
// MESSAGE BRIEF_FEATURE UNPACKING


/**
 * @brief Get field x from brief_feature message
 *
 * @return x position in m
 */
public static Single mavlink_msg_brief_feature_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field y from brief_feature message
 *
 * @return y position in m
 */
public static Single mavlink_msg_brief_feature_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field z from brief_feature message
 *
 * @return z position in m
 */
public static Single mavlink_msg_brief_feature_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field orientation_assignment from brief_feature message
 *
 * @return Orientation assignment 0: false, 1:true
 */
public static byte mavlink_msg_brief_feature_get_orientation_assignment(byte[] msg)
{
    return getByte(msg,  12);
}

/**
 * @brief Get field size from brief_feature message
 *
 * @return Size in pixels
 */
public static UInt16 mavlink_msg_brief_feature_get_size(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  13);
}

/**
 * @brief Get field orientation from brief_feature message
 *
 * @return Orientation
 */
public static UInt16 mavlink_msg_brief_feature_get_orientation(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  15);
}

/**
 * @brief Get field descriptor from brief_feature message
 *
 * @return Descriptor
 */
public static byte[] mavlink_msg_brief_feature_get_descriptor(byte[] msg)
{
    return getBytes(msg, 32,  17);
}

/**
 * @brief Get field response from brief_feature message
 *
 * @return Harris operator response at this location
 */
public static Single mavlink_msg_brief_feature_get_response(byte[] msg)
{
    return BitConverter.ToSingle(msg,  49);
}

/**
 * @brief Decode a brief_feature message into a struct
 *
 * @param msg The message to decode
 * @param brief_feature C-struct to decode the message contents into
 */
public static void mavlink_msg_brief_feature_decode(byte[] msg, ref mavlink_brief_feature_t brief_feature)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	brief_feature.x = mavlink_msg_brief_feature_get_x(msg);
    	brief_feature.y = mavlink_msg_brief_feature_get_y(msg);
    	brief_feature.z = mavlink_msg_brief_feature_get_z(msg);
    	brief_feature.orientation_assignment = mavlink_msg_brief_feature_get_orientation_assignment(msg);
    	brief_feature.size = mavlink_msg_brief_feature_get_size(msg);
    	brief_feature.orientation = mavlink_msg_brief_feature_get_orientation(msg);
    	brief_feature.descriptor = mavlink_msg_brief_feature_get_descriptor(msg);
    	brief_feature.response = mavlink_msg_brief_feature_get_response(msg);
    
    } else {
        int len = 53; //Marshal.SizeOf(brief_feature);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        brief_feature = (mavlink_brief_feature_t)Marshal.PtrToStructure(i, ((object)brief_feature).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
