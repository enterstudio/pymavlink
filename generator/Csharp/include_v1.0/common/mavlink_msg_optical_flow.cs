// MESSAGE OPTICAL_FLOW PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_OPTICAL_FLOW = 100;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_optical_flow_t
    {
         public  UInt64 time_usec; /// Timestamp (UNIX)
     public  Single ground_distance; /// Ground distance in meters
     public  Int16 flow_x; /// Flow in pixels in x-sensor direction
     public  Int16 flow_y; /// Flow in pixels in y-sensor direction
     public  byte sensor_id; /// Sensor ID
     public  byte quality; /// Optical flow quality / confidence. 0: bad, 255: maximum quality
    
    };

/**
 * @brief Pack a optical_flow message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param flow_x Flow in pixels in x-sensor direction
 * @param flow_y Flow in pixels in y-sensor direction
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param ground_distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_optical_flow_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec, byte sensor_id, Int16 flow_x, Int16 flow_y, byte quality, Single ground_distance)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(ground_distance),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(flow_x),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(flow_y),0,msg,14,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(sensor_id),0,msg,16,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(quality),0,msg,17,sizeof(byte));

} else {
    mavlink_optical_flow_t packet = new mavlink_optical_flow_t();
	packet.time_usec = time_usec;
	packet.ground_distance = ground_distance;
	packet.flow_x = flow_x;
	packet.flow_y = flow_y;
	packet.sensor_id = sensor_id;
	packet.quality = quality;

        
        int len = 18;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
    //return mavlink_finalize_message(msg, system_id, component_id, 18, 19);
    return 0;
}

/**
 * @brief Pack a optical_flow message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param flow_x Flow in pixels in x-sensor direction
 * @param flow_y Flow in pixels in y-sensor direction
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param ground_distance Ground distance in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_optical_flow_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec,byte public sensor_id,Int16 public flow_x,Int16 public flow_y,byte public quality,Single public ground_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Single(buf, 8, ground_distance);
	_mav_put_Int16(buf, 12, flow_x);
	_mav_put_Int16(buf, 14, flow_y);
	_mav_put_byte(buf, 16, sensor_id);
	_mav_put_byte(buf, 17, quality);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_optical_flow_t packet;
	packet.time_usec = time_usec;
	packet.ground_distance = ground_distance;
	packet.flow_x = flow_x;
	packet.flow_y = flow_y;
	packet.sensor_id = sensor_id;
	packet.quality = quality;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_OPTICAL_FLOW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 19);
}
*/
/**
 * @brief Encode a optical_flow struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param optical_flow C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_optical_flow_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_optical_flow_t* optical_flow)
{
    return mavlink_msg_optical_flow_pack(system_id, component_id, msg, optical_flow->time_usec, optical_flow->sensor_id, optical_flow->flow_x, optical_flow->flow_y, optical_flow->quality, optical_flow->ground_distance);
}
*/
/**
 * @brief Send a optical_flow message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (UNIX)
 * @param sensor_id Sensor ID
 * @param flow_x Flow in pixels in x-sensor direction
 * @param flow_y Flow in pixels in y-sensor direction
 * @param quality Optical flow quality / confidence. 0: bad, 255: maximum quality
 * @param ground_distance Ground distance in meters
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_optical_flow_send(mavlink_channel_t chan, UInt64 public time_usec, byte public sensor_id, Int16 public flow_x, Int16 public flow_y, byte public quality, Single public ground_distance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Single(buf, 8, ground_distance);
	_mav_put_Int16(buf, 12, flow_x);
	_mav_put_Int16(buf, 14, flow_y);
	_mav_put_byte(buf, 16, sensor_id);
	_mav_put_byte(buf, 17, quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, buf, 18, 19);
#else
    mavlink_optical_flow_t packet;
	packet.time_usec = time_usec;
	packet.ground_distance = ground_distance;
	packet.flow_x = flow_x;
	packet.flow_y = flow_y;
	packet.sensor_id = sensor_id;
	packet.quality = quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OPTICAL_FLOW, (const char *)&packet, 18, 19);
#endif
}

#endif
*/
// MESSAGE OPTICAL_FLOW UNPACKING


/**
 * @brief Get field time_usec from optical_flow message
 *
 * @return Timestamp (UNIX)
 */
public static UInt64 mavlink_msg_optical_flow_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field sensor_id from optical_flow message
 *
 * @return Sensor ID
 */
public static byte mavlink_msg_optical_flow_get_sensor_id(byte[] msg)
{
    return getByte(msg,  16);
}

/**
 * @brief Get field flow_x from optical_flow message
 *
 * @return Flow in pixels in x-sensor direction
 */
public static Int16 mavlink_msg_optical_flow_get_flow_x(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field flow_y from optical_flow message
 *
 * @return Flow in pixels in y-sensor direction
 */
public static Int16 mavlink_msg_optical_flow_get_flow_y(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Get field quality from optical_flow message
 *
 * @return Optical flow quality / confidence. 0: bad, 255: maximum quality
 */
public static byte mavlink_msg_optical_flow_get_quality(byte[] msg)
{
    return getByte(msg,  17);
}

/**
 * @brief Get field ground_distance from optical_flow message
 *
 * @return Ground distance in meters
 */
public static Single mavlink_msg_optical_flow_get_ground_distance(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Decode a optical_flow message into a struct
 *
 * @param msg The message to decode
 * @param optical_flow C-struct to decode the message contents into
 */
public static void mavlink_msg_optical_flow_decode(byte[] msg, ref mavlink_optical_flow_t optical_flow)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	optical_flow.time_usec = mavlink_msg_optical_flow_get_time_usec(msg);
    	optical_flow.ground_distance = mavlink_msg_optical_flow_get_ground_distance(msg);
    	optical_flow.flow_x = mavlink_msg_optical_flow_get_flow_x(msg);
    	optical_flow.flow_y = mavlink_msg_optical_flow_get_flow_y(msg);
    	optical_flow.sensor_id = mavlink_msg_optical_flow_get_sensor_id(msg);
    	optical_flow.quality = mavlink_msg_optical_flow_get_quality(msg);
    
    } else {
        int len = 18; //Marshal.SizeOf(optical_flow);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        optical_flow = (mavlink_optical_flow_t)Marshal.PtrToStructure(i, ((object)optical_flow).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
