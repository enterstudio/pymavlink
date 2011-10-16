// MESSAGE REQUEST_DATA_STREAM PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_request_data_stream_t
    {
         public  byte target_system; /// The target requested to send the message stream.
     public  byte target_component; /// The target requested to send the message stream.
     public  byte req_stream_id; /// The ID of the requested message type
     public  UInt16 req_message_rate; /// Update rate in Hertz
     public  byte start_stop; /// 1 to start sending, 0 to stop sending.
    
    };

/**
 * @brief Pack a request_data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested message type
 * @param req_message_rate Update rate in Hertz
 * @param start_stop 1 to start sending, 0 to stop sending.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_request_data_stream_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, byte req_stream_id, UInt16 req_message_rate, byte start_stop)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(req_stream_id),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(req_message_rate),0,msg,3,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(start_stop),0,msg,5,sizeof(byte));

} else {
    mavlink_request_data_stream_t packet = new mavlink_request_data_stream_t();
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.req_stream_id = req_stream_id;
	packet.req_message_rate = req_message_rate;
	packet.start_stop = start_stop;

        
        int len = 6;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;
    //return mavlink_finalize_message(msg, system_id, component_id, 6);
    return 0;
}

/**
 * @brief Pack a request_data_stream message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested message type
 * @param req_message_rate Update rate in Hertz
 * @param start_stop 1 to start sending, 0 to stop sending.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_request_data_stream_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,byte public req_stream_id,UInt16 public req_message_rate,byte public start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[6];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, req_stream_id);
	_mav_put_UInt16(buf, 3, req_message_rate);
	_mav_put_byte(buf, 5, start_stop);

        memcpy(_MAV_PAYLOAD(msg), buf, 6);
#else
    mavlink_request_data_stream_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.req_stream_id = req_stream_id;
	packet.req_message_rate = req_message_rate;
	packet.start_stop = start_stop;

        memcpy(_MAV_PAYLOAD(msg), &packet, 6);
#endif

    msg->msgid = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6);
}
*/
/**
 * @brief Encode a request_data_stream struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_data_stream C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_request_data_stream_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_data_stream_t* request_data_stream)
{
    return mavlink_msg_request_data_stream_pack(system_id, component_id, msg, request_data_stream->target_system, request_data_stream->target_component, request_data_stream->req_stream_id, request_data_stream->req_message_rate, request_data_stream->start_stop);
}
*/
/**
 * @brief Send a request_data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system The target requested to send the message stream.
 * @param target_component The target requested to send the message stream.
 * @param req_stream_id The ID of the requested message type
 * @param req_message_rate Update rate in Hertz
 * @param start_stop 1 to start sending, 0 to stop sending.
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_data_stream_send(mavlink_channel_t chan, byte public target_system, byte public target_component, byte public req_stream_id, UInt16 public req_message_rate, byte public start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[6];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, req_stream_id);
	_mav_put_UInt16(buf, 3, req_message_rate);
	_mav_put_byte(buf, 5, start_stop);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, buf, 6);
#else
    mavlink_request_data_stream_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.req_stream_id = req_stream_id;
	packet.req_message_rate = req_message_rate;
	packet.start_stop = start_stop;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, (const char *)&packet, 6);
#endif
}

#endif
*/
// MESSAGE REQUEST_DATA_STREAM UNPACKING


/**
 * @brief Get field target_system from request_data_stream message
 *
 * @return The target requested to send the message stream.
 */
public static byte mavlink_msg_request_data_stream_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from request_data_stream message
 *
 * @return The target requested to send the message stream.
 */
public static byte mavlink_msg_request_data_stream_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field req_stream_id from request_data_stream message
 *
 * @return The ID of the requested message type
 */
public static byte mavlink_msg_request_data_stream_get_req_stream_id(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field req_message_rate from request_data_stream message
 *
 * @return Update rate in Hertz
 */
public static UInt16 mavlink_msg_request_data_stream_get_req_message_rate(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  3);
}

/**
 * @brief Get field start_stop from request_data_stream message
 *
 * @return 1 to start sending, 0 to stop sending.
 */
public static byte mavlink_msg_request_data_stream_get_start_stop(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Decode a request_data_stream message into a struct
 *
 * @param msg The message to decode
 * @param request_data_stream C-struct to decode the message contents into
 */
public static void mavlink_msg_request_data_stream_decode(byte[] msg, ref mavlink_request_data_stream_t request_data_stream)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	request_data_stream.target_system = mavlink_msg_request_data_stream_get_target_system(msg);
    	request_data_stream.target_component = mavlink_msg_request_data_stream_get_target_component(msg);
    	request_data_stream.req_stream_id = mavlink_msg_request_data_stream_get_req_stream_id(msg);
    	request_data_stream.req_message_rate = mavlink_msg_request_data_stream_get_req_message_rate(msg);
    	request_data_stream.start_stop = mavlink_msg_request_data_stream_get_start_stop(msg);
    
    } else {
        int len = 6; //Marshal.SizeOf(request_data_stream);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        request_data_stream = (mavlink_request_data_stream_t)Marshal.PtrToStructure(i, ((object)request_data_stream).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
