// MESSAGE DATA_STREAM PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_DATA_STREAM = 67;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_data_stream_t
    {
         public  UInt16 message_rate; /// The requested interval between two messages of this type
     public  byte stream_id; /// The ID of the requested data stream
     public  byte on_off; /// 1 stream is enabled, 0 stream is stopped.
    
    };

/**
 * @brief Pack a data_stream message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param stream_id The ID of the requested data stream
 * @param message_rate The requested interval between two messages of this type
 * @param on_off 1 stream is enabled, 0 stream is stopped.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_data_stream_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public stream_id, UInt16 public message_rate, byte public on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[4];
	_mav_put_UInt16(buf, 0, message_rate);
	_mav_put_byte(buf, 2, stream_id);
	_mav_put_byte(buf, 3, on_off);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_data_stream_t packet;
	packet.message_rate = message_rate;
	packet.stream_id = stream_id;
	packet.on_off = on_off;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_STREAM;
    return mavlink_finalize_message(msg, system_id, component_id, 4, 21);
}
*/
/**
 * @brief Pack a data_stream message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param stream_id The ID of the requested data stream
 * @param message_rate The requested interval between two messages of this type
 * @param on_off 1 stream is enabled, 0 stream is stopped.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_data_stream_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public stream_id,UInt16 public message_rate,byte public on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, message_rate);
	_mav_put_byte(buf, 2, stream_id);
	_mav_put_byte(buf, 3, on_off);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_data_stream_t packet;
	packet.message_rate = message_rate;
	packet.stream_id = stream_id;
	packet.on_off = on_off;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_STREAM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 21);
}
*/
/**
 * @brief Encode a data_stream struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_stream C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_data_stream_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_stream_t* data_stream)
{
    return mavlink_msg_data_stream_pack(system_id, component_id, msg, data_stream->stream_id, data_stream->message_rate, data_stream->on_off);
}
*/
/**
 * @brief Send a data_stream message
 * @param chan MAVLink channel to send the message
 *
 * @param stream_id The ID of the requested data stream
 * @param message_rate The requested interval between two messages of this type
 * @param on_off 1 stream is enabled, 0 stream is stopped.
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_stream_send(mavlink_channel_t chan, byte public stream_id, UInt16 public message_rate, byte public on_off)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, message_rate);
	_mav_put_byte(buf, 2, stream_id);
	_mav_put_byte(buf, 3, on_off);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, buf, 4, 21);
#else
    mavlink_data_stream_t packet;
	packet.message_rate = message_rate;
	packet.stream_id = stream_id;
	packet.on_off = on_off;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_STREAM, (const char *)&packet, 4, 21);
#endif
}

#endif
*/
// MESSAGE DATA_STREAM UNPACKING


/**
 * @brief Get field stream_id from data_stream message
 *
 * @return The ID of the requested data stream
 */
public static byte mavlink_msg_data_stream_get_stream_id(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field message_rate from data_stream message
 *
 * @return The requested interval between two messages of this type
 */
public static UInt16 mavlink_msg_data_stream_get_message_rate(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field on_off from data_stream message
 *
 * @return 1 stream is enabled, 0 stream is stopped.
 */
public static byte mavlink_msg_data_stream_get_on_off(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Decode a data_stream message into a struct
 *
 * @param msg The message to decode
 * @param data_stream C-struct to decode the message contents into
 */
public static void mavlink_msg_data_stream_decode(byte[] msg, ref mavlink_data_stream_t data_stream)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	data_stream.message_rate = mavlink_msg_data_stream_get_message_rate(msg);
	data_stream.stream_id = mavlink_msg_data_stream_get_stream_id(msg);
	data_stream.on_off = mavlink_msg_data_stream_get_on_off(msg);
} else {
    int len = 4; //Marshal.SizeOf(data_stream);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    data_stream = (mavlink_data_stream_t)Marshal.PtrToStructure(i, ((object)data_stream).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
