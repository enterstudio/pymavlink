// MESSAGE DATA_LOG PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_DATA_LOG = 177;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_data_log_t
    {
         public  Single fl_1; /// Log value 1 
     public  Single fl_2; /// Log value 2 
     public  Single fl_3; /// Log value 3 
     public  Single fl_4; /// Log value 4 
     public  Single fl_5; /// Log value 5 
     public  Single fl_6; /// Log value 6 
    
    };

/**
 * @brief Pack a data_log message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_data_log_pack(byte system_id, byte component_id, ref byte[] msg,
                               Single public fl_1, Single public fl_2, Single public fl_3, Single public fl_4, Single public fl_5, Single public fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[24];
	_mav_put_Single(buf, 0, fl_1);
	_mav_put_Single(buf, 4, fl_2);
	_mav_put_Single(buf, 8, fl_3);
	_mav_put_Single(buf, 12, fl_4);
	_mav_put_Single(buf, 16, fl_5);
	_mav_put_Single(buf, 20, fl_6);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
    mavlink_data_log_t packet;
	packet.fl_1 = fl_1;
	packet.fl_2 = fl_2;
	packet.fl_3 = fl_3;
	packet.fl_4 = fl_4;
	packet.fl_5 = fl_5;
	packet.fl_6 = fl_6;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_LOG;
    return mavlink_finalize_message(msg, system_id, component_id, 24);
}
*/
/**
 * @brief Pack a data_log message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_data_log_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public fl_1,Single public fl_2,Single public fl_3,Single public fl_4,Single public fl_5,Single public fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[24];
	_mav_put_Single(buf, 0, fl_1);
	_mav_put_Single(buf, 4, fl_2);
	_mav_put_Single(buf, 8, fl_3);
	_mav_put_Single(buf, 12, fl_4);
	_mav_put_Single(buf, 16, fl_5);
	_mav_put_Single(buf, 20, fl_6);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
    mavlink_data_log_t packet;
	packet.fl_1 = fl_1;
	packet.fl_2 = fl_2;
	packet.fl_3 = fl_3;
	packet.fl_4 = fl_4;
	packet.fl_5 = fl_5;
	packet.fl_6 = fl_6;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

    msg->msgid = MAVLINK_MSG_ID_DATA_LOG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24);
}
*/
/**
 * @brief Encode a data_log struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param data_log C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_data_log_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_data_log_t* data_log)
{
    return mavlink_msg_data_log_pack(system_id, component_id, msg, data_log->fl_1, data_log->fl_2, data_log->fl_3, data_log->fl_4, data_log->fl_5, data_log->fl_6);
}
*/
/**
 * @brief Send a data_log message
 * @param chan MAVLink channel to send the message
 *
 * @param fl_1 Log value 1 
 * @param fl_2 Log value 2 
 * @param fl_3 Log value 3 
 * @param fl_4 Log value 4 
 * @param fl_5 Log value 5 
 * @param fl_6 Log value 6 
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_data_log_send(mavlink_channel_t chan, Single public fl_1, Single public fl_2, Single public fl_3, Single public fl_4, Single public fl_5, Single public fl_6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[24];
	_mav_put_Single(buf, 0, fl_1);
	_mav_put_Single(buf, 4, fl_2);
	_mav_put_Single(buf, 8, fl_3);
	_mav_put_Single(buf, 12, fl_4);
	_mav_put_Single(buf, 16, fl_5);
	_mav_put_Single(buf, 20, fl_6);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, buf, 24);
#else
    mavlink_data_log_t packet;
	packet.fl_1 = fl_1;
	packet.fl_2 = fl_2;
	packet.fl_3 = fl_3;
	packet.fl_4 = fl_4;
	packet.fl_5 = fl_5;
	packet.fl_6 = fl_6;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DATA_LOG, (const char *)&packet, 24);
#endif
}

#endif
*/
// MESSAGE DATA_LOG UNPACKING


/**
 * @brief Get field fl_1 from data_log message
 *
 * @return Log value 1 
 */
public static Single mavlink_msg_data_log_get_fl_1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field fl_2 from data_log message
 *
 * @return Log value 2 
 */
public static Single mavlink_msg_data_log_get_fl_2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field fl_3 from data_log message
 *
 * @return Log value 3 
 */
public static Single mavlink_msg_data_log_get_fl_3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field fl_4 from data_log message
 *
 * @return Log value 4 
 */
public static Single mavlink_msg_data_log_get_fl_4(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field fl_5 from data_log message
 *
 * @return Log value 5 
 */
public static Single mavlink_msg_data_log_get_fl_5(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field fl_6 from data_log message
 *
 * @return Log value 6 
 */
public static Single mavlink_msg_data_log_get_fl_6(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Decode a data_log message into a struct
 *
 * @param msg The message to decode
 * @param data_log C-struct to decode the message contents into
 */
public static void mavlink_msg_data_log_decode(byte[] msg, ref mavlink_data_log_t data_log)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	data_log.fl_1 = mavlink_msg_data_log_get_fl_1(msg);
	data_log.fl_2 = mavlink_msg_data_log_get_fl_2(msg);
	data_log.fl_3 = mavlink_msg_data_log_get_fl_3(msg);
	data_log.fl_4 = mavlink_msg_data_log_get_fl_4(msg);
	data_log.fl_5 = mavlink_msg_data_log_get_fl_5(msg);
	data_log.fl_6 = mavlink_msg_data_log_get_fl_6(msg);
} else {
    int len = 24; //Marshal.SizeOf(data_log);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    data_log = (mavlink_data_log_t)Marshal.PtrToStructure(i, ((object)data_log).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
