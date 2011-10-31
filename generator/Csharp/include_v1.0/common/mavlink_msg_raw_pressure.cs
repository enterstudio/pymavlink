// MESSAGE RAW_PRESSURE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RAW_PRESSURE = 28;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_raw_pressure_t
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public  UInt64 time_usec;
            /// <summary>
        /// Absolute pressure (raw)
        /// </summary>
        public  Int16 press_abs;
            /// <summary>
        /// Differential pressure 1 (raw)
        /// </summary>
        public  Int16 press_diff1;
            /// <summary>
        /// Differential pressure 2 (raw)
        /// </summary>
        public  Int16 press_diff2;
            /// <summary>
        /// Raw Temperature measurement (raw)
        /// </summary>
        public  Int16 temperature;
    
    };

/// <summary>
/// * @brief Pack a raw_pressure message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
/// * @param press_abs Absolute pressure (raw)
/// * @param press_diff1 Differential pressure 1 (raw)
/// * @param press_diff2 Differential pressure 2 (raw)
/// * @param temperature Raw Temperature measurement (raw)
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_raw_pressure_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec, Int16 press_abs, Int16 press_diff1, Int16 press_diff2, Int16 temperature)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(press_abs),0,msg,8,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(press_diff1),0,msg,10,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(press_diff2),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(temperature),0,msg,14,sizeof(Int16));

} else {
    mavlink_raw_pressure_t packet = new mavlink_raw_pressure_t();
	packet.time_usec = time_usec;
	packet.press_abs = press_abs;
	packet.press_diff1 = press_diff1;
	packet.press_diff2 = press_diff2;
	packet.temperature = temperature;

        
        int len = 16;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_RAW_PRESSURE;
    //return mavlink_finalize_message(msg, system_id, component_id, 16, 67);
    return 0;
}

/**
 * @brief Pack a raw_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (raw)
 * @param press_diff1 Differential pressure 1 (raw)
 * @param press_diff2 Differential pressure 2 (raw)
 * @param temperature Raw Temperature measurement (raw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_raw_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec,Int16 public press_abs,Int16 public press_diff1,Int16 public press_diff2,Int16 public temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Int16(buf, 8, press_abs);
	_mav_put_Int16(buf, 10, press_diff1);
	_mav_put_Int16(buf, 12, press_diff2);
	_mav_put_Int16(buf, 14, temperature);

        memcpy(_MAV_PAYLOAD(msg), buf, 16);
#else
    mavlink_raw_pressure_t packet;
	packet.time_usec = time_usec;
	packet.press_abs = press_abs;
	packet.press_diff1 = press_diff1;
	packet.press_diff2 = press_diff2;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD(msg), &packet, 16);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAW_PRESSURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16, 67);
}
*/
/**
 * @brief Encode a raw_pressure struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_pressure C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_raw_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_pressure_t* raw_pressure)
{
    return mavlink_msg_raw_pressure_pack(system_id, component_id, msg, raw_pressure->time_usec, raw_pressure->press_abs, raw_pressure->press_diff1, raw_pressure->press_diff2, raw_pressure->temperature);
}
*/
/**
 * @brief Send a raw_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (raw)
 * @param press_diff1 Differential pressure 1 (raw)
 * @param press_diff2 Differential pressure 2 (raw)
 * @param temperature Raw Temperature measurement (raw)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_pressure_send(mavlink_channel_t chan, UInt64 public time_usec, Int16 public press_abs, Int16 public press_diff1, Int16 public press_diff2, Int16 public temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[16];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Int16(buf, 8, press_abs);
	_mav_put_Int16(buf, 10, press_diff1);
	_mav_put_Int16(buf, 12, press_diff2);
	_mav_put_Int16(buf, 14, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, buf, 16, 67);
#else
    mavlink_raw_pressure_t packet;
	packet.time_usec = time_usec;
	packet.press_abs = press_abs;
	packet.press_diff1 = press_diff1;
	packet.press_diff2 = press_diff2;
	packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_PRESSURE, (const char *)&packet, 16, 67);
#endif
}

#endif
*/
// MESSAGE RAW_PRESSURE UNPACKING


/**
 * @brief Get field time_usec from raw_pressure message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_raw_pressure_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field press_abs from raw_pressure message
 *
 * @return Absolute pressure (raw)
 */
public static Int16 mavlink_msg_raw_pressure_get_press_abs(byte[] msg)
{
    return BitConverter.ToInt16(msg,  8);
}

/**
 * @brief Get field press_diff1 from raw_pressure message
 *
 * @return Differential pressure 1 (raw)
 */
public static Int16 mavlink_msg_raw_pressure_get_press_diff1(byte[] msg)
{
    return BitConverter.ToInt16(msg,  10);
}

/**
 * @brief Get field press_diff2 from raw_pressure message
 *
 * @return Differential pressure 2 (raw)
 */
public static Int16 mavlink_msg_raw_pressure_get_press_diff2(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field temperature from raw_pressure message
 *
 * @return Raw Temperature measurement (raw)
 */
public static Int16 mavlink_msg_raw_pressure_get_temperature(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Decode a raw_pressure message into a struct
 *
 * @param msg The message to decode
 * @param raw_pressure C-struct to decode the message contents into
 */
public static void mavlink_msg_raw_pressure_decode(byte[] msg, ref mavlink_raw_pressure_t raw_pressure)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	raw_pressure.time_usec = mavlink_msg_raw_pressure_get_time_usec(msg);
    	raw_pressure.press_abs = mavlink_msg_raw_pressure_get_press_abs(msg);
    	raw_pressure.press_diff1 = mavlink_msg_raw_pressure_get_press_diff1(msg);
    	raw_pressure.press_diff2 = mavlink_msg_raw_pressure_get_press_diff2(msg);
    	raw_pressure.temperature = mavlink_msg_raw_pressure_get_temperature(msg);
    
    } else {
        int len = 16; //Marshal.SizeOf(raw_pressure);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        raw_pressure = (mavlink_raw_pressure_t)Marshal.PtrToStructure(i, ((object)raw_pressure).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
