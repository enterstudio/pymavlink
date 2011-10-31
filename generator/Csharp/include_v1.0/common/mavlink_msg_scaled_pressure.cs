// MESSAGE SCALED_PRESSURE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SCALED_PRESSURE = 29;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_scaled_pressure_t
    {
        /// <summary>
        /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// Absolute pressure (hectopascal)
        /// </summary>
        public  Single press_abs;
            /// <summary>
        /// Differential pressure 1 (hectopascal)
        /// </summary>
        public  Single press_diff;
            /// <summary>
        /// Temperature measurement (0.01 degrees celsius)
        /// </summary>
        public  Int16 temperature;
    
    };

/// <summary>
/// * @brief Pack a scaled_pressure message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp (microseconds since UNIX epoch or microseconds since system boot)
/// * @param press_abs Absolute pressure (hectopascal)
/// * @param press_diff Differential pressure 1 (hectopascal)
/// * @param temperature Temperature measurement (0.01 degrees celsius)
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_scaled_pressure_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Single press_abs, Single press_diff, Int16 temperature)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(press_abs),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(press_diff),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(temperature),0,msg,12,sizeof(Int16));

} else {
    mavlink_scaled_pressure_t packet = new mavlink_scaled_pressure_t();
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.temperature = temperature;

        
        int len = 14;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;
    //return mavlink_finalize_message(msg, system_id, component_id, 14, 115);
    return 0;
}

/**
 * @brief Pack a scaled_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_scaled_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,Single public press_abs,Single public press_diff,Int16 public temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, press_abs);
	_mav_put_Single(buf, 8, press_diff);
	_mav_put_Int16(buf, 12, temperature);

        memcpy(_MAV_PAYLOAD(msg), buf, 14);
#else
    mavlink_scaled_pressure_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.temperature = temperature;

        memcpy(_MAV_PAYLOAD(msg), &packet, 14);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_PRESSURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 14, 115);
}
*/
/**
 * @brief Encode a scaled_pressure struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_pressure C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_scaled_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_pressure_t* scaled_pressure)
{
    return mavlink_msg_scaled_pressure_pack(system_id, component_id, msg, scaled_pressure->time_boot_ms, scaled_pressure->press_abs, scaled_pressure->press_diff, scaled_pressure->temperature);
}
*/
/**
 * @brief Send a scaled_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param press_abs Absolute pressure (hectopascal)
 * @param press_diff Differential pressure 1 (hectopascal)
 * @param temperature Temperature measurement (0.01 degrees celsius)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_pressure_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Single public press_abs, Single public press_diff, Int16 public temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[14];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Single(buf, 4, press_abs);
	_mav_put_Single(buf, 8, press_diff);
	_mav_put_Int16(buf, 12, temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, buf, 14, 115);
#else
    mavlink_scaled_pressure_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.press_abs = press_abs;
	packet.press_diff = press_diff;
	packet.temperature = temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_PRESSURE, (const char *)&packet, 14, 115);
#endif
}

#endif
*/
// MESSAGE SCALED_PRESSURE UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_pressure message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt32 mavlink_msg_scaled_pressure_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field press_abs from scaled_pressure message
 *
 * @return Absolute pressure (hectopascal)
 */
public static Single mavlink_msg_scaled_pressure_get_press_abs(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field press_diff from scaled_pressure message
 *
 * @return Differential pressure 1 (hectopascal)
 */
public static Single mavlink_msg_scaled_pressure_get_press_diff(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field temperature from scaled_pressure message
 *
 * @return Temperature measurement (0.01 degrees celsius)
 */
public static Int16 mavlink_msg_scaled_pressure_get_temperature(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Decode a scaled_pressure message into a struct
 *
 * @param msg The message to decode
 * @param scaled_pressure C-struct to decode the message contents into
 */
public static void mavlink_msg_scaled_pressure_decode(byte[] msg, ref mavlink_scaled_pressure_t scaled_pressure)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	scaled_pressure.time_boot_ms = mavlink_msg_scaled_pressure_get_time_boot_ms(msg);
    	scaled_pressure.press_abs = mavlink_msg_scaled_pressure_get_press_abs(msg);
    	scaled_pressure.press_diff = mavlink_msg_scaled_pressure_get_press_diff(msg);
    	scaled_pressure.temperature = mavlink_msg_scaled_pressure_get_temperature(msg);
    
    } else {
        int len = 14; //Marshal.SizeOf(scaled_pressure);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        scaled_pressure = (mavlink_scaled_pressure_t)Marshal.PtrToStructure(i, ((object)scaled_pressure).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
