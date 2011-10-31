// MESSAGE SENSOR_OFFSETS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SENSOR_OFFSETS = 150;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_sensor_offsets_t
    {
        /// <summary>
        /// magnetic declination (radians)
        /// </summary>
        public  Single mag_declination;
            /// <summary>
        /// raw pressure from barometer
        /// </summary>
        public  Int32 raw_press;
            /// <summary>
        /// raw temperature from barometer
        /// </summary>
        public  Int32 raw_temp;
            /// <summary>
        /// gyro X calibration
        /// </summary>
        public  Single gyro_cal_x;
            /// <summary>
        /// gyro Y calibration
        /// </summary>
        public  Single gyro_cal_y;
            /// <summary>
        /// gyro Z calibration
        /// </summary>
        public  Single gyro_cal_z;
            /// <summary>
        /// accel X calibration
        /// </summary>
        public  Single accel_cal_x;
            /// <summary>
        /// accel Y calibration
        /// </summary>
        public  Single accel_cal_y;
            /// <summary>
        /// accel Z calibration
        /// </summary>
        public  Single accel_cal_z;
            /// <summary>
        /// magnetometer X offset
        /// </summary>
        public  Int16 mag_ofs_x;
            /// <summary>
        /// magnetometer Y offset
        /// </summary>
        public  Int16 mag_ofs_y;
            /// <summary>
        /// magnetometer Z offset
        /// </summary>
        public  Int16 mag_ofs_z;
    
    };

/// <summary>
/// * @brief Pack a sensor_offsets message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param mag_ofs_x magnetometer X offset
/// * @param mag_ofs_y magnetometer Y offset
/// * @param mag_ofs_z magnetometer Z offset
/// * @param mag_declination magnetic declination (radians)
/// * @param raw_press raw pressure from barometer
/// * @param raw_temp raw temperature from barometer
/// * @param gyro_cal_x gyro X calibration
/// * @param gyro_cal_y gyro Y calibration
/// * @param gyro_cal_z gyro Z calibration
/// * @param accel_cal_x accel X calibration
/// * @param accel_cal_y accel Y calibration
/// * @param accel_cal_z accel Z calibration
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_sensor_offsets_pack(byte system_id, byte component_id, byte[] msg,
                               Int16 mag_ofs_x, Int16 mag_ofs_y, Int16 mag_ofs_z, Single mag_declination, Int32 raw_press, Int32 raw_temp, Single gyro_cal_x, Single gyro_cal_y, Single gyro_cal_z, Single accel_cal_x, Single accel_cal_y, Single accel_cal_z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(mag_declination),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(raw_press),0,msg,4,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(raw_temp),0,msg,8,sizeof(Int32));
	Array.Copy(BitConverter.GetBytes(gyro_cal_x),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(gyro_cal_y),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(gyro_cal_z),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(accel_cal_x),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(accel_cal_y),0,msg,28,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(accel_cal_z),0,msg,32,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(mag_ofs_x),0,msg,36,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(mag_ofs_y),0,msg,38,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(mag_ofs_z),0,msg,40,sizeof(Int16));

} else {
    mavlink_sensor_offsets_t packet = new mavlink_sensor_offsets_t();
	packet.mag_declination = mag_declination;
	packet.raw_press = raw_press;
	packet.raw_temp = raw_temp;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.accel_cal_x = accel_cal_x;
	packet.accel_cal_y = accel_cal_y;
	packet.accel_cal_z = accel_cal_z;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

        
        int len = 42;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SENSOR_OFFSETS;
    //return mavlink_finalize_message(msg, system_id, component_id, 42, 134);
    return 0;
}

/**
 * @brief Pack a sensor_offsets message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @param mag_declination magnetic declination (radians)
 * @param raw_press raw pressure from barometer
 * @param raw_temp raw temperature from barometer
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @param accel_cal_x accel X calibration
 * @param accel_cal_y accel Y calibration
 * @param accel_cal_z accel Z calibration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_sensor_offsets_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Int16 public mag_ofs_x,Int16 public mag_ofs_y,Int16 public mag_ofs_z,Single public mag_declination,Int32 public raw_press,Int32 public raw_temp,Single public gyro_cal_x,Single public gyro_cal_y,Single public gyro_cal_z,Single public accel_cal_x,Single public accel_cal_y,Single public accel_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[42];
	_mav_put_Single(buf, 0, mag_declination);
	_mav_put_Int32(buf, 4, raw_press);
	_mav_put_Int32(buf, 8, raw_temp);
	_mav_put_Single(buf, 12, gyro_cal_x);
	_mav_put_Single(buf, 16, gyro_cal_y);
	_mav_put_Single(buf, 20, gyro_cal_z);
	_mav_put_Single(buf, 24, accel_cal_x);
	_mav_put_Single(buf, 28, accel_cal_y);
	_mav_put_Single(buf, 32, accel_cal_z);
	_mav_put_Int16(buf, 36, mag_ofs_x);
	_mav_put_Int16(buf, 38, mag_ofs_y);
	_mav_put_Int16(buf, 40, mag_ofs_z);

        memcpy(_MAV_PAYLOAD(msg), buf, 42);
#else
    mavlink_sensor_offsets_t packet;
	packet.mag_declination = mag_declination;
	packet.raw_press = raw_press;
	packet.raw_temp = raw_temp;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.accel_cal_x = accel_cal_x;
	packet.accel_cal_y = accel_cal_y;
	packet.accel_cal_z = accel_cal_z;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 42);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_OFFSETS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 42, 134);
}
*/
/**
 * @brief Encode a sensor_offsets struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_offsets C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_sensor_offsets_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_offsets_t* sensor_offsets)
{
    return mavlink_msg_sensor_offsets_pack(system_id, component_id, msg, sensor_offsets->mag_ofs_x, sensor_offsets->mag_ofs_y, sensor_offsets->mag_ofs_z, sensor_offsets->mag_declination, sensor_offsets->raw_press, sensor_offsets->raw_temp, sensor_offsets->gyro_cal_x, sensor_offsets->gyro_cal_y, sensor_offsets->gyro_cal_z, sensor_offsets->accel_cal_x, sensor_offsets->accel_cal_y, sensor_offsets->accel_cal_z);
}
*/
/**
 * @brief Send a sensor_offsets message
 * @param chan MAVLink channel to send the message
 *
 * @param mag_ofs_x magnetometer X offset
 * @param mag_ofs_y magnetometer Y offset
 * @param mag_ofs_z magnetometer Z offset
 * @param mag_declination magnetic declination (radians)
 * @param raw_press raw pressure from barometer
 * @param raw_temp raw temperature from barometer
 * @param gyro_cal_x gyro X calibration
 * @param gyro_cal_y gyro Y calibration
 * @param gyro_cal_z gyro Z calibration
 * @param accel_cal_x accel X calibration
 * @param accel_cal_y accel Y calibration
 * @param accel_cal_z accel Z calibration
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_offsets_send(mavlink_channel_t chan, Int16 public mag_ofs_x, Int16 public mag_ofs_y, Int16 public mag_ofs_z, Single public mag_declination, Int32 public raw_press, Int32 public raw_temp, Single public gyro_cal_x, Single public gyro_cal_y, Single public gyro_cal_z, Single public accel_cal_x, Single public accel_cal_y, Single public accel_cal_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[42];
	_mav_put_Single(buf, 0, mag_declination);
	_mav_put_Int32(buf, 4, raw_press);
	_mav_put_Int32(buf, 8, raw_temp);
	_mav_put_Single(buf, 12, gyro_cal_x);
	_mav_put_Single(buf, 16, gyro_cal_y);
	_mav_put_Single(buf, 20, gyro_cal_z);
	_mav_put_Single(buf, 24, accel_cal_x);
	_mav_put_Single(buf, 28, accel_cal_y);
	_mav_put_Single(buf, 32, accel_cal_z);
	_mav_put_Int16(buf, 36, mag_ofs_x);
	_mav_put_Int16(buf, 38, mag_ofs_y);
	_mav_put_Int16(buf, 40, mag_ofs_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_OFFSETS, buf, 42, 134);
#else
    mavlink_sensor_offsets_t packet;
	packet.mag_declination = mag_declination;
	packet.raw_press = raw_press;
	packet.raw_temp = raw_temp;
	packet.gyro_cal_x = gyro_cal_x;
	packet.gyro_cal_y = gyro_cal_y;
	packet.gyro_cal_z = gyro_cal_z;
	packet.accel_cal_x = accel_cal_x;
	packet.accel_cal_y = accel_cal_y;
	packet.accel_cal_z = accel_cal_z;
	packet.mag_ofs_x = mag_ofs_x;
	packet.mag_ofs_y = mag_ofs_y;
	packet.mag_ofs_z = mag_ofs_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_OFFSETS, (const char *)&packet, 42, 134);
#endif
}

#endif
*/
// MESSAGE SENSOR_OFFSETS UNPACKING


/**
 * @brief Get field mag_ofs_x from sensor_offsets message
 *
 * @return magnetometer X offset
 */
public static Int16 mavlink_msg_sensor_offsets_get_mag_ofs_x(byte[] msg)
{
    return BitConverter.ToInt16(msg,  36);
}

/**
 * @brief Get field mag_ofs_y from sensor_offsets message
 *
 * @return magnetometer Y offset
 */
public static Int16 mavlink_msg_sensor_offsets_get_mag_ofs_y(byte[] msg)
{
    return BitConverter.ToInt16(msg,  38);
}

/**
 * @brief Get field mag_ofs_z from sensor_offsets message
 *
 * @return magnetometer Z offset
 */
public static Int16 mavlink_msg_sensor_offsets_get_mag_ofs_z(byte[] msg)
{
    return BitConverter.ToInt16(msg,  40);
}

/**
 * @brief Get field mag_declination from sensor_offsets message
 *
 * @return magnetic declination (radians)
 */
public static Single mavlink_msg_sensor_offsets_get_mag_declination(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field raw_press from sensor_offsets message
 *
 * @return raw pressure from barometer
 */
public static Int32 mavlink_msg_sensor_offsets_get_raw_press(byte[] msg)
{
    return BitConverter.ToInt32(msg,  4);
}

/**
 * @brief Get field raw_temp from sensor_offsets message
 *
 * @return raw temperature from barometer
 */
public static Int32 mavlink_msg_sensor_offsets_get_raw_temp(byte[] msg)
{
    return BitConverter.ToInt32(msg,  8);
}

/**
 * @brief Get field gyro_cal_x from sensor_offsets message
 *
 * @return gyro X calibration
 */
public static Single mavlink_msg_sensor_offsets_get_gyro_cal_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field gyro_cal_y from sensor_offsets message
 *
 * @return gyro Y calibration
 */
public static Single mavlink_msg_sensor_offsets_get_gyro_cal_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field gyro_cal_z from sensor_offsets message
 *
 * @return gyro Z calibration
 */
public static Single mavlink_msg_sensor_offsets_get_gyro_cal_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field accel_cal_x from sensor_offsets message
 *
 * @return accel X calibration
 */
public static Single mavlink_msg_sensor_offsets_get_accel_cal_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field accel_cal_y from sensor_offsets message
 *
 * @return accel Y calibration
 */
public static Single mavlink_msg_sensor_offsets_get_accel_cal_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Get field accel_cal_z from sensor_offsets message
 *
 * @return accel Z calibration
 */
public static Single mavlink_msg_sensor_offsets_get_accel_cal_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  32);
}

/**
 * @brief Decode a sensor_offsets message into a struct
 *
 * @param msg The message to decode
 * @param sensor_offsets C-struct to decode the message contents into
 */
public static void mavlink_msg_sensor_offsets_decode(byte[] msg, ref mavlink_sensor_offsets_t sensor_offsets)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	sensor_offsets.mag_declination = mavlink_msg_sensor_offsets_get_mag_declination(msg);
    	sensor_offsets.raw_press = mavlink_msg_sensor_offsets_get_raw_press(msg);
    	sensor_offsets.raw_temp = mavlink_msg_sensor_offsets_get_raw_temp(msg);
    	sensor_offsets.gyro_cal_x = mavlink_msg_sensor_offsets_get_gyro_cal_x(msg);
    	sensor_offsets.gyro_cal_y = mavlink_msg_sensor_offsets_get_gyro_cal_y(msg);
    	sensor_offsets.gyro_cal_z = mavlink_msg_sensor_offsets_get_gyro_cal_z(msg);
    	sensor_offsets.accel_cal_x = mavlink_msg_sensor_offsets_get_accel_cal_x(msg);
    	sensor_offsets.accel_cal_y = mavlink_msg_sensor_offsets_get_accel_cal_y(msg);
    	sensor_offsets.accel_cal_z = mavlink_msg_sensor_offsets_get_accel_cal_z(msg);
    	sensor_offsets.mag_ofs_x = mavlink_msg_sensor_offsets_get_mag_ofs_x(msg);
    	sensor_offsets.mag_ofs_y = mavlink_msg_sensor_offsets_get_mag_ofs_y(msg);
    	sensor_offsets.mag_ofs_z = mavlink_msg_sensor_offsets_get_mag_ofs_z(msg);
    
    } else {
        int len = 42; //Marshal.SizeOf(sensor_offsets);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        sensor_offsets = (mavlink_sensor_offsets_t)Marshal.PtrToStructure(i, ((object)sensor_offsets).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
