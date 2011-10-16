// MESSAGE SENSOR_BIAS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SENSOR_BIAS = 172;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_sensor_bias_t
    {
         public  Single axBias; /// Accelerometer X bias (m/s)
     public  Single ayBias; /// Accelerometer Y bias (m/s)
     public  Single azBias; /// Accelerometer Z bias (m/s)
     public  Single gxBias; /// Gyro X bias (rad/s)
     public  Single gyBias; /// Gyro Y bias (rad/s)
     public  Single gzBias; /// Gyro Z bias (rad/s)
    
    };

/**
 * @brief Pack a sensor_bias message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_sensor_bias_pack(byte system_id, byte component_id, byte[] msg,
                               Single axBias, Single ayBias, Single azBias, Single gxBias, Single gyBias, Single gzBias)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(axBias),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(ayBias),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(azBias),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(gxBias),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(gyBias),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(gzBias),0,msg,20,sizeof(Single));

} else {
    mavlink_sensor_bias_t packet = new mavlink_sensor_bias_t();
	packet.axBias = axBias;
	packet.ayBias = ayBias;
	packet.azBias = azBias;
	packet.gxBias = gxBias;
	packet.gyBias = gyBias;
	packet.gzBias = gzBias;

        
        int len = 24;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SENSOR_BIAS;
    //return mavlink_finalize_message(msg, system_id, component_id, 24);
    return 0;
}

/**
 * @brief Pack a sensor_bias message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_sensor_bias_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public axBias,Single public ayBias,Single public azBias,Single public gxBias,Single public gyBias,Single public gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[24];
	_mav_put_Single(buf, 0, axBias);
	_mav_put_Single(buf, 4, ayBias);
	_mav_put_Single(buf, 8, azBias);
	_mav_put_Single(buf, 12, gxBias);
	_mav_put_Single(buf, 16, gyBias);
	_mav_put_Single(buf, 20, gzBias);

        memcpy(_MAV_PAYLOAD(msg), buf, 24);
#else
    mavlink_sensor_bias_t packet;
	packet.axBias = axBias;
	packet.ayBias = ayBias;
	packet.azBias = azBias;
	packet.gxBias = gxBias;
	packet.gyBias = gyBias;
	packet.gzBias = gzBias;

        memcpy(_MAV_PAYLOAD(msg), &packet, 24);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_BIAS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24);
}
*/
/**
 * @brief Encode a sensor_bias struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_bias C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_sensor_bias_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_bias_t* sensor_bias)
{
    return mavlink_msg_sensor_bias_pack(system_id, component_id, msg, sensor_bias->axBias, sensor_bias->ayBias, sensor_bias->azBias, sensor_bias->gxBias, sensor_bias->gyBias, sensor_bias->gzBias);
}
*/
/**
 * @brief Send a sensor_bias message
 * @param chan MAVLink channel to send the message
 *
 * @param axBias Accelerometer X bias (m/s)
 * @param ayBias Accelerometer Y bias (m/s)
 * @param azBias Accelerometer Z bias (m/s)
 * @param gxBias Gyro X bias (rad/s)
 * @param gyBias Gyro Y bias (rad/s)
 * @param gzBias Gyro Z bias (rad/s)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_bias_send(mavlink_channel_t chan, Single public axBias, Single public ayBias, Single public azBias, Single public gxBias, Single public gyBias, Single public gzBias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[24];
	_mav_put_Single(buf, 0, axBias);
	_mav_put_Single(buf, 4, ayBias);
	_mav_put_Single(buf, 8, azBias);
	_mav_put_Single(buf, 12, gxBias);
	_mav_put_Single(buf, 16, gyBias);
	_mav_put_Single(buf, 20, gzBias);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, buf, 24);
#else
    mavlink_sensor_bias_t packet;
	packet.axBias = axBias;
	packet.ayBias = ayBias;
	packet.azBias = azBias;
	packet.gxBias = gxBias;
	packet.gyBias = gyBias;
	packet.gzBias = gzBias;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_BIAS, (const char *)&packet, 24);
#endif
}

#endif
*/
// MESSAGE SENSOR_BIAS UNPACKING


/**
 * @brief Get field axBias from sensor_bias message
 *
 * @return Accelerometer X bias (m/s)
 */
public static Single mavlink_msg_sensor_bias_get_axBias(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field ayBias from sensor_bias message
 *
 * @return Accelerometer Y bias (m/s)
 */
public static Single mavlink_msg_sensor_bias_get_ayBias(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field azBias from sensor_bias message
 *
 * @return Accelerometer Z bias (m/s)
 */
public static Single mavlink_msg_sensor_bias_get_azBias(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field gxBias from sensor_bias message
 *
 * @return Gyro X bias (rad/s)
 */
public static Single mavlink_msg_sensor_bias_get_gxBias(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field gyBias from sensor_bias message
 *
 * @return Gyro Y bias (rad/s)
 */
public static Single mavlink_msg_sensor_bias_get_gyBias(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field gzBias from sensor_bias message
 *
 * @return Gyro Z bias (rad/s)
 */
public static Single mavlink_msg_sensor_bias_get_gzBias(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Decode a sensor_bias message into a struct
 *
 * @param msg The message to decode
 * @param sensor_bias C-struct to decode the message contents into
 */
public static void mavlink_msg_sensor_bias_decode(byte[] msg, ref mavlink_sensor_bias_t sensor_bias)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	sensor_bias.axBias = mavlink_msg_sensor_bias_get_axBias(msg);
    	sensor_bias.ayBias = mavlink_msg_sensor_bias_get_ayBias(msg);
    	sensor_bias.azBias = mavlink_msg_sensor_bias_get_azBias(msg);
    	sensor_bias.gxBias = mavlink_msg_sensor_bias_get_gxBias(msg);
    	sensor_bias.gyBias = mavlink_msg_sensor_bias_get_gyBias(msg);
    	sensor_bias.gzBias = mavlink_msg_sensor_bias_get_gzBias(msg);
    
    } else {
        int len = 24; //Marshal.SizeOf(sensor_bias);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        sensor_bias = (mavlink_sensor_bias_t)Marshal.PtrToStructure(i, ((object)sensor_bias).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
