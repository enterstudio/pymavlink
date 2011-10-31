// MESSAGE SERVO_OUTPUT_RAW PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 37;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_servo_output_raw_t
    {
        /// <summary>
        /// Timestamp (since UNIX epoch or microseconds since system boot)
        /// </summary>
        public  UInt32 time_usec;
            /// <summary>
        /// Servo output 1 value, in microseconds
        /// </summary>
        public  UInt16 servo1_raw;
            /// <summary>
        /// Servo output 2 value, in microseconds
        /// </summary>
        public  UInt16 servo2_raw;
            /// <summary>
        /// Servo output 3 value, in microseconds
        /// </summary>
        public  UInt16 servo3_raw;
            /// <summary>
        /// Servo output 4 value, in microseconds
        /// </summary>
        public  UInt16 servo4_raw;
            /// <summary>
        /// Servo output 5 value, in microseconds
        /// </summary>
        public  UInt16 servo5_raw;
            /// <summary>
        /// Servo output 6 value, in microseconds
        /// </summary>
        public  UInt16 servo6_raw;
            /// <summary>
        /// Servo output 7 value, in microseconds
        /// </summary>
        public  UInt16 servo7_raw;
            /// <summary>
        /// Servo output 8 value, in microseconds
        /// </summary>
        public  UInt16 servo8_raw;
            /// <summary>
        /// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
        /// </summary>
        public  byte port;
    
    };

/// <summary>
/// * @brief Pack a servo_output_raw message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_usec Timestamp (since UNIX epoch or microseconds since system boot)
/// * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
/// * @param servo1_raw Servo output 1 value, in microseconds
/// * @param servo2_raw Servo output 2 value, in microseconds
/// * @param servo3_raw Servo output 3 value, in microseconds
/// * @param servo4_raw Servo output 4 value, in microseconds
/// * @param servo5_raw Servo output 5 value, in microseconds
/// * @param servo6_raw Servo output 6 value, in microseconds
/// * @param servo7_raw Servo output 7 value, in microseconds
/// * @param servo8_raw Servo output 8 value, in microseconds
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_servo_output_raw_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_usec, byte port, UInt16 servo1_raw, UInt16 servo2_raw, UInt16 servo3_raw, UInt16 servo4_raw, UInt16 servo5_raw, UInt16 servo6_raw, UInt16 servo7_raw, UInt16 servo8_raw)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(servo1_raw),0,msg,4,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo2_raw),0,msg,6,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo3_raw),0,msg,8,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo4_raw),0,msg,10,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo5_raw),0,msg,12,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo6_raw),0,msg,14,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo7_raw),0,msg,16,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(servo8_raw),0,msg,18,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(port),0,msg,20,sizeof(byte));

} else {
    mavlink_servo_output_raw_t packet = new mavlink_servo_output_raw_t();
	packet.time_usec = time_usec;
	packet.servo1_raw = servo1_raw;
	packet.servo2_raw = servo2_raw;
	packet.servo3_raw = servo3_raw;
	packet.servo4_raw = servo4_raw;
	packet.servo5_raw = servo5_raw;
	packet.servo6_raw = servo6_raw;
	packet.servo7_raw = servo7_raw;
	packet.servo8_raw = servo8_raw;
	packet.port = port;

        
        int len = 21;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    //return mavlink_finalize_message(msg, system_id, component_id, 21, 222);
    return 0;
}

/**
 * @brief Pack a servo_output_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (since UNIX epoch or microseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_servo_output_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_usec,byte public port,UInt16 public servo1_raw,UInt16 public servo2_raw,UInt16 public servo3_raw,UInt16 public servo4_raw,UInt16 public servo5_raw,UInt16 public servo6_raw,UInt16 public servo7_raw,UInt16 public servo8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[21];
	_mav_put_UInt32(buf, 0, time_usec);
	_mav_put_UInt16(buf, 4, servo1_raw);
	_mav_put_UInt16(buf, 6, servo2_raw);
	_mav_put_UInt16(buf, 8, servo3_raw);
	_mav_put_UInt16(buf, 10, servo4_raw);
	_mav_put_UInt16(buf, 12, servo5_raw);
	_mav_put_UInt16(buf, 14, servo6_raw);
	_mav_put_UInt16(buf, 16, servo7_raw);
	_mav_put_UInt16(buf, 18, servo8_raw);
	_mav_put_byte(buf, 20, port);

        memcpy(_MAV_PAYLOAD(msg), buf, 21);
#else
    mavlink_servo_output_raw_t packet;
	packet.time_usec = time_usec;
	packet.servo1_raw = servo1_raw;
	packet.servo2_raw = servo2_raw;
	packet.servo3_raw = servo3_raw;
	packet.servo4_raw = servo4_raw;
	packet.servo5_raw = servo5_raw;
	packet.servo6_raw = servo6_raw;
	packet.servo7_raw = servo7_raw;
	packet.servo8_raw = servo8_raw;
	packet.port = port;

        memcpy(_MAV_PAYLOAD(msg), &packet, 21);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 21, 222);
}
*/
/**
 * @brief Encode a servo_output_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param servo_output_raw C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_servo_output_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_servo_output_raw_t* servo_output_raw)
{
    return mavlink_msg_servo_output_raw_pack(system_id, component_id, msg, servo_output_raw->time_usec, servo_output_raw->port, servo_output_raw->servo1_raw, servo_output_raw->servo2_raw, servo_output_raw->servo3_raw, servo_output_raw->servo4_raw, servo_output_raw->servo5_raw, servo_output_raw->servo6_raw, servo_output_raw->servo7_raw, servo_output_raw->servo8_raw);
}
*/
/**
 * @brief Send a servo_output_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (since UNIX epoch or microseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 * @param servo1_raw Servo output 1 value, in microseconds
 * @param servo2_raw Servo output 2 value, in microseconds
 * @param servo3_raw Servo output 3 value, in microseconds
 * @param servo4_raw Servo output 4 value, in microseconds
 * @param servo5_raw Servo output 5 value, in microseconds
 * @param servo6_raw Servo output 6 value, in microseconds
 * @param servo7_raw Servo output 7 value, in microseconds
 * @param servo8_raw Servo output 8 value, in microseconds
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_servo_output_raw_send(mavlink_channel_t chan, UInt32 public time_usec, byte public port, UInt16 public servo1_raw, UInt16 public servo2_raw, UInt16 public servo3_raw, UInt16 public servo4_raw, UInt16 public servo5_raw, UInt16 public servo6_raw, UInt16 public servo7_raw, UInt16 public servo8_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[21];
	_mav_put_UInt32(buf, 0, time_usec);
	_mav_put_UInt16(buf, 4, servo1_raw);
	_mav_put_UInt16(buf, 6, servo2_raw);
	_mav_put_UInt16(buf, 8, servo3_raw);
	_mav_put_UInt16(buf, 10, servo4_raw);
	_mav_put_UInt16(buf, 12, servo5_raw);
	_mav_put_UInt16(buf, 14, servo6_raw);
	_mav_put_UInt16(buf, 16, servo7_raw);
	_mav_put_UInt16(buf, 18, servo8_raw);
	_mav_put_byte(buf, 20, port);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, buf, 21, 222);
#else
    mavlink_servo_output_raw_t packet;
	packet.time_usec = time_usec;
	packet.servo1_raw = servo1_raw;
	packet.servo2_raw = servo2_raw;
	packet.servo3_raw = servo3_raw;
	packet.servo4_raw = servo4_raw;
	packet.servo5_raw = servo5_raw;
	packet.servo6_raw = servo6_raw;
	packet.servo7_raw = servo7_raw;
	packet.servo8_raw = servo8_raw;
	packet.port = port;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, (const char *)&packet, 21, 222);
#endif
}

#endif
*/
// MESSAGE SERVO_OUTPUT_RAW UNPACKING


/**
 * @brief Get field time_usec from servo_output_raw message
 *
 * @return Timestamp (since UNIX epoch or microseconds since system boot)
 */
public static UInt32 mavlink_msg_servo_output_raw_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field port from servo_output_raw message
 *
 * @return Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
 */
public static byte mavlink_msg_servo_output_raw_get_port(byte[] msg)
{
    return getByte(msg,  20);
}

/**
 * @brief Get field servo1_raw from servo_output_raw message
 *
 * @return Servo output 1 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo1_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  4);
}

/**
 * @brief Get field servo2_raw from servo_output_raw message
 *
 * @return Servo output 2 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo2_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  6);
}

/**
 * @brief Get field servo3_raw from servo_output_raw message
 *
 * @return Servo output 3 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo3_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  8);
}

/**
 * @brief Get field servo4_raw from servo_output_raw message
 *
 * @return Servo output 4 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo4_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  10);
}

/**
 * @brief Get field servo5_raw from servo_output_raw message
 *
 * @return Servo output 5 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo5_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field servo6_raw from servo_output_raw message
 *
 * @return Servo output 6 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo6_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  14);
}

/**
 * @brief Get field servo7_raw from servo_output_raw message
 *
 * @return Servo output 7 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo7_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  16);
}

/**
 * @brief Get field servo8_raw from servo_output_raw message
 *
 * @return Servo output 8 value, in microseconds
 */
public static UInt16 mavlink_msg_servo_output_raw_get_servo8_raw(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  18);
}

/**
 * @brief Decode a servo_output_raw message into a struct
 *
 * @param msg The message to decode
 * @param servo_output_raw C-struct to decode the message contents into
 */
public static void mavlink_msg_servo_output_raw_decode(byte[] msg, ref mavlink_servo_output_raw_t servo_output_raw)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	servo_output_raw.time_usec = mavlink_msg_servo_output_raw_get_time_usec(msg);
    	servo_output_raw.servo1_raw = mavlink_msg_servo_output_raw_get_servo1_raw(msg);
    	servo_output_raw.servo2_raw = mavlink_msg_servo_output_raw_get_servo2_raw(msg);
    	servo_output_raw.servo3_raw = mavlink_msg_servo_output_raw_get_servo3_raw(msg);
    	servo_output_raw.servo4_raw = mavlink_msg_servo_output_raw_get_servo4_raw(msg);
    	servo_output_raw.servo5_raw = mavlink_msg_servo_output_raw_get_servo5_raw(msg);
    	servo_output_raw.servo6_raw = mavlink_msg_servo_output_raw_get_servo6_raw(msg);
    	servo_output_raw.servo7_raw = mavlink_msg_servo_output_raw_get_servo7_raw(msg);
    	servo_output_raw.servo8_raw = mavlink_msg_servo_output_raw_get_servo8_raw(msg);
    	servo_output_raw.port = mavlink_msg_servo_output_raw_get_port(msg);
    
    } else {
        int len = 21; //Marshal.SizeOf(servo_output_raw);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        servo_output_raw = (mavlink_servo_output_raw_t)Marshal.PtrToStructure(i, ((object)servo_output_raw).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
