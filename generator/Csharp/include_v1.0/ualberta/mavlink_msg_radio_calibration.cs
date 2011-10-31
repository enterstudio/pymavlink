// MESSAGE RADIO_CALIBRATION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RADIO_CALIBRATION = 221;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_radio_calibration_t
    {
        /// <summary>
        /// Aileron setpoints: left, center, right
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public uint16_t aileron;
            /// <summary>
        /// Elevator setpoints: nose down, center, nose up
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public uint16_t elevator;
            /// <summary>
        /// Rudder setpoints: nose left, center, nose right
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 public uint16_t rudder;
            /// <summary>
        /// Tail gyro mode/gain setpoints: heading hold, rate mode
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=2)]
 public uint16_t gyro;
            /// <summary>
        /// Pitch curve setpoints (every 25%)
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=5)]
 public uint16_t pitch;
            /// <summary>
        /// Throttle curve setpoints (every 25%)
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=5)]
 public uint16_t throttle;
    
    };

/// <summary>
/// * @brief Pack a radio_calibration message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param aileron Aileron setpoints: left, center, right
/// * @param elevator Elevator setpoints: nose down, center, nose up
/// * @param rudder Rudder setpoints: nose left, center, nose right
/// * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
/// * @param pitch Pitch curve setpoints (every 25%)
/// * @param throttle Throttle curve setpoints (every 25%)
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_radio_calibration_pack(byte system_id, byte component_id, byte[] msg,
                               uint16_t aileron, uint16_t elevator, uint16_t rudder, uint16_t gyro, uint16_t pitch, uint16_t throttle)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {

	Array.Copy(toArray(aileron),0,msg,0,3);
	Array.Copy(toArray(elevator),0,msg,6,3);
	Array.Copy(toArray(rudder),0,msg,12,3);
	Array.Copy(toArray(gyro),0,msg,18,2);
	Array.Copy(toArray(pitch),0,msg,22,5);
	Array.Copy(toArray(throttle),0,msg,32,5);
} else {
    mavlink_radio_calibration_t packet = new mavlink_radio_calibration_t();

	packet.aileron = aileron;
	packet.elevator = elevator;
	packet.rudder = rudder;
	packet.gyro = gyro;
	packet.pitch = pitch;
	packet.throttle = throttle;
        
        int len = 42;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_RADIO_CALIBRATION;
    //return mavlink_finalize_message(msg, system_id, component_id, 42, 71);
    return 0;
}

/**
 * @brief Pack a radio_calibration message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param aileron Aileron setpoints: left, center, right
 * @param elevator Elevator setpoints: nose down, center, nose up
 * @param rudder Rudder setpoints: nose left, center, nose right
 * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
 * @param pitch Pitch curve setpoints (every 25%)
 * @param throttle Throttle curve setpoints (every 25%)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_radio_calibration_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicaileron,const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicelevator,const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicrudder,const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=2)]
 publicgyro,const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=5)]
 publicpitch,const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=5)]
 publicthrottle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[42];

	_mav_put_uint16_t_array(buf, 0, aileron, 3);
	_mav_put_uint16_t_array(buf, 6, elevator, 3);
	_mav_put_uint16_t_array(buf, 12, rudder, 3);
	_mav_put_uint16_t_array(buf, 18, gyro, 2);
	_mav_put_uint16_t_array(buf, 22, pitch, 5);
	_mav_put_uint16_t_array(buf, 32, throttle, 5);
        memcpy(_MAV_PAYLOAD(msg), buf, 42);
#else
    mavlink_radio_calibration_t packet;

	memcpy(packet.aileron, aileron, sizeof(uint16_t)*3);
	memcpy(packet.elevator, elevator, sizeof(uint16_t)*3);
	memcpy(packet.rudder, rudder, sizeof(uint16_t)*3);
	memcpy(packet.gyro, gyro, sizeof(uint16_t)*2);
	memcpy(packet.pitch, pitch, sizeof(uint16_t)*5);
	memcpy(packet.throttle, throttle, sizeof(uint16_t)*5);
        memcpy(_MAV_PAYLOAD(msg), &packet, 42);
#endif

    msg->msgid = MAVLINK_MSG_ID_RADIO_CALIBRATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 42, 71);
}
*/
/**
 * @brief Encode a radio_calibration struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param radio_calibration C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_radio_calibration_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_radio_calibration_t* radio_calibration)
{
    return mavlink_msg_radio_calibration_pack(system_id, component_id, msg, radio_calibration->aileron, radio_calibration->elevator, radio_calibration->rudder, radio_calibration->gyro, radio_calibration->pitch, radio_calibration->throttle);
}
*/
/**
 * @brief Send a radio_calibration message
 * @param chan MAVLink channel to send the message
 *
 * @param aileron Aileron setpoints: left, center, right
 * @param elevator Elevator setpoints: nose down, center, nose up
 * @param rudder Rudder setpoints: nose left, center, nose right
 * @param gyro Tail gyro mode/gain setpoints: heading hold, rate mode
 * @param pitch Pitch curve setpoints (every 25%)
 * @param throttle Throttle curve setpoints (every 25%)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_radio_calibration_send(mavlink_channel_t chan, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicaileron, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicelevator, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=3)]
 publicrudder, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=2)]
 publicgyro, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=5)]
 publicpitch, const uint16_t [MarshalAs(UnmanagedType.ByValArray,SizeConst=5)]
 publicthrottle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[42];

	_mav_put_uint16_t_array(buf, 0, aileron, 3);
	_mav_put_uint16_t_array(buf, 6, elevator, 3);
	_mav_put_uint16_t_array(buf, 12, rudder, 3);
	_mav_put_uint16_t_array(buf, 18, gyro, 2);
	_mav_put_uint16_t_array(buf, 22, pitch, 5);
	_mav_put_uint16_t_array(buf, 32, throttle, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_CALIBRATION, buf, 42, 71);
#else
    mavlink_radio_calibration_t packet;

	memcpy(packet.aileron, aileron, sizeof(uint16_t)*3);
	memcpy(packet.elevator, elevator, sizeof(uint16_t)*3);
	memcpy(packet.rudder, rudder, sizeof(uint16_t)*3);
	memcpy(packet.gyro, gyro, sizeof(uint16_t)*2);
	memcpy(packet.pitch, pitch, sizeof(uint16_t)*5);
	memcpy(packet.throttle, throttle, sizeof(uint16_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RADIO_CALIBRATION, (const char *)&packet, 42, 71);
#endif
}

#endif
*/
// MESSAGE RADIO_CALIBRATION UNPACKING


/**
 * @brief Get field aileron from radio_calibration message
 *
 * @return Aileron setpoints: left, center, right
 */
public static void mavlink_msg_radio_calibration_get_aileron(byte[] msg)
{
    return !!!uint16_t(msg, 3,  0);
}

/**
 * @brief Get field elevator from radio_calibration message
 *
 * @return Elevator setpoints: nose down, center, nose up
 */
public static void mavlink_msg_radio_calibration_get_elevator(byte[] msg)
{
    return !!!uint16_t(msg, 3,  6);
}

/**
 * @brief Get field rudder from radio_calibration message
 *
 * @return Rudder setpoints: nose left, center, nose right
 */
public static void mavlink_msg_radio_calibration_get_rudder(byte[] msg)
{
    return !!!uint16_t(msg, 3,  12);
}

/**
 * @brief Get field gyro from radio_calibration message
 *
 * @return Tail gyro mode/gain setpoints: heading hold, rate mode
 */
public static void mavlink_msg_radio_calibration_get_gyro(byte[] msg)
{
    return !!!uint16_t(msg, 2,  18);
}

/**
 * @brief Get field pitch from radio_calibration message
 *
 * @return Pitch curve setpoints (every 25%)
 */
public static void mavlink_msg_radio_calibration_get_pitch(byte[] msg)
{
    return !!!uint16_t(msg, 5,  22);
}

/**
 * @brief Get field throttle from radio_calibration message
 *
 * @return Throttle curve setpoints (every 25%)
 */
public static void mavlink_msg_radio_calibration_get_throttle(byte[] msg)
{
    return !!!uint16_t(msg, 5,  32);
}

/**
 * @brief Decode a radio_calibration message into a struct
 *
 * @param msg The message to decode
 * @param radio_calibration C-struct to decode the message contents into
 */
public static void mavlink_msg_radio_calibration_decode(byte[] msg, ref mavlink_radio_calibration_t radio_calibration)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	radio_calibration.aileron = mavlink_msg_radio_calibration_get_aileron(msg);
    	radio_calibration.elevator = mavlink_msg_radio_calibration_get_elevator(msg);
    	radio_calibration.rudder = mavlink_msg_radio_calibration_get_rudder(msg);
    	radio_calibration.gyro = mavlink_msg_radio_calibration_get_gyro(msg);
    	radio_calibration.pitch = mavlink_msg_radio_calibration_get_pitch(msg);
    	radio_calibration.throttle = mavlink_msg_radio_calibration_get_throttle(msg);
    
    } else {
        int len = 42; //Marshal.SizeOf(radio_calibration);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        radio_calibration = (mavlink_radio_calibration_t)Marshal.PtrToStructure(i, ((object)radio_calibration).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
