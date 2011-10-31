// MESSAGE SCALED_IMU PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SCALED_IMU = 26;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_scaled_imu_t
    {
        /// <summary>
        /// Timestamp (milliseconds since system boot)
        /// </summary>
        public  UInt32 time_boot_ms;
            /// <summary>
        /// X acceleration (mg)
        /// </summary>
        public  Int16 xacc;
            /// <summary>
        /// Y acceleration (mg)
        /// </summary>
        public  Int16 yacc;
            /// <summary>
        /// Z acceleration (mg)
        /// </summary>
        public  Int16 zacc;
            /// <summary>
        /// Angular speed around X axis (millirad /sec)
        /// </summary>
        public  Int16 xgyro;
            /// <summary>
        /// Angular speed around Y axis (millirad /sec)
        /// </summary>
        public  Int16 ygyro;
            /// <summary>
        /// Angular speed around Z axis (millirad /sec)
        /// </summary>
        public  Int16 zgyro;
            /// <summary>
        /// X Magnetic field (milli tesla)
        /// </summary>
        public  Int16 xmag;
            /// <summary>
        /// Y Magnetic field (milli tesla)
        /// </summary>
        public  Int16 ymag;
            /// <summary>
        /// Z Magnetic field (milli tesla)
        /// </summary>
        public  Int16 zmag;
    
    };

/// <summary>
/// * @brief Pack a scaled_imu message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param time_boot_ms Timestamp (milliseconds since system boot)
/// * @param xacc X acceleration (mg)
/// * @param yacc Y acceleration (mg)
/// * @param zacc Z acceleration (mg)
/// * @param xgyro Angular speed around X axis (millirad /sec)
/// * @param ygyro Angular speed around Y axis (millirad /sec)
/// * @param zgyro Angular speed around Z axis (millirad /sec)
/// * @param xmag X Magnetic field (milli tesla)
/// * @param ymag Y Magnetic field (milli tesla)
/// * @param zmag Z Magnetic field (milli tesla)
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_scaled_imu_pack(byte system_id, byte component_id, byte[] msg,
                               UInt32 time_boot_ms, Int16 xacc, Int16 yacc, Int16 zacc, Int16 xgyro, Int16 ygyro, Int16 zgyro, Int16 xmag, Int16 ymag, Int16 zmag)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_boot_ms),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(xacc),0,msg,4,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(yacc),0,msg,6,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(zacc),0,msg,8,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(xgyro),0,msg,10,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(ygyro),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(zgyro),0,msg,14,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(xmag),0,msg,16,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(ymag),0,msg,18,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(zmag),0,msg,20,sizeof(Int16));

} else {
    mavlink_scaled_imu_t packet = new mavlink_scaled_imu_t();
	packet.time_boot_ms = time_boot_ms;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        
        int len = 22;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SCALED_IMU;
    //return mavlink_finalize_message(msg, system_id, component_id, 22, 170);
    return 0;
}

/**
 * @brief Pack a scaled_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_scaled_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt32 public time_boot_ms,Int16 public xacc,Int16 public yacc,Int16 public zacc,Int16 public xgyro,Int16 public ygyro,Int16 public zgyro,Int16 public xmag,Int16 public ymag,Int16 public zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[22];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int16(buf, 4, xacc);
	_mav_put_Int16(buf, 6, yacc);
	_mav_put_Int16(buf, 8, zacc);
	_mav_put_Int16(buf, 10, xgyro);
	_mav_put_Int16(buf, 12, ygyro);
	_mav_put_Int16(buf, 14, zgyro);
	_mav_put_Int16(buf, 16, xmag);
	_mav_put_Int16(buf, 18, ymag);
	_mav_put_Int16(buf, 20, zmag);

        memcpy(_MAV_PAYLOAD(msg), buf, 22);
#else
    mavlink_scaled_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD(msg), &packet, 22);
#endif

    msg->msgid = MAVLINK_MSG_ID_SCALED_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 22, 170);
}
*/
/**
 * @brief Encode a scaled_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param scaled_imu C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_scaled_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_scaled_imu_t* scaled_imu)
{
    return mavlink_msg_scaled_imu_pack(system_id, component_id, msg, scaled_imu->time_boot_ms, scaled_imu->xacc, scaled_imu->yacc, scaled_imu->zacc, scaled_imu->xgyro, scaled_imu->ygyro, scaled_imu->zgyro, scaled_imu->xmag, scaled_imu->ymag, scaled_imu->zmag);
}
*/
/**
 * @brief Send a scaled_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param xacc X acceleration (mg)
 * @param yacc Y acceleration (mg)
 * @param zacc Z acceleration (mg)
 * @param xgyro Angular speed around X axis (millirad /sec)
 * @param ygyro Angular speed around Y axis (millirad /sec)
 * @param zgyro Angular speed around Z axis (millirad /sec)
 * @param xmag X Magnetic field (milli tesla)
 * @param ymag Y Magnetic field (milli tesla)
 * @param zmag Z Magnetic field (milli tesla)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_scaled_imu_send(mavlink_channel_t chan, UInt32 public time_boot_ms, Int16 public xacc, Int16 public yacc, Int16 public zacc, Int16 public xgyro, Int16 public ygyro, Int16 public zgyro, Int16 public xmag, Int16 public ymag, Int16 public zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[22];
	_mav_put_UInt32(buf, 0, time_boot_ms);
	_mav_put_Int16(buf, 4, xacc);
	_mav_put_Int16(buf, 6, yacc);
	_mav_put_Int16(buf, 8, zacc);
	_mav_put_Int16(buf, 10, xgyro);
	_mav_put_Int16(buf, 12, ygyro);
	_mav_put_Int16(buf, 14, zgyro);
	_mav_put_Int16(buf, 16, xmag);
	_mav_put_Int16(buf, 18, ymag);
	_mav_put_Int16(buf, 20, zmag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU, buf, 22, 170);
#else
    mavlink_scaled_imu_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SCALED_IMU, (const char *)&packet, 22, 170);
#endif
}

#endif
*/
// MESSAGE SCALED_IMU UNPACKING


/**
 * @brief Get field time_boot_ms from scaled_imu message
 *
 * @return Timestamp (milliseconds since system boot)
 */
public static UInt32 mavlink_msg_scaled_imu_get_time_boot_ms(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field xacc from scaled_imu message
 *
 * @return X acceleration (mg)
 */
public static Int16 mavlink_msg_scaled_imu_get_xacc(byte[] msg)
{
    return BitConverter.ToInt16(msg,  4);
}

/**
 * @brief Get field yacc from scaled_imu message
 *
 * @return Y acceleration (mg)
 */
public static Int16 mavlink_msg_scaled_imu_get_yacc(byte[] msg)
{
    return BitConverter.ToInt16(msg,  6);
}

/**
 * @brief Get field zacc from scaled_imu message
 *
 * @return Z acceleration (mg)
 */
public static Int16 mavlink_msg_scaled_imu_get_zacc(byte[] msg)
{
    return BitConverter.ToInt16(msg,  8);
}

/**
 * @brief Get field xgyro from scaled_imu message
 *
 * @return Angular speed around X axis (millirad /sec)
 */
public static Int16 mavlink_msg_scaled_imu_get_xgyro(byte[] msg)
{
    return BitConverter.ToInt16(msg,  10);
}

/**
 * @brief Get field ygyro from scaled_imu message
 *
 * @return Angular speed around Y axis (millirad /sec)
 */
public static Int16 mavlink_msg_scaled_imu_get_ygyro(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field zgyro from scaled_imu message
 *
 * @return Angular speed around Z axis (millirad /sec)
 */
public static Int16 mavlink_msg_scaled_imu_get_zgyro(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Get field xmag from scaled_imu message
 *
 * @return X Magnetic field (milli tesla)
 */
public static Int16 mavlink_msg_scaled_imu_get_xmag(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Get field ymag from scaled_imu message
 *
 * @return Y Magnetic field (milli tesla)
 */
public static Int16 mavlink_msg_scaled_imu_get_ymag(byte[] msg)
{
    return BitConverter.ToInt16(msg,  18);
}

/**
 * @brief Get field zmag from scaled_imu message
 *
 * @return Z Magnetic field (milli tesla)
 */
public static Int16 mavlink_msg_scaled_imu_get_zmag(byte[] msg)
{
    return BitConverter.ToInt16(msg,  20);
}

/**
 * @brief Decode a scaled_imu message into a struct
 *
 * @param msg The message to decode
 * @param scaled_imu C-struct to decode the message contents into
 */
public static void mavlink_msg_scaled_imu_decode(byte[] msg, ref mavlink_scaled_imu_t scaled_imu)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	scaled_imu.time_boot_ms = mavlink_msg_scaled_imu_get_time_boot_ms(msg);
    	scaled_imu.xacc = mavlink_msg_scaled_imu_get_xacc(msg);
    	scaled_imu.yacc = mavlink_msg_scaled_imu_get_yacc(msg);
    	scaled_imu.zacc = mavlink_msg_scaled_imu_get_zacc(msg);
    	scaled_imu.xgyro = mavlink_msg_scaled_imu_get_xgyro(msg);
    	scaled_imu.ygyro = mavlink_msg_scaled_imu_get_ygyro(msg);
    	scaled_imu.zgyro = mavlink_msg_scaled_imu_get_zgyro(msg);
    	scaled_imu.xmag = mavlink_msg_scaled_imu_get_xmag(msg);
    	scaled_imu.ymag = mavlink_msg_scaled_imu_get_ymag(msg);
    	scaled_imu.zmag = mavlink_msg_scaled_imu_get_zmag(msg);
    
    } else {
        int len = 22; //Marshal.SizeOf(scaled_imu);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        scaled_imu = (mavlink_scaled_imu_t)Marshal.PtrToStructure(i, ((object)scaled_imu).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
