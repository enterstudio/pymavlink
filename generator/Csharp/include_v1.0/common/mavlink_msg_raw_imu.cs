// MESSAGE RAW_IMU PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_RAW_IMU = 27;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_raw_imu_t
    {
         public  UInt64 time_usec; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     public  Int16 xacc; /// X acceleration (raw)
     public  Int16 yacc; /// Y acceleration (raw)
     public  Int16 zacc; /// Z acceleration (raw)
     public  Int16 xgyro; /// Angular speed around X axis (raw)
     public  Int16 ygyro; /// Angular speed around Y axis (raw)
     public  Int16 zgyro; /// Angular speed around Z axis (raw)
     public  Int16 xmag; /// X Magnetic field (raw)
     public  Int16 ymag; /// Y Magnetic field (raw)
     public  Int16 zmag; /// Z Magnetic field (raw)
    
    };

/**
 * @brief Pack a raw_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param xacc X acceleration (raw)
 * @param yacc Y acceleration (raw)
 * @param zacc Z acceleration (raw)
 * @param xgyro Angular speed around X axis (raw)
 * @param ygyro Angular speed around Y axis (raw)
 * @param zgyro Angular speed around Z axis (raw)
 * @param xmag X Magnetic field (raw)
 * @param ymag Y Magnetic field (raw)
 * @param zmag Z Magnetic field (raw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_raw_imu_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_usec, Int16 xacc, Int16 yacc, Int16 zacc, Int16 xgyro, Int16 ygyro, Int16 zgyro, Int16 xmag, Int16 ymag, Int16 zmag)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_usec),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(xacc),0,msg,8,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(yacc),0,msg,10,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(zacc),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(xgyro),0,msg,14,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(ygyro),0,msg,16,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(zgyro),0,msg,18,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(xmag),0,msg,20,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(ymag),0,msg,22,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(zmag),0,msg,24,sizeof(Int16));

} else {
    mavlink_raw_imu_t packet = new mavlink_raw_imu_t();
	packet.time_usec = time_usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        
        int len = 26;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_RAW_IMU;
    //return mavlink_finalize_message(msg, system_id, component_id, 26, 144);
    return 0;
}

/**
 * @brief Pack a raw_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param xacc X acceleration (raw)
 * @param yacc Y acceleration (raw)
 * @param zacc Z acceleration (raw)
 * @param xgyro Angular speed around X axis (raw)
 * @param ygyro Angular speed around Y axis (raw)
 * @param zgyro Angular speed around Z axis (raw)
 * @param xmag X Magnetic field (raw)
 * @param ymag Y Magnetic field (raw)
 * @param zmag Z Magnetic field (raw)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_raw_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_usec,Int16 public xacc,Int16 public yacc,Int16 public zacc,Int16 public xgyro,Int16 public ygyro,Int16 public zgyro,Int16 public xmag,Int16 public ymag,Int16 public zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Int16(buf, 8, xacc);
	_mav_put_Int16(buf, 10, yacc);
	_mav_put_Int16(buf, 12, zacc);
	_mav_put_Int16(buf, 14, xgyro);
	_mav_put_Int16(buf, 16, ygyro);
	_mav_put_Int16(buf, 18, zgyro);
	_mav_put_Int16(buf, 20, xmag);
	_mav_put_Int16(buf, 22, ymag);
	_mav_put_Int16(buf, 24, zmag);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_raw_imu_t packet;
	packet.time_usec = time_usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_RAW_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26, 144);
}
*/
/**
 * @brief Encode a raw_imu struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param raw_imu C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_raw_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_raw_imu_t* raw_imu)
{
    return mavlink_msg_raw_imu_pack(system_id, component_id, msg, raw_imu->time_usec, raw_imu->xacc, raw_imu->yacc, raw_imu->zacc, raw_imu->xgyro, raw_imu->ygyro, raw_imu->zgyro, raw_imu->xmag, raw_imu->ymag, raw_imu->zmag);
}
*/
/**
 * @brief Send a raw_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param xacc X acceleration (raw)
 * @param yacc Y acceleration (raw)
 * @param zacc Z acceleration (raw)
 * @param xgyro Angular speed around X axis (raw)
 * @param ygyro Angular speed around Y axis (raw)
 * @param zgyro Angular speed around Z axis (raw)
 * @param xmag X Magnetic field (raw)
 * @param ymag Y Magnetic field (raw)
 * @param zmag Z Magnetic field (raw)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_raw_imu_send(mavlink_channel_t chan, UInt64 public time_usec, Int16 public xacc, Int16 public yacc, Int16 public zacc, Int16 public xgyro, Int16 public ygyro, Int16 public zgyro, Int16 public xmag, Int16 public ymag, Int16 public zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_UInt64(buf, 0, time_usec);
	_mav_put_Int16(buf, 8, xacc);
	_mav_put_Int16(buf, 10, yacc);
	_mav_put_Int16(buf, 12, zacc);
	_mav_put_Int16(buf, 14, xgyro);
	_mav_put_Int16(buf, 16, ygyro);
	_mav_put_Int16(buf, 18, zgyro);
	_mav_put_Int16(buf, 20, xmag);
	_mav_put_Int16(buf, 22, ymag);
	_mav_put_Int16(buf, 24, zmag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU, buf, 26, 144);
#else
    mavlink_raw_imu_t packet;
	packet.time_usec = time_usec;
	packet.xacc = xacc;
	packet.yacc = yacc;
	packet.zacc = zacc;
	packet.xgyro = xgyro;
	packet.ygyro = ygyro;
	packet.zgyro = zgyro;
	packet.xmag = xmag;
	packet.ymag = ymag;
	packet.zmag = zmag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RAW_IMU, (const char *)&packet, 26, 144);
#endif
}

#endif
*/
// MESSAGE RAW_IMU UNPACKING


/**
 * @brief Get field time_usec from raw_imu message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_raw_imu_get_time_usec(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field xacc from raw_imu message
 *
 * @return X acceleration (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_xacc(byte[] msg)
{
    return BitConverter.ToInt16(msg,  8);
}

/**
 * @brief Get field yacc from raw_imu message
 *
 * @return Y acceleration (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_yacc(byte[] msg)
{
    return BitConverter.ToInt16(msg,  10);
}

/**
 * @brief Get field zacc from raw_imu message
 *
 * @return Z acceleration (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_zacc(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field xgyro from raw_imu message
 *
 * @return Angular speed around X axis (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_xgyro(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Get field ygyro from raw_imu message
 *
 * @return Angular speed around Y axis (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_ygyro(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Get field zgyro from raw_imu message
 *
 * @return Angular speed around Z axis (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_zgyro(byte[] msg)
{
    return BitConverter.ToInt16(msg,  18);
}

/**
 * @brief Get field xmag from raw_imu message
 *
 * @return X Magnetic field (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_xmag(byte[] msg)
{
    return BitConverter.ToInt16(msg,  20);
}

/**
 * @brief Get field ymag from raw_imu message
 *
 * @return Y Magnetic field (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_ymag(byte[] msg)
{
    return BitConverter.ToInt16(msg,  22);
}

/**
 * @brief Get field zmag from raw_imu message
 *
 * @return Z Magnetic field (raw)
 */
public static Int16 mavlink_msg_raw_imu_get_zmag(byte[] msg)
{
    return BitConverter.ToInt16(msg,  24);
}

/**
 * @brief Decode a raw_imu message into a struct
 *
 * @param msg The message to decode
 * @param raw_imu C-struct to decode the message contents into
 */
public static void mavlink_msg_raw_imu_decode(byte[] msg, ref mavlink_raw_imu_t raw_imu)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	raw_imu.time_usec = mavlink_msg_raw_imu_get_time_usec(msg);
    	raw_imu.xacc = mavlink_msg_raw_imu_get_xacc(msg);
    	raw_imu.yacc = mavlink_msg_raw_imu_get_yacc(msg);
    	raw_imu.zacc = mavlink_msg_raw_imu_get_zacc(msg);
    	raw_imu.xgyro = mavlink_msg_raw_imu_get_xgyro(msg);
    	raw_imu.ygyro = mavlink_msg_raw_imu_get_ygyro(msg);
    	raw_imu.zgyro = mavlink_msg_raw_imu_get_zgyro(msg);
    	raw_imu.xmag = mavlink_msg_raw_imu_get_xmag(msg);
    	raw_imu.ymag = mavlink_msg_raw_imu_get_ymag(msg);
    	raw_imu.zmag = mavlink_msg_raw_imu_get_zmag(msg);
    
    } else {
        int len = 26; //Marshal.SizeOf(raw_imu);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        raw_imu = (mavlink_raw_imu_t)Marshal.PtrToStructure(i, ((object)raw_imu).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
