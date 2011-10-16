// MESSAGE DIAGNOSTIC PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_DIAGNOSTIC = 173;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_diagnostic_t
    {
         public  Single diagFl1; /// Diagnostic float 1
     public  Single diagFl2; /// Diagnostic float 2
     public  Single diagFl3; /// Diagnostic float 3
     public  Int16 diagSh1; /// Diagnostic short 1
     public  Int16 diagSh2; /// Diagnostic short 2
     public  Int16 diagSh3; /// Diagnostic short 3
    
    };

/**
 * @brief Pack a diagnostic message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_diagnostic_pack(byte system_id, byte component_id, byte[] msg,
                               Single diagFl1, Single diagFl2, Single diagFl3, Int16 diagSh1, Int16 diagSh2, Int16 diagSh3)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(diagFl1),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(diagFl2),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(diagFl3),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(diagSh1),0,msg,12,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(diagSh2),0,msg,14,sizeof(Int16));
	Array.Copy(BitConverter.GetBytes(diagSh3),0,msg,16,sizeof(Int16));

} else {
    mavlink_diagnostic_t packet = new mavlink_diagnostic_t();
	packet.diagFl1 = diagFl1;
	packet.diagFl2 = diagFl2;
	packet.diagFl3 = diagFl3;
	packet.diagSh1 = diagSh1;
	packet.diagSh2 = diagSh2;
	packet.diagSh3 = diagSh3;

        
        int len = 18;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_DIAGNOSTIC;
    //return mavlink_finalize_message(msg, system_id, component_id, 18, 2);
    return 0;
}

/**
 * @brief Pack a diagnostic message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_diagnostic_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public diagFl1,Single public diagFl2,Single public diagFl3,Int16 public diagSh1,Int16 public diagSh2,Int16 public diagSh3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_Single(buf, 0, diagFl1);
	_mav_put_Single(buf, 4, diagFl2);
	_mav_put_Single(buf, 8, diagFl3);
	_mav_put_Int16(buf, 12, diagSh1);
	_mav_put_Int16(buf, 14, diagSh2);
	_mav_put_Int16(buf, 16, diagSh3);

        memcpy(_MAV_PAYLOAD(msg), buf, 18);
#else
    mavlink_diagnostic_t packet;
	packet.diagFl1 = diagFl1;
	packet.diagFl2 = diagFl2;
	packet.diagFl3 = diagFl3;
	packet.diagSh1 = diagSh1;
	packet.diagSh2 = diagSh2;
	packet.diagSh3 = diagSh3;

        memcpy(_MAV_PAYLOAD(msg), &packet, 18);
#endif

    msg->msgid = MAVLINK_MSG_ID_DIAGNOSTIC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 2);
}
*/
/**
 * @brief Encode a diagnostic struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param diagnostic C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_diagnostic_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_diagnostic_t* diagnostic)
{
    return mavlink_msg_diagnostic_pack(system_id, component_id, msg, diagnostic->diagFl1, diagnostic->diagFl2, diagnostic->diagFl3, diagnostic->diagSh1, diagnostic->diagSh2, diagnostic->diagSh3);
}
*/
/**
 * @brief Send a diagnostic message
 * @param chan MAVLink channel to send the message
 *
 * @param diagFl1 Diagnostic float 1
 * @param diagFl2 Diagnostic float 2
 * @param diagFl3 Diagnostic float 3
 * @param diagSh1 Diagnostic short 1
 * @param diagSh2 Diagnostic short 2
 * @param diagSh3 Diagnostic short 3
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_diagnostic_send(mavlink_channel_t chan, Single public diagFl1, Single public diagFl2, Single public diagFl3, Int16 public diagSh1, Int16 public diagSh2, Int16 public diagSh3)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[18];
	_mav_put_Single(buf, 0, diagFl1);
	_mav_put_Single(buf, 4, diagFl2);
	_mav_put_Single(buf, 8, diagFl3);
	_mav_put_Int16(buf, 12, diagSh1);
	_mav_put_Int16(buf, 14, diagSh2);
	_mav_put_Int16(buf, 16, diagSh3);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIAGNOSTIC, buf, 18, 2);
#else
    mavlink_diagnostic_t packet;
	packet.diagFl1 = diagFl1;
	packet.diagFl2 = diagFl2;
	packet.diagFl3 = diagFl3;
	packet.diagSh1 = diagSh1;
	packet.diagSh2 = diagSh2;
	packet.diagSh3 = diagSh3;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DIAGNOSTIC, (const char *)&packet, 18, 2);
#endif
}

#endif
*/
// MESSAGE DIAGNOSTIC UNPACKING


/**
 * @brief Get field diagFl1 from diagnostic message
 *
 * @return Diagnostic float 1
 */
public static Single mavlink_msg_diagnostic_get_diagFl1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field diagFl2 from diagnostic message
 *
 * @return Diagnostic float 2
 */
public static Single mavlink_msg_diagnostic_get_diagFl2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field diagFl3 from diagnostic message
 *
 * @return Diagnostic float 3
 */
public static Single mavlink_msg_diagnostic_get_diagFl3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field diagSh1 from diagnostic message
 *
 * @return Diagnostic short 1
 */
public static Int16 mavlink_msg_diagnostic_get_diagSh1(byte[] msg)
{
    return BitConverter.ToInt16(msg,  12);
}

/**
 * @brief Get field diagSh2 from diagnostic message
 *
 * @return Diagnostic short 2
 */
public static Int16 mavlink_msg_diagnostic_get_diagSh2(byte[] msg)
{
    return BitConverter.ToInt16(msg,  14);
}

/**
 * @brief Get field diagSh3 from diagnostic message
 *
 * @return Diagnostic short 3
 */
public static Int16 mavlink_msg_diagnostic_get_diagSh3(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Decode a diagnostic message into a struct
 *
 * @param msg The message to decode
 * @param diagnostic C-struct to decode the message contents into
 */
public static void mavlink_msg_diagnostic_decode(byte[] msg, ref mavlink_diagnostic_t diagnostic)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	diagnostic.diagFl1 = mavlink_msg_diagnostic_get_diagFl1(msg);
    	diagnostic.diagFl2 = mavlink_msg_diagnostic_get_diagFl2(msg);
    	diagnostic.diagFl3 = mavlink_msg_diagnostic_get_diagFl3(msg);
    	diagnostic.diagSh1 = mavlink_msg_diagnostic_get_diagSh1(msg);
    	diagnostic.diagSh2 = mavlink_msg_diagnostic_get_diagSh2(msg);
    	diagnostic.diagSh3 = mavlink_msg_diagnostic_get_diagSh3(msg);
    
    } else {
        int len = 18; //Marshal.SizeOf(diagnostic);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        diagnostic = (mavlink_diagnostic_t)Marshal.PtrToStructure(i, ((object)diagnostic).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
