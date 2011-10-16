// MESSAGE STATUSTEXT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_STATUSTEXT = 254;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_statustext_t
    {
         public  byte severity; /// Severity of status, 0 = info message, 255 = critical fault
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=50)]
 public byte[] text; /// Status text message, without null termination character
    
    };

/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity Severity of status, 0 = info message, 255 = critical fault
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_statustext_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public severity, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=50)]
 publictext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[51];
	_mav_put_byte(buf, 0, severity);
	_mav_put_byte[]_array(buf, 1, text, 50);
        memcpy(_MAV_PAYLOAD(msg), buf, 51);
#else
    mavlink_statustext_t packet;
	packet.severity = severity;
	memcpy(packet.text, text, sizeof(byte[])*50);
        memcpy(_MAV_PAYLOAD(msg), &packet, 51);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    return mavlink_finalize_message(msg, system_id, component_id, 51);
}
*/
/**
 * @brief Pack a statustext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity Severity of status, 0 = info message, 255 = critical fault
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public severity,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=50)]
 publictext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[51];
	_mav_put_byte(buf, 0, severity);
	_mav_put_byte[]_array(buf, 1, text, 50);
        memcpy(_MAV_PAYLOAD(msg), buf, 51);
#else
    mavlink_statustext_t packet;
	packet.severity = severity;
	memcpy(packet.text, text, sizeof(byte[])*50);
        memcpy(_MAV_PAYLOAD(msg), &packet, 51);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 51);
}
*/
/**
 * @brief Encode a statustext struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
    return mavlink_msg_statustext_pack(system_id, component_id, msg, statustext->severity, statustext->text);
}
*/
/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param severity Severity of status, 0 = info message, 255 = critical fault
 * @param text Status text message, without null termination character
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_statustext_send(mavlink_channel_t chan, byte public severity, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=50)]
 publictext)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[51];
	_mav_put_byte(buf, 0, severity);
	_mav_put_byte[]_array(buf, 1, text, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, buf, 51);
#else
    mavlink_statustext_t packet;
	packet.severity = severity;
	memcpy(packet.text, text, sizeof(byte[])*50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, (const char *)&packet, 51);
#endif
}

#endif
*/
// MESSAGE STATUSTEXT UNPACKING


/**
 * @brief Get field severity from statustext message
 *
 * @return Severity of status, 0 = info message, 255 = critical fault
 */
public static byte mavlink_msg_statustext_get_severity(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field text from statustext message
 *
 * @return Status text message, without null termination character
 */
public static byte[] mavlink_msg_statustext_get_text(byte[] msg)
{
    return getBytes(msg, 50,  1);
}

/**
 * @brief Decode a statustext message into a struct
 *
 * @param msg The message to decode
 * @param statustext C-struct to decode the message contents into
 */
public static void mavlink_msg_statustext_decode(byte[] msg, ref mavlink_statustext_t statustext)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	statustext.severity = mavlink_msg_statustext_get_severity(msg);
	statustext.text = mavlink_msg_statustext_get_text(msg);
} else {
    int len = 51; //Marshal.SizeOf(statustext);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    statustext = (mavlink_statustext_t)Marshal.PtrToStructure(i, ((object)statustext).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
