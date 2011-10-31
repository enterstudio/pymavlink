// MESSAGE PATTERN_DETECTED PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_PATTERN_DETECTED = 190;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_pattern_detected_t
    {
        /// <summary>
        /// 0: Pattern, 1: Letter
        /// </summary>
        public  byte type;
            /// <summary>
        /// Confidence of detection
        /// </summary>
        public  Single confidence;
            /// <summary>
        /// Pattern file name
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=100)]
 public string file;
            /// <summary>
        /// Accepted as true detection, 0 no, 1 yes
        /// </summary>
        public  byte detected;
    
    };

/// <summary>
/// * @brief Pack a pattern_detected message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param type 0: Pattern, 1: Letter
/// * @param confidence Confidence of detection
/// * @param file Pattern file name
/// * @param detected Accepted as true detection, 0 no, 1 yes
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_pattern_detected_pack(byte system_id, byte component_id, byte[] msg,
                               byte type, Single confidence, string file, byte detected)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(type),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(confidence),0,msg,1,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(detected),0,msg,105,sizeof(byte));
	Array.Copy(toArray(file),0,msg,5,100);
} else {
    mavlink_pattern_detected_t packet = new mavlink_pattern_detected_t();
	packet.type = type;
	packet.confidence = confidence;
	packet.detected = detected;
	packet.file = file;
        
        int len = 106;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_PATTERN_DETECTED;
    //return mavlink_finalize_message(msg, system_id, component_id, 106);
    return 0;
}

/**
 * @brief Pack a pattern_detected message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Pattern, 1: Letter
 * @param confidence Confidence of detection
 * @param file Pattern file name
 * @param detected Accepted as true detection, 0 no, 1 yes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_pattern_detected_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,Single public confidence,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=100)]
 publicfile,byte public detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[106];
	_mav_put_byte(buf, 0, type);
	_mav_put_Single(buf, 1, confidence);
	_mav_put_byte(buf, 105, detected);
	_mav_put_string_array(buf, 5, file, 100);
        memcpy(_MAV_PAYLOAD(msg), buf, 106);
#else
    mavlink_pattern_detected_t packet;
	packet.type = type;
	packet.confidence = confidence;
	packet.detected = detected;
	memcpy(packet.file, file, sizeof(string)*100);
        memcpy(_MAV_PAYLOAD(msg), &packet, 106);
#endif

    msg->msgid = MAVLINK_MSG_ID_PATTERN_DETECTED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 106);
}
*/
/**
 * @brief Encode a pattern_detected struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pattern_detected C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_pattern_detected_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pattern_detected_t* pattern_detected)
{
    return mavlink_msg_pattern_detected_pack(system_id, component_id, msg, pattern_detected->type, pattern_detected->confidence, pattern_detected->file, pattern_detected->detected);
}
*/
/**
 * @brief Send a pattern_detected message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Pattern, 1: Letter
 * @param confidence Confidence of detection
 * @param file Pattern file name
 * @param detected Accepted as true detection, 0 no, 1 yes
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pattern_detected_send(mavlink_channel_t chan, byte public type, Single public confidence, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=100)]
 publicfile, byte public detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[106];
	_mav_put_byte(buf, 0, type);
	_mav_put_Single(buf, 1, confidence);
	_mav_put_byte(buf, 105, detected);
	_mav_put_string_array(buf, 5, file, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, buf, 106);
#else
    mavlink_pattern_detected_t packet;
	packet.type = type;
	packet.confidence = confidence;
	packet.detected = detected;
	memcpy(packet.file, file, sizeof(string)*100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, (const char *)&packet, 106);
#endif
}

#endif
*/
// MESSAGE PATTERN_DETECTED UNPACKING


/**
 * @brief Get field type from pattern_detected message
 *
 * @return 0: Pattern, 1: Letter
 */
public static byte mavlink_msg_pattern_detected_get_type(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field confidence from pattern_detected message
 *
 * @return Confidence of detection
 */
public static Single mavlink_msg_pattern_detected_get_confidence(byte[] msg)
{
    return BitConverter.ToSingle(msg,  1);
}

/**
 * @brief Get field file from pattern_detected message
 *
 * @return Pattern file name
 */
public static string mavlink_msg_pattern_detected_get_file(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,5,100); //(msg, 100,  5);
}

/**
 * @brief Get field detected from pattern_detected message
 *
 * @return Accepted as true detection, 0 no, 1 yes
 */
public static byte mavlink_msg_pattern_detected_get_detected(byte[] msg)
{
    return getByte(msg,  105);
}

/**
 * @brief Decode a pattern_detected message into a struct
 *
 * @param msg The message to decode
 * @param pattern_detected C-struct to decode the message contents into
 */
public static void mavlink_msg_pattern_detected_decode(byte[] msg, ref mavlink_pattern_detected_t pattern_detected)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	pattern_detected.type = mavlink_msg_pattern_detected_get_type(msg);
    	pattern_detected.confidence = mavlink_msg_pattern_detected_get_confidence(msg);
    	pattern_detected.file = mavlink_msg_pattern_detected_get_file(msg);
    	pattern_detected.detected = mavlink_msg_pattern_detected_get_detected(msg);
    
    } else {
        int len = 106; //Marshal.SizeOf(pattern_detected);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        pattern_detected = (mavlink_pattern_detected_t)Marshal.PtrToStructure(i, ((object)pattern_detected).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
