// MESSAGE CHANGE_OPERATOR_CONTROL PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL = 5;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_change_operator_control_t
    {
        /// <summary>
        /// System the GCS requests control for
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// 0: request control of this MAV, 1: Release control of this MAV
        /// </summary>
        public  byte control_request;
            /// <summary>
        /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
        /// </summary>
        public  byte version;
            /// <summary>
        /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray,SizeConst=25)]
 public string passkey;
    
    };

/// <summary>
/// * @brief Pack a change_operator_control message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System the GCS requests control for
/// * @param control_request 0: request control of this MAV, 1: Release control of this MAV
/// * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
/// * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_change_operator_control_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte control_request, byte version, string passkey)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(control_request),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(version),0,msg,2,sizeof(byte));
	Array.Copy(toArray(passkey),0,msg,3,25);
} else {
    mavlink_change_operator_control_t packet = new mavlink_change_operator_control_t();
	packet.target_system = target_system;
	packet.control_request = control_request;
	packet.version = version;
	packet.passkey = passkey;
        
        int len = 28;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL;
    //return mavlink_finalize_message(msg, system_id, component_id, 28, 217);
    return 0;
}

/**
 * @brief Pack a change_operator_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_change_operator_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public control_request,byte public version,const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=25)]
 publicpasskey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, control_request);
	_mav_put_byte(buf, 2, version);
	_mav_put_string_array(buf, 3, passkey, 25);
        memcpy(_MAV_PAYLOAD(msg), buf, 28);
#else
    mavlink_change_operator_control_t packet;
	packet.target_system = target_system;
	packet.control_request = control_request;
	packet.version = version;
	memcpy(packet.passkey, passkey, sizeof(string)*25);
        memcpy(_MAV_PAYLOAD(msg), &packet, 28);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 28, 217);
}
*/
/**
 * @brief Encode a change_operator_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param change_operator_control C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_change_operator_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_change_operator_control_t* change_operator_control)
{
    return mavlink_msg_change_operator_control_pack(system_id, component_id, msg, change_operator_control->target_system, change_operator_control->control_request, change_operator_control->version, change_operator_control->passkey);
}
*/
/**
 * @brief Send a change_operator_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System the GCS requests control for
 * @param control_request 0: request control of this MAV, 1: Release control of this MAV
 * @param version 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 * @param passkey Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_change_operator_control_send(mavlink_channel_t chan, byte public target_system, byte public control_request, byte public version, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=25)]
 publicpasskey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[28];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, control_request);
	_mav_put_byte(buf, 2, version);
	_mav_put_string_array(buf, 3, passkey, 25);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, buf, 28, 217);
#else
    mavlink_change_operator_control_t packet;
	packet.target_system = target_system;
	packet.control_request = control_request;
	packet.version = version;
	memcpy(packet.passkey, passkey, sizeof(string)*25);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL, (const char *)&packet, 28, 217);
#endif
}

#endif
*/
// MESSAGE CHANGE_OPERATOR_CONTROL UNPACKING


/**
 * @brief Get field target_system from change_operator_control message
 *
 * @return System the GCS requests control for
 */
public static byte mavlink_msg_change_operator_control_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field control_request from change_operator_control message
 *
 * @return 0: request control of this MAV, 1: Release control of this MAV
 */
public static byte mavlink_msg_change_operator_control_get_control_request(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field version from change_operator_control message
 *
 * @return 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
 */
public static byte mavlink_msg_change_operator_control_get_version(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field passkey from change_operator_control message
 *
 * @return Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
 */
public static string mavlink_msg_change_operator_control_get_passkey(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,3,25); //(msg, 25,  3);
}

/**
 * @brief Decode a change_operator_control message into a struct
 *
 * @param msg The message to decode
 * @param change_operator_control C-struct to decode the message contents into
 */
public static void mavlink_msg_change_operator_control_decode(byte[] msg, ref mavlink_change_operator_control_t change_operator_control)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	change_operator_control.target_system = mavlink_msg_change_operator_control_get_target_system(msg);
    	change_operator_control.control_request = mavlink_msg_change_operator_control_get_control_request(msg);
    	change_operator_control.version = mavlink_msg_change_operator_control_get_version(msg);
    	change_operator_control.passkey = mavlink_msg_change_operator_control_get_passkey(msg);
    
    } else {
        int len = 28; //Marshal.SizeOf(change_operator_control);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        change_operator_control = (mavlink_change_operator_control_t)Marshal.PtrToStructure(i, ((object)change_operator_control).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
