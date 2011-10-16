// MESSAGE AUTH_KEY PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_AUTH_KEY = 7;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_auth_key_t
    {
         [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 public string key; /// key
    
    };

/**
 * @brief Pack a auth_key message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param key key
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_auth_key_pack(byte system_id, byte component_id, byte[] msg,
                               string key)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {

	//Array.Copy(key,0,msg,0,32);
} else {
    mavlink_auth_key_t packet = new mavlink_auth_key_t();

	packet.key = key;
        
        int len = 32;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_AUTH_KEY;
    //return mavlink_finalize_message(msg, system_id, component_id, 32);
    return 0;
}

/**
 * @brief Pack a auth_key message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param key key
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_auth_key_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 publickey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];

	_mav_put_string_array(buf, 0, key, 32);
        memcpy(_MAV_PAYLOAD(msg), buf, 32);
#else
    mavlink_auth_key_t packet;

	memcpy(packet.key, key, sizeof(string)*32);
        memcpy(_MAV_PAYLOAD(msg), &packet, 32);
#endif

    msg->msgid = MAVLINK_MSG_ID_AUTH_KEY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 32);
}
*/
/**
 * @brief Encode a auth_key struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param auth_key C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_auth_key_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_auth_key_t* auth_key)
{
    return mavlink_msg_auth_key_pack(system_id, component_id, msg, auth_key->key);
}
*/
/**
 * @brief Send a auth_key message
 * @param chan MAVLink channel to send the message
 *
 * @param key key
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_auth_key_send(mavlink_channel_t chan, const string [MarshalAs(UnmanagedType.ByValArray,SizeConst=32)]
 publickey)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[32];

	_mav_put_string_array(buf, 0, key, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_KEY, buf, 32);
#else
    mavlink_auth_key_t packet;

	memcpy(packet.key, key, sizeof(string)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AUTH_KEY, (const char *)&packet, 32);
#endif
}

#endif
*/
// MESSAGE AUTH_KEY UNPACKING


/**
 * @brief Get field key from auth_key message
 *
 * @return key
 */
public static string mavlink_msg_auth_key_get_key(byte[] msg)
{
    return System.Text.ASCIIEncoding.ASCII.GetString(msg,0,32); //(msg, 32,  0);
}

/**
 * @brief Decode a auth_key message into a struct
 *
 * @param msg The message to decode
 * @param auth_key C-struct to decode the message contents into
 */
public static void mavlink_msg_auth_key_decode(byte[] msg, ref mavlink_auth_key_t auth_key)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	auth_key.key = mavlink_msg_auth_key_get_key(msg);
    
    } else {
        int len = 32; //Marshal.SizeOf(auth_key);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        auth_key = (mavlink_auth_key_t)Marshal.PtrToStructure(i, ((object)auth_key).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
