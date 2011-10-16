// MESSAGE ENCAPSULATED_DATA PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ENCAPSULATED_DATA = 194;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_encapsulated_data_t
    {
         public  UInt16 seqnr; /// sequence number (starting with 0 on every transmission)
     [MarshalAs(UnmanagedType.ByValArray,SizeConst=253)]
 public byte[] data; /// image data bytes
    
    };

/**
 * @brief Pack a encapsulated_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_encapsulated_data_pack(byte system_id, byte component_id, byte[] msg,
                               UInt16 seqnr, byte[] data)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(seqnr),0,msg,0,sizeof(UInt16));
	//Array.Copy(data,0,msg,2,253);
} else {
    mavlink_encapsulated_data_t packet = new mavlink_encapsulated_data_t();
	packet.seqnr = seqnr;
	packet.data = data;
        
        int len = 255;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
    //return mavlink_finalize_message(msg, system_id, component_id, 255);
    return 0;
}

/**
 * @brief Pack a encapsulated_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_encapsulated_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public seqnr,const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=253)]
 publicdata)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[255];
	_mav_put_UInt16(buf, 0, seqnr);
	_mav_put_byte[]_array(buf, 2, data, 253);
        memcpy(_MAV_PAYLOAD(msg), buf, 255);
#else
    mavlink_encapsulated_data_t packet;
	packet.seqnr = seqnr;
	memcpy(packet.data, data, sizeof(byte[])*253);
        memcpy(_MAV_PAYLOAD(msg), &packet, 255);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENCAPSULATED_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 255);
}
*/
/**
 * @brief Encode a encapsulated_data struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param encapsulated_data C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_encapsulated_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_encapsulated_data_t* encapsulated_data)
{
    return mavlink_msg_encapsulated_data_pack(system_id, component_id, msg, encapsulated_data->seqnr, encapsulated_data->data);
}
*/
/**
 * @brief Send a encapsulated_data message
 * @param chan MAVLink channel to send the message
 *
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_encapsulated_data_send(mavlink_channel_t chan, UInt16 public seqnr, const byte[] [MarshalAs(UnmanagedType.ByValArray,SizeConst=253)]
 publicdata)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[255];
	_mav_put_UInt16(buf, 0, seqnr);
	_mav_put_byte[]_array(buf, 2, data, 253);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCAPSULATED_DATA, buf, 255);
#else
    mavlink_encapsulated_data_t packet;
	packet.seqnr = seqnr;
	memcpy(packet.data, data, sizeof(byte[])*253);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCAPSULATED_DATA, (const char *)&packet, 255);
#endif
}

#endif
*/
// MESSAGE ENCAPSULATED_DATA UNPACKING


/**
 * @brief Get field seqnr from encapsulated_data message
 *
 * @return sequence number (starting with 0 on every transmission)
 */
public static UInt16 mavlink_msg_encapsulated_data_get_seqnr(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field data from encapsulated_data message
 *
 * @return image data bytes
 */
public static byte[] mavlink_msg_encapsulated_data_get_data(byte[] msg)
{
    return getBytes(msg, 253,  2);
}

/**
 * @brief Decode a encapsulated_data message into a struct
 *
 * @param msg The message to decode
 * @param encapsulated_data C-struct to decode the message contents into
 */
public static void mavlink_msg_encapsulated_data_decode(byte[] msg, ref mavlink_encapsulated_data_t encapsulated_data)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	encapsulated_data.seqnr = mavlink_msg_encapsulated_data_get_seqnr(msg);
    	encapsulated_data.data = mavlink_msg_encapsulated_data_get_data(msg);
    
    } else {
        int len = 255; //Marshal.SizeOf(encapsulated_data);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        encapsulated_data = (mavlink_encapsulated_data_t)Marshal.PtrToStructure(i, ((object)encapsulated_data).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
