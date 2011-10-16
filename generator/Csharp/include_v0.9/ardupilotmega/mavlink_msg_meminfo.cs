// MESSAGE MEMINFO PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MEMINFO = 152;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_meminfo_t
    {
         public  UInt16 brkval; /// heap top
     public  UInt16 freemem; /// free memory
    
    };

/**
 * @brief Pack a meminfo message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param brkval heap top
 * @param freemem free memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_meminfo_pack(byte system_id, byte component_id, ref byte[] msg,
                               UInt16 public brkval, UInt16 public freemem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[4];
	_mav_put_UInt16(buf, 0, brkval);
	_mav_put_UInt16(buf, 2, freemem);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_meminfo_t packet;
	packet.brkval = brkval;
	packet.freemem = freemem;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_MEMINFO;
    return mavlink_finalize_message(msg, system_id, component_id, 4);
}
*/
/**
 * @brief Pack a meminfo message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param brkval heap top
 * @param freemem free memory
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_meminfo_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt16 public brkval,UInt16 public freemem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, brkval);
	_mav_put_UInt16(buf, 2, freemem);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_meminfo_t packet;
	packet.brkval = brkval;
	packet.freemem = freemem;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_MEMINFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4);
}
*/
/**
 * @brief Encode a meminfo struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param meminfo C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_meminfo_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_meminfo_t* meminfo)
{
    return mavlink_msg_meminfo_pack(system_id, component_id, msg, meminfo->brkval, meminfo->freemem);
}
*/
/**
 * @brief Send a meminfo message
 * @param chan MAVLink channel to send the message
 *
 * @param brkval heap top
 * @param freemem free memory
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_meminfo_send(mavlink_channel_t chan, UInt16 public brkval, UInt16 public freemem)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, brkval);
	_mav_put_UInt16(buf, 2, freemem);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMINFO, buf, 4);
#else
    mavlink_meminfo_t packet;
	packet.brkval = brkval;
	packet.freemem = freemem;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MEMINFO, (const char *)&packet, 4);
#endif
}

#endif
*/
// MESSAGE MEMINFO UNPACKING


/**
 * @brief Get field brkval from meminfo message
 *
 * @return heap top
 */
public static UInt16 mavlink_msg_meminfo_get_brkval(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Get field freemem from meminfo message
 *
 * @return free memory
 */
public static UInt16 mavlink_msg_meminfo_get_freemem(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Decode a meminfo message into a struct
 *
 * @param msg The message to decode
 * @param meminfo C-struct to decode the message contents into
 */
public static void mavlink_msg_meminfo_decode(byte[] msg, ref mavlink_meminfo_t meminfo)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	meminfo.brkval = mavlink_msg_meminfo_get_brkval(msg);
    	meminfo.freemem = mavlink_msg_meminfo_get_freemem(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(meminfo);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        meminfo = (mavlink_meminfo_t)Marshal.PtrToStructure(i, ((object)meminfo).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
