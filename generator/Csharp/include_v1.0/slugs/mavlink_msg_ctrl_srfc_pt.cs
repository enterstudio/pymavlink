// MESSAGE CTRL_SRFC_PT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_CTRL_SRFC_PT = 181;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_ctrl_srfc_pt_t
    {
         public  UInt16 bitfieldPt; /// Bitfield containing the PT configuration
     public  byte target; /// The system setting the commands
    
    };

/**
 * @brief Pack a ctrl_srfc_pt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the PT configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_ctrl_srfc_pt_pack(byte system_id, byte component_id, ref byte[] msg,
                               byte public target, UInt16 public bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[3];
	_mav_put_UInt16(buf, 0, bitfieldPt);
	_mav_put_byte(buf, 2, target);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_ctrl_srfc_pt_t packet;
	packet.bitfieldPt = bitfieldPt;
	packet.target = target;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;
    return mavlink_finalize_message(msg, system_id, component_id, 3, 104);
}
*/
/**
 * @brief Pack a ctrl_srfc_pt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the PT configuration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_ctrl_srfc_pt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,UInt16 public bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_UInt16(buf, 0, bitfieldPt);
	_mav_put_byte(buf, 2, target);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_ctrl_srfc_pt_t packet;
	packet.bitfieldPt = bitfieldPt;
	packet.target = target;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 104);
}
*/
/**
 * @brief Encode a ctrl_srfc_pt struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ctrl_srfc_pt C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_ctrl_srfc_pt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ctrl_srfc_pt_t* ctrl_srfc_pt)
{
    return mavlink_msg_ctrl_srfc_pt_pack(system_id, component_id, msg, ctrl_srfc_pt->target, ctrl_srfc_pt->bitfieldPt);
}
*/
/**
 * @brief Send a ctrl_srfc_pt message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param bitfieldPt Bitfield containing the PT configuration
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ctrl_srfc_pt_send(mavlink_channel_t chan, byte public target, UInt16 public bitfieldPt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_UInt16(buf, 0, bitfieldPt);
	_mav_put_byte(buf, 2, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, buf, 3, 104);
#else
    mavlink_ctrl_srfc_pt_t packet;
	packet.bitfieldPt = bitfieldPt;
	packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, (const char *)&packet, 3, 104);
#endif
}

#endif
*/
// MESSAGE CTRL_SRFC_PT UNPACKING


/**
 * @brief Get field target from ctrl_srfc_pt message
 *
 * @return The system setting the commands
 */
public static byte mavlink_msg_ctrl_srfc_pt_get_target(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field bitfieldPt from ctrl_srfc_pt message
 *
 * @return Bitfield containing the PT configuration
 */
public static UInt16 mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Decode a ctrl_srfc_pt message into a struct
 *
 * @param msg The message to decode
 * @param ctrl_srfc_pt C-struct to decode the message contents into
 */
public static void mavlink_msg_ctrl_srfc_pt_decode(byte[] msg, ref mavlink_ctrl_srfc_pt_t ctrl_srfc_pt)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	ctrl_srfc_pt.bitfieldPt = mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(msg);
	ctrl_srfc_pt.target = mavlink_msg_ctrl_srfc_pt_get_target(msg);
} else {
    int len = 3; //Marshal.SizeOf(ctrl_srfc_pt);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    ctrl_srfc_pt = (mavlink_ctrl_srfc_pt_t)Marshal.PtrToStructure(i, ((object)ctrl_srfc_pt).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
