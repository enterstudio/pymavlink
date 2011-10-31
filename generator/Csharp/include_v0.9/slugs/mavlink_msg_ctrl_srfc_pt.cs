// MESSAGE CTRL_SRFC_PT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_CTRL_SRFC_PT = 181;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_ctrl_srfc_pt_t
    {
        /// <summary>
        /// The system setting the commands
        /// </summary>
        public  byte target;
            /// <summary>
        /// Bitfield containing the PT configuration
        /// </summary>
        public  UInt16 bitfieldPt;
    
    };

/// <summary>
/// * @brief Pack a ctrl_srfc_pt message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target The system setting the commands
/// * @param bitfieldPt Bitfield containing the PT configuration
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_ctrl_srfc_pt_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, UInt16 bitfieldPt)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(bitfieldPt),0,msg,1,sizeof(UInt16));

} else {
    mavlink_ctrl_srfc_pt_t packet = new mavlink_ctrl_srfc_pt_t();
	packet.target = target;
	packet.bitfieldPt = bitfieldPt;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;
    //return mavlink_finalize_message(msg, system_id, component_id, 3);
    return 0;
}

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
	_mav_put_byte(buf, 0, target);
	_mav_put_UInt16(buf, 1, bitfieldPt);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_ctrl_srfc_pt_t packet;
	packet.target = target;
	packet.bitfieldPt = bitfieldPt;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_CTRL_SRFC_PT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3);
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
	_mav_put_byte(buf, 0, target);
	_mav_put_UInt16(buf, 1, bitfieldPt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, buf, 3);
#else
    mavlink_ctrl_srfc_pt_t packet;
	packet.target = target;
	packet.bitfieldPt = bitfieldPt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CTRL_SRFC_PT, (const char *)&packet, 3);
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
    return getByte(msg,  0);
}

/**
 * @brief Get field bitfieldPt from ctrl_srfc_pt message
 *
 * @return Bitfield containing the PT configuration
 */
public static UInt16 mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  1);
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
    	ctrl_srfc_pt.target = mavlink_msg_ctrl_srfc_pt_get_target(msg);
    	ctrl_srfc_pt.bitfieldPt = mavlink_msg_ctrl_srfc_pt_get_bitfieldPt(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(ctrl_srfc_pt);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        ctrl_srfc_pt = (mavlink_ctrl_srfc_pt_t)Marshal.PtrToStructure(i, ((object)ctrl_srfc_pt).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}