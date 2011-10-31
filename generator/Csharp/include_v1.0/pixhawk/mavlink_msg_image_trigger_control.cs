// MESSAGE IMAGE_TRIGGER_CONTROL PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL = 153;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_image_trigger_control_t
    {
        /// <summary>
        /// 0 to disable, 1 to enable
        /// </summary>
        public  byte enable;
    
    };

/// <summary>
/// * @brief Pack a image_trigger_control message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param enable 0 to disable, 1 to enable
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_image_trigger_control_pack(byte system_id, byte component_id, byte[] msg,
                               byte enable)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(enable),0,msg,0,sizeof(byte));

} else {
    mavlink_image_trigger_control_t packet = new mavlink_image_trigger_control_t();
	packet.enable = enable;

        
        int len = 1;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;
    //return mavlink_finalize_message(msg, system_id, component_id, 1, 95);
    return 0;
}

/**
 * @brief Pack a image_trigger_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable 0 to disable, 1 to enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_image_trigger_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[1];
	_mav_put_byte(buf, 0, enable);

        memcpy(_MAV_PAYLOAD(msg), buf, 1);
#else
    mavlink_image_trigger_control_t packet;
	packet.enable = enable;

        memcpy(_MAV_PAYLOAD(msg), &packet, 1);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 95);
}
*/
/**
 * @brief Encode a image_trigger_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_trigger_control C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_image_trigger_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_trigger_control_t* image_trigger_control)
{
    return mavlink_msg_image_trigger_control_pack(system_id, component_id, msg, image_trigger_control->enable);
}
*/
/**
 * @brief Send a image_trigger_control message
 * @param chan MAVLink channel to send the message
 *
 * @param enable 0 to disable, 1 to enable
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_trigger_control_send(mavlink_channel_t chan, byte public enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[1];
	_mav_put_byte(buf, 0, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, buf, 1, 95);
#else
    mavlink_image_trigger_control_t packet;
	packet.enable = enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, (const char *)&packet, 1, 95);
#endif
}

#endif
*/
// MESSAGE IMAGE_TRIGGER_CONTROL UNPACKING


/**
 * @brief Get field enable from image_trigger_control message
 *
 * @return 0 to disable, 1 to enable
 */
public static byte mavlink_msg_image_trigger_control_get_enable(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Decode a image_trigger_control message into a struct
 *
 * @param msg The message to decode
 * @param image_trigger_control C-struct to decode the message contents into
 */
public static void mavlink_msg_image_trigger_control_decode(byte[] msg, ref mavlink_image_trigger_control_t image_trigger_control)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	image_trigger_control.enable = mavlink_msg_image_trigger_control_get_enable(msg);
    
    } else {
        int len = 1; //Marshal.SizeOf(image_trigger_control);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        image_trigger_control = (mavlink_image_trigger_control_t)Marshal.PtrToStructure(i, ((object)image_trigger_control).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
