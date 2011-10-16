// MESSAGE SET_CAM_SHUTTER PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_CAM_SHUTTER = 151;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_cam_shutter_t
    {
         public  byte cam_no; /// Camera id
     public  byte cam_mode; /// Camera mode: 0 = auto, 1 = manual
     public  byte trigger_pin; /// Trigger pin, 0-3 for PtGrey FireFly
     public  UInt16 interval; /// Shutter interval, in microseconds
     public  UInt16 exposure; /// Exposure time, in microseconds
     public  Single gain; /// Camera gain
    
    };

/**
 * @brief Pack a set_cam_shutter message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_set_cam_shutter_pack(byte system_id, byte component_id, byte[] msg,
                               byte cam_no, byte cam_mode, byte trigger_pin, UInt16 interval, UInt16 exposure, Single gain)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(cam_no),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(cam_mode),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(trigger_pin),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(interval),0,msg,3,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(exposure),0,msg,5,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(gain),0,msg,7,sizeof(Single));

} else {
    mavlink_set_cam_shutter_t packet = new mavlink_set_cam_shutter_t();
	packet.cam_no = cam_no;
	packet.cam_mode = cam_mode;
	packet.trigger_pin = trigger_pin;
	packet.interval = interval;
	packet.exposure = exposure;
	packet.gain = gain;

        
        int len = 11;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SET_CAM_SHUTTER;
    //return mavlink_finalize_message(msg, system_id, component_id, 11);
    return 0;
}

/**
 * @brief Pack a set_cam_shutter message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_cam_shutter_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public cam_no,byte public cam_mode,byte public trigger_pin,UInt16 public interval,UInt16 public exposure,Single public gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[11];
	_mav_put_byte(buf, 0, cam_no);
	_mav_put_byte(buf, 1, cam_mode);
	_mav_put_byte(buf, 2, trigger_pin);
	_mav_put_UInt16(buf, 3, interval);
	_mav_put_UInt16(buf, 5, exposure);
	_mav_put_Single(buf, 7, gain);

        memcpy(_MAV_PAYLOAD(msg), buf, 11);
#else
    mavlink_set_cam_shutter_t packet;
	packet.cam_no = cam_no;
	packet.cam_mode = cam_mode;
	packet.trigger_pin = trigger_pin;
	packet.interval = interval;
	packet.exposure = exposure;
	packet.gain = gain;

        memcpy(_MAV_PAYLOAD(msg), &packet, 11);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_CAM_SHUTTER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 11);
}
*/
/**
 * @brief Encode a set_cam_shutter struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_cam_shutter C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_cam_shutter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_cam_shutter_t* set_cam_shutter)
{
    return mavlink_msg_set_cam_shutter_pack(system_id, component_id, msg, set_cam_shutter->cam_no, set_cam_shutter->cam_mode, set_cam_shutter->trigger_pin, set_cam_shutter->interval, set_cam_shutter->exposure, set_cam_shutter->gain);
}
*/
/**
 * @brief Send a set_cam_shutter message
 * @param chan MAVLink channel to send the message
 *
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_cam_shutter_send(mavlink_channel_t chan, byte public cam_no, byte public cam_mode, byte public trigger_pin, UInt16 public interval, UInt16 public exposure, Single public gain)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[11];
	_mav_put_byte(buf, 0, cam_no);
	_mav_put_byte(buf, 1, cam_mode);
	_mav_put_byte(buf, 2, trigger_pin);
	_mav_put_UInt16(buf, 3, interval);
	_mav_put_UInt16(buf, 5, exposure);
	_mav_put_Single(buf, 7, gain);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CAM_SHUTTER, buf, 11);
#else
    mavlink_set_cam_shutter_t packet;
	packet.cam_no = cam_no;
	packet.cam_mode = cam_mode;
	packet.trigger_pin = trigger_pin;
	packet.interval = interval;
	packet.exposure = exposure;
	packet.gain = gain;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CAM_SHUTTER, (const char *)&packet, 11);
#endif
}

#endif
*/
// MESSAGE SET_CAM_SHUTTER UNPACKING


/**
 * @brief Get field cam_no from set_cam_shutter message
 *
 * @return Camera id
 */
public static byte mavlink_msg_set_cam_shutter_get_cam_no(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field cam_mode from set_cam_shutter message
 *
 * @return Camera mode: 0 = auto, 1 = manual
 */
public static byte mavlink_msg_set_cam_shutter_get_cam_mode(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field trigger_pin from set_cam_shutter message
 *
 * @return Trigger pin, 0-3 for PtGrey FireFly
 */
public static byte mavlink_msg_set_cam_shutter_get_trigger_pin(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field interval from set_cam_shutter message
 *
 * @return Shutter interval, in microseconds
 */
public static UInt16 mavlink_msg_set_cam_shutter_get_interval(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  3);
}

/**
 * @brief Get field exposure from set_cam_shutter message
 *
 * @return Exposure time, in microseconds
 */
public static UInt16 mavlink_msg_set_cam_shutter_get_exposure(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  5);
}

/**
 * @brief Get field gain from set_cam_shutter message
 *
 * @return Camera gain
 */
public static Single mavlink_msg_set_cam_shutter_get_gain(byte[] msg)
{
    return BitConverter.ToSingle(msg,  7);
}

/**
 * @brief Decode a set_cam_shutter message into a struct
 *
 * @param msg The message to decode
 * @param set_cam_shutter C-struct to decode the message contents into
 */
public static void mavlink_msg_set_cam_shutter_decode(byte[] msg, ref mavlink_set_cam_shutter_t set_cam_shutter)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	set_cam_shutter.cam_no = mavlink_msg_set_cam_shutter_get_cam_no(msg);
    	set_cam_shutter.cam_mode = mavlink_msg_set_cam_shutter_get_cam_mode(msg);
    	set_cam_shutter.trigger_pin = mavlink_msg_set_cam_shutter_get_trigger_pin(msg);
    	set_cam_shutter.interval = mavlink_msg_set_cam_shutter_get_interval(msg);
    	set_cam_shutter.exposure = mavlink_msg_set_cam_shutter_get_exposure(msg);
    	set_cam_shutter.gain = mavlink_msg_set_cam_shutter_get_gain(msg);
    
    } else {
        int len = 11; //Marshal.SizeOf(set_cam_shutter);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        set_cam_shutter = (mavlink_set_cam_shutter_t)Marshal.PtrToStructure(i, ((object)set_cam_shutter).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
