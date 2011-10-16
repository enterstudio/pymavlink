// MESSAGE MID_LVL_CMDS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MID_LVL_CMDS = 180;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_mid_lvl_cmds_t
    {
         public  Single hCommand; /// Commanded Airspeed
     public  Single uCommand; /// Log value 2 
     public  Single rCommand; /// Log value 3 
     public  byte target; /// The system setting the commands
    
    };

/**
 * @brief Pack a mid_lvl_cmds message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system setting the commands
 * @param hCommand Commanded Airspeed
 * @param uCommand Log value 2 
 * @param rCommand Log value 3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_mid_lvl_cmds_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, Single hCommand, Single uCommand, Single rCommand)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(hCommand),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(uCommand),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(rCommand),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(target),0,msg,12,sizeof(byte));

} else {
    mavlink_mid_lvl_cmds_t packet = new mavlink_mid_lvl_cmds_t();
	packet.hCommand = hCommand;
	packet.uCommand = uCommand;
	packet.rCommand = rCommand;
	packet.target = target;

        
        int len = 13;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;
    //return mavlink_finalize_message(msg, system_id, component_id, 13, 146);
    return 0;
}

/**
 * @brief Pack a mid_lvl_cmds message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the commands
 * @param hCommand Commanded Airspeed
 * @param uCommand Log value 2 
 * @param rCommand Log value 3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_mid_lvl_cmds_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,Single public hCommand,Single public uCommand,Single public rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[13];
	_mav_put_Single(buf, 0, hCommand);
	_mav_put_Single(buf, 4, uCommand);
	_mav_put_Single(buf, 8, rCommand);
	_mav_put_byte(buf, 12, target);

        memcpy(_MAV_PAYLOAD(msg), buf, 13);
#else
    mavlink_mid_lvl_cmds_t packet;
	packet.hCommand = hCommand;
	packet.uCommand = uCommand;
	packet.rCommand = rCommand;
	packet.target = target;

        memcpy(_MAV_PAYLOAD(msg), &packet, 13);
#endif

    msg->msgid = MAVLINK_MSG_ID_MID_LVL_CMDS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 13, 146);
}
*/
/**
 * @brief Encode a mid_lvl_cmds struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mid_lvl_cmds C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_mid_lvl_cmds_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mid_lvl_cmds_t* mid_lvl_cmds)
{
    return mavlink_msg_mid_lvl_cmds_pack(system_id, component_id, msg, mid_lvl_cmds->target, mid_lvl_cmds->hCommand, mid_lvl_cmds->uCommand, mid_lvl_cmds->rCommand);
}
*/
/**
 * @brief Send a mid_lvl_cmds message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the commands
 * @param hCommand Commanded Airspeed
 * @param uCommand Log value 2 
 * @param rCommand Log value 3 
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mid_lvl_cmds_send(mavlink_channel_t chan, byte public target, Single public hCommand, Single public uCommand, Single public rCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[13];
	_mav_put_Single(buf, 0, hCommand);
	_mav_put_Single(buf, 4, uCommand);
	_mav_put_Single(buf, 8, rCommand);
	_mav_put_byte(buf, 12, target);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, buf, 13, 146);
#else
    mavlink_mid_lvl_cmds_t packet;
	packet.hCommand = hCommand;
	packet.uCommand = uCommand;
	packet.rCommand = rCommand;
	packet.target = target;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MID_LVL_CMDS, (const char *)&packet, 13, 146);
#endif
}

#endif
*/
// MESSAGE MID_LVL_CMDS UNPACKING


/**
 * @brief Get field target from mid_lvl_cmds message
 *
 * @return The system setting the commands
 */
public static byte mavlink_msg_mid_lvl_cmds_get_target(byte[] msg)
{
    return getByte(msg,  12);
}

/**
 * @brief Get field hCommand from mid_lvl_cmds message
 *
 * @return Commanded Airspeed
 */
public static Single mavlink_msg_mid_lvl_cmds_get_hCommand(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field uCommand from mid_lvl_cmds message
 *
 * @return Log value 2 
 */
public static Single mavlink_msg_mid_lvl_cmds_get_uCommand(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field rCommand from mid_lvl_cmds message
 *
 * @return Log value 3 
 */
public static Single mavlink_msg_mid_lvl_cmds_get_rCommand(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Decode a mid_lvl_cmds message into a struct
 *
 * @param msg The message to decode
 * @param mid_lvl_cmds C-struct to decode the message contents into
 */
public static void mavlink_msg_mid_lvl_cmds_decode(byte[] msg, ref mavlink_mid_lvl_cmds_t mid_lvl_cmds)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	mid_lvl_cmds.hCommand = mavlink_msg_mid_lvl_cmds_get_hCommand(msg);
    	mid_lvl_cmds.uCommand = mavlink_msg_mid_lvl_cmds_get_uCommand(msg);
    	mid_lvl_cmds.rCommand = mavlink_msg_mid_lvl_cmds_get_rCommand(msg);
    	mid_lvl_cmds.target = mavlink_msg_mid_lvl_cmds_get_target(msg);
    
    } else {
        int len = 13; //Marshal.SizeOf(mid_lvl_cmds);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        mid_lvl_cmds = (mavlink_mid_lvl_cmds_t)Marshal.PtrToStructure(i, ((object)mid_lvl_cmds).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
