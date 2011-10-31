// MESSAGE SLUGS_ACTION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SLUGS_ACTION = 183;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_slugs_action_t
    {
        /// <summary>
        /// Value associated with the action
        /// </summary>
        public  UInt16 actionVal;
            /// <summary>
        /// The system reporting the action
        /// </summary>
        public  byte target;
            /// <summary>
        /// Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
        /// </summary>
        public  byte actionId;
    
    };

/// <summary>
/// * @brief Pack a slugs_action message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target The system reporting the action
/// * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
/// * @param actionVal Value associated with the action
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_slugs_action_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, byte actionId, UInt16 actionVal)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(actionVal),0,msg,0,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(target),0,msg,2,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(actionId),0,msg,3,sizeof(byte));

} else {
    mavlink_slugs_action_t packet = new mavlink_slugs_action_t();
	packet.actionVal = actionVal;
	packet.target = target;
	packet.actionId = actionId;

        
        int len = 4;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SLUGS_ACTION;
    //return mavlink_finalize_message(msg, system_id, component_id, 4, 65);
    return 0;
}

/**
 * @brief Pack a slugs_action message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system reporting the action
 * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 * @param actionVal Value associated with the action
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_slugs_action_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,byte public actionId,UInt16 public actionVal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, actionVal);
	_mav_put_byte(buf, 2, target);
	_mav_put_byte(buf, 3, actionId);

        memcpy(_MAV_PAYLOAD(msg), buf, 4);
#else
    mavlink_slugs_action_t packet;
	packet.actionVal = actionVal;
	packet.target = target;
	packet.actionId = actionId;

        memcpy(_MAV_PAYLOAD(msg), &packet, 4);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_ACTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 65);
}
*/
/**
 * @brief Encode a slugs_action struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_action C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_slugs_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_action_t* slugs_action)
{
    return mavlink_msg_slugs_action_pack(system_id, component_id, msg, slugs_action->target, slugs_action->actionId, slugs_action->actionVal);
}
*/
/**
 * @brief Send a slugs_action message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system reporting the action
 * @param actionId Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 * @param actionVal Value associated with the action
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_action_send(mavlink_channel_t chan, byte public target, byte public actionId, UInt16 public actionVal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[4];
	_mav_put_UInt16(buf, 0, actionVal);
	_mav_put_byte(buf, 2, target);
	_mav_put_byte(buf, 3, actionId);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_ACTION, buf, 4, 65);
#else
    mavlink_slugs_action_t packet;
	packet.actionVal = actionVal;
	packet.target = target;
	packet.actionId = actionId;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_ACTION, (const char *)&packet, 4, 65);
#endif
}

#endif
*/
// MESSAGE SLUGS_ACTION UNPACKING


/**
 * @brief Get field target from slugs_action message
 *
 * @return The system reporting the action
 */
public static byte mavlink_msg_slugs_action_get_target(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Get field actionId from slugs_action message
 *
 * @return Action ID. See apDefinitions.h in the SLUGS /clib directory for the ID names
 */
public static byte mavlink_msg_slugs_action_get_actionId(byte[] msg)
{
    return getByte(msg,  3);
}

/**
 * @brief Get field actionVal from slugs_action message
 *
 * @return Value associated with the action
 */
public static UInt16 mavlink_msg_slugs_action_get_actionVal(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  0);
}

/**
 * @brief Decode a slugs_action message into a struct
 *
 * @param msg The message to decode
 * @param slugs_action C-struct to decode the message contents into
 */
public static void mavlink_msg_slugs_action_decode(byte[] msg, ref mavlink_slugs_action_t slugs_action)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	slugs_action.actionVal = mavlink_msg_slugs_action_get_actionVal(msg);
    	slugs_action.target = mavlink_msg_slugs_action_get_target(msg);
    	slugs_action.actionId = mavlink_msg_slugs_action_get_actionId(msg);
    
    } else {
        int len = 4; //Marshal.SizeOf(slugs_action);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        slugs_action = (mavlink_slugs_action_t)Marshal.PtrToStructure(i, ((object)slugs_action).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
