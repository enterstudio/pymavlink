// MESSAGE ACTION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_ACTION = 10;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_action_t
    {
         public  byte target; /// The system executing the action
     public  byte target_component; /// The component executing the action
     public  byte action; /// The action id
    
    };

/**
 * @brief Pack a action message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target The system executing the action
 * @param target_component The component executing the action
 * @param action The action id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_action_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, byte target_component, byte action)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(action),0,msg,2,sizeof(byte));

} else {
    mavlink_action_t packet = new mavlink_action_t();
	packet.target = target;
	packet.target_component = target_component;
	packet.action = action;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_ACTION;
    //return mavlink_finalize_message(msg, system_id, component_id, 3);
    return 0;
}

/**
 * @brief Pack a action message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system executing the action
 * @param target_component The component executing the action
 * @param action The action id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_action_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,byte public target_component,byte public action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, action);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_action_t packet;
	packet.target = target;
	packet.target_component = target_component;
	packet.action = action;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3);
}
*/
/**
 * @brief Encode a action struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param action C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_action_t* action)
{
    return mavlink_msg_action_pack(system_id, component_id, msg, action->target, action->target_component, action->action);
}
*/
/**
 * @brief Send a action message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system executing the action
 * @param target_component The component executing the action
 * @param action The action id
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_action_send(mavlink_channel_t chan, byte public target, byte public target_component, byte public action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, target);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_byte(buf, 2, action);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTION, buf, 3);
#else
    mavlink_action_t packet;
	packet.target = target;
	packet.target_component = target_component;
	packet.action = action;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTION, (const char *)&packet, 3);
#endif
}

#endif
*/
// MESSAGE ACTION UNPACKING


/**
 * @brief Get field target from action message
 *
 * @return The system executing the action
 */
public static byte mavlink_msg_action_get_target(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from action message
 *
 * @return The component executing the action
 */
public static byte mavlink_msg_action_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field action from action message
 *
 * @return The action id
 */
public static byte mavlink_msg_action_get_action(byte[] msg)
{
    return getByte(msg,  2);
}

/**
 * @brief Decode a action message into a struct
 *
 * @param msg The message to decode
 * @param action C-struct to decode the message contents into
 */
public static void mavlink_msg_action_decode(byte[] msg, ref mavlink_action_t action)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	action.target = mavlink_msg_action_get_target(msg);
    	action.target_component = mavlink_msg_action_get_target_component(msg);
    	action.action = mavlink_msg_action_get_action(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(action);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        action = (mavlink_action_t)Marshal.PtrToStructure(i, ((object)action).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
