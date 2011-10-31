// MESSAGE MISSION_ITEM PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_MISSION_ITEM = 39;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_mission_item_t
    {
        /// <summary>
        /// PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
        /// </summary>
        public  Single param1;
            /// <summary>
        /// PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
        /// </summary>
        public  Single param2;
            /// <summary>
        /// PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
        /// </summary>
        public  Single param3;
            /// <summary>
        /// PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
        /// </summary>
        public  Single param4;
            /// <summary>
        /// PARAM5 / local: x position, global: latitude
        /// </summary>
        public  Single x;
            /// <summary>
        /// PARAM6 / y position: global: longitude
        /// </summary>
        public  Single y;
            /// <summary>
        /// PARAM7 / z position: global: altitude
        /// </summary>
        public  Single z;
            /// <summary>
        /// Sequence
        /// </summary>
        public  UInt16 seq;
            /// <summary>
        /// System ID
        /// </summary>
        public  byte target_system;
            /// <summary>
        /// Component ID
        /// </summary>
        public  byte target_component;
            /// <summary>
        /// The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
        /// </summary>
        public  byte frame;
            /// <summary>
        /// The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
        /// </summary>
        public  byte command;
            /// <summary>
        /// false:0, true:1
        /// </summary>
        public  byte current;
            /// <summary>
        /// autocontinue to next wp
        /// </summary>
        public  byte autocontinue;
    
    };

/// <summary>
/// * @brief Pack a mission_item message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target_system System ID
/// * @param target_component Component ID
/// * @param seq Sequence
/// * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
/// * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
/// * @param current false:0, true:1
/// * @param autocontinue autocontinue to next wp
/// * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
/// * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
/// * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
/// * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
/// * @param x PARAM5 / local: x position, global: latitude
/// * @param y PARAM6 / y position: global: longitude
/// * @param z PARAM7 / z position: global: altitude
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_mission_item_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, UInt16 seq, byte frame, byte command, byte current, byte autocontinue, Single param1, Single param2, Single param3, Single param4, Single x, Single y, Single z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(param1),0,msg,0,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param2),0,msg,4,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param3),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param4),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(x),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(seq),0,msg,28,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,30,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,31,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(frame),0,msg,32,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(command),0,msg,33,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(current),0,msg,34,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(autocontinue),0,msg,35,sizeof(byte));

} else {
    mavlink_mission_item_t packet = new mavlink_mission_item_t();
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;

        
        int len = 36;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_MISSION_ITEM;
    //return mavlink_finalize_message(msg, system_id, component_id, 36, 158);
    return 0;
}

/**
 * @brief Pack a mission_item message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_mission_item_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,UInt16 public seq,byte public frame,byte public command,byte public current,byte public autocontinue,Single public param1,Single public param2,Single public param3,Single public param4,Single public x,Single public y,Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, x);
	_mav_put_Single(buf, 20, y);
	_mav_put_Single(buf, 24, z);
	_mav_put_UInt16(buf, 28, seq);
	_mav_put_byte(buf, 30, target_system);
	_mav_put_byte(buf, 31, target_component);
	_mav_put_byte(buf, 32, frame);
	_mav_put_byte(buf, 33, command);
	_mav_put_byte(buf, 34, current);
	_mav_put_byte(buf, 35, autocontinue);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
    mavlink_mission_item_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 158);
}
*/
/**
 * @brief Encode a mission_item struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_item C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_mission_item_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_item_t* mission_item)
{
    return mavlink_msg_mission_item_pack(system_id, component_id, msg, mission_item->target_system, mission_item->target_component, mission_item->seq, mission_item->frame, mission_item->command, mission_item->current, mission_item->autocontinue, mission_item->param1, mission_item->param2, mission_item->param3, mission_item->param4, mission_item->x, mission_item->y, mission_item->z);
}
*/
/**
 * @brief Send a mission_item message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_item_send(mavlink_channel_t chan, byte public target_system, byte public target_component, UInt16 public seq, byte public frame, byte public command, byte public current, byte public autocontinue, Single public param1, Single public param2, Single public param3, Single public param4, Single public x, Single public y, Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_Single(buf, 0, param1);
	_mav_put_Single(buf, 4, param2);
	_mav_put_Single(buf, 8, param3);
	_mav_put_Single(buf, 12, param4);
	_mav_put_Single(buf, 16, x);
	_mav_put_Single(buf, 20, y);
	_mav_put_Single(buf, 24, z);
	_mav_put_UInt16(buf, 28, seq);
	_mav_put_byte(buf, 30, target_system);
	_mav_put_byte(buf, 31, target_component);
	_mav_put_byte(buf, 32, frame);
	_mav_put_byte(buf, 33, command);
	_mav_put_byte(buf, 34, current);
	_mav_put_byte(buf, 35, autocontinue);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM, buf, 36, 158);
#else
    mavlink_mission_item_t packet;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.seq = seq;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_ITEM, (const char *)&packet, 36, 158);
#endif
}

#endif
*/
// MESSAGE MISSION_ITEM UNPACKING


/**
 * @brief Get field target_system from mission_item message
 *
 * @return System ID
 */
public static byte mavlink_msg_mission_item_get_target_system(byte[] msg)
{
    return getByte(msg,  30);
}

/**
 * @brief Get field target_component from mission_item message
 *
 * @return Component ID
 */
public static byte mavlink_msg_mission_item_get_target_component(byte[] msg)
{
    return getByte(msg,  31);
}

/**
 * @brief Get field seq from mission_item message
 *
 * @return Sequence
 */
public static UInt16 mavlink_msg_mission_item_get_seq(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  28);
}

/**
 * @brief Get field frame from mission_item message
 *
 * @return The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
 */
public static byte mavlink_msg_mission_item_get_frame(byte[] msg)
{
    return getByte(msg,  32);
}

/**
 * @brief Get field command from mission_item message
 *
 * @return The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
 */
public static byte mavlink_msg_mission_item_get_command(byte[] msg)
{
    return getByte(msg,  33);
}

/**
 * @brief Get field current from mission_item message
 *
 * @return false:0, true:1
 */
public static byte mavlink_msg_mission_item_get_current(byte[] msg)
{
    return getByte(msg,  34);
}

/**
 * @brief Get field autocontinue from mission_item message
 *
 * @return autocontinue to next wp
 */
public static byte mavlink_msg_mission_item_get_autocontinue(byte[] msg)
{
    return getByte(msg,  35);
}

/**
 * @brief Get field param1 from mission_item message
 *
 * @return PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters
 */
public static Single mavlink_msg_mission_item_get_param1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field param2 from mission_item message
 *
 * @return PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 */
public static Single mavlink_msg_mission_item_get_param2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field param3 from mission_item message
 *
 * @return PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 */
public static Single mavlink_msg_mission_item_get_param3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field param4 from mission_item message
 *
 * @return PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH
 */
public static Single mavlink_msg_mission_item_get_param4(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field x from mission_item message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
public static Single mavlink_msg_mission_item_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field y from mission_item message
 *
 * @return PARAM6 / y position: global: longitude
 */
public static Single mavlink_msg_mission_item_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field z from mission_item message
 *
 * @return PARAM7 / z position: global: altitude
 */
public static Single mavlink_msg_mission_item_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Decode a mission_item message into a struct
 *
 * @param msg The message to decode
 * @param mission_item C-struct to decode the message contents into
 */
public static void mavlink_msg_mission_item_decode(byte[] msg, ref mavlink_mission_item_t mission_item)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	mission_item.param1 = mavlink_msg_mission_item_get_param1(msg);
    	mission_item.param2 = mavlink_msg_mission_item_get_param2(msg);
    	mission_item.param3 = mavlink_msg_mission_item_get_param3(msg);
    	mission_item.param4 = mavlink_msg_mission_item_get_param4(msg);
    	mission_item.x = mavlink_msg_mission_item_get_x(msg);
    	mission_item.y = mavlink_msg_mission_item_get_y(msg);
    	mission_item.z = mavlink_msg_mission_item_get_z(msg);
    	mission_item.seq = mavlink_msg_mission_item_get_seq(msg);
    	mission_item.target_system = mavlink_msg_mission_item_get_target_system(msg);
    	mission_item.target_component = mavlink_msg_mission_item_get_target_component(msg);
    	mission_item.frame = mavlink_msg_mission_item_get_frame(msg);
    	mission_item.command = mavlink_msg_mission_item_get_command(msg);
    	mission_item.current = mavlink_msg_mission_item_get_current(msg);
    	mission_item.autocontinue = mavlink_msg_mission_item_get_autocontinue(msg);
    
    } else {
        int len = 36; //Marshal.SizeOf(mission_item);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        mission_item = (mavlink_mission_item_t)Marshal.PtrToStructure(i, ((object)mission_item).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
