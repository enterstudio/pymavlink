// MESSAGE WAYPOINT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_WAYPOINT = 39;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_waypoint_t
    {
         public  byte target_system; /// System ID
     public  byte target_component; /// Component ID
     public  UInt16 seq; /// Sequence
     public  byte frame; /// The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
     public  byte command; /// The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
     public  byte current; /// false:0, true:1
     public  byte autocontinue; /// autocontinue to next wp
     public  Single param1; /// PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
     public  Single param2; /// PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
     public  Single param3; /// PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
     public  Single param4; /// PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
     public  Single x; /// PARAM5 / local: x position, global: latitude
     public  Single y; /// PARAM6 / y position: global: longitude
     public  Single z; /// PARAM7 / z position: global: altitude
    
    };

/**
 * @brief Pack a waypoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_waypoint_pack(byte system_id, byte component_id, byte[] msg,
                               byte target_system, byte target_component, UInt16 seq, byte frame, byte command, byte current, byte autocontinue, Single param1, Single param2, Single param3, Single param4, Single x, Single y, Single z)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target_system),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(target_component),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(seq),0,msg,2,sizeof(UInt16));
	Array.Copy(BitConverter.GetBytes(frame),0,msg,4,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(command),0,msg,5,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(current),0,msg,6,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(autocontinue),0,msg,7,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(param1),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param2),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param3),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(param4),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(x),0,msg,24,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(y),0,msg,28,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(z),0,msg,32,sizeof(Single));

} else {
    mavlink_waypoint_t packet = new mavlink_waypoint_t();
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;

        
        int len = 36;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_WAYPOINT;
    //return mavlink_finalize_message(msg, system_id, component_id, 36);
    return 0;
}

/**
 * @brief Pack a waypoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_waypoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target_system,byte public target_component,UInt16 public seq,byte public frame,byte public command,byte public current,byte public autocontinue,Single public param1,Single public param2,Single public param3,Single public param4,Single public x,Single public y,Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, seq);
	_mav_put_byte(buf, 4, frame);
	_mav_put_byte(buf, 5, command);
	_mav_put_byte(buf, 6, current);
	_mav_put_byte(buf, 7, autocontinue);
	_mav_put_Single(buf, 8, param1);
	_mav_put_Single(buf, 12, param2);
	_mav_put_Single(buf, 16, param3);
	_mav_put_Single(buf, 20, param4);
	_mav_put_Single(buf, 24, x);
	_mav_put_Single(buf, 28, y);
	_mav_put_Single(buf, 32, z);

        memcpy(_MAV_PAYLOAD(msg), buf, 36);
#else
    mavlink_waypoint_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;

        memcpy(_MAV_PAYLOAD(msg), &packet, 36);
#endif

    msg->msgid = MAVLINK_MSG_ID_WAYPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36);
}
*/
/**
 * @brief Encode a waypoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_waypoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_t* waypoint)
{
    return mavlink_msg_waypoint_pack(system_id, component_id, msg, waypoint->target_system, waypoint->target_component, waypoint->seq, waypoint->frame, waypoint->command, waypoint->current, waypoint->autocontinue, waypoint->param1, waypoint->param2, waypoint->param3, waypoint->param4, waypoint->x, waypoint->y, waypoint->z);
}
*/
/**
 * @brief Send a waypoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param seq Sequence
 * @param frame The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 * @param command The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 * @param current false:0, true:1
 * @param autocontinue autocontinue to next wp
 * @param param1 PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 * @param param2 PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 * @param param3 PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 * @param param4 PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 * @param x PARAM5 / local: x position, global: latitude
 * @param y PARAM6 / y position: global: longitude
 * @param z PARAM7 / z position: global: altitude
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_send(mavlink_channel_t chan, byte public target_system, byte public target_component, UInt16 public seq, byte public frame, byte public command, byte public current, byte public autocontinue, Single public param1, Single public param2, Single public param3, Single public param4, Single public x, Single public y, Single public z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[36];
	_mav_put_byte(buf, 0, target_system);
	_mav_put_byte(buf, 1, target_component);
	_mav_put_UInt16(buf, 2, seq);
	_mav_put_byte(buf, 4, frame);
	_mav_put_byte(buf, 5, command);
	_mav_put_byte(buf, 6, current);
	_mav_put_byte(buf, 7, autocontinue);
	_mav_put_Single(buf, 8, param1);
	_mav_put_Single(buf, 12, param2);
	_mav_put_Single(buf, 16, param3);
	_mav_put_Single(buf, 20, param4);
	_mav_put_Single(buf, 24, x);
	_mav_put_Single(buf, 28, y);
	_mav_put_Single(buf, 32, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, buf, 36);
#else
    mavlink_waypoint_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.seq = seq;
	packet.frame = frame;
	packet.command = command;
	packet.current = current;
	packet.autocontinue = autocontinue;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.param4 = param4;
	packet.x = x;
	packet.y = y;
	packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT, (const char *)&packet, 36);
#endif
}

#endif
*/
// MESSAGE WAYPOINT UNPACKING


/**
 * @brief Get field target_system from waypoint message
 *
 * @return System ID
 */
public static byte mavlink_msg_waypoint_get_target_system(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field target_component from waypoint message
 *
 * @return Component ID
 */
public static byte mavlink_msg_waypoint_get_target_component(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field seq from waypoint message
 *
 * @return Sequence
 */
public static UInt16 mavlink_msg_waypoint_get_seq(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  2);
}

/**
 * @brief Get field frame from waypoint message
 *
 * @return The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h
 */
public static byte mavlink_msg_waypoint_get_frame(byte[] msg)
{
    return getByte(msg,  4);
}

/**
 * @brief Get field command from waypoint message
 *
 * @return The scheduled action for the waypoint. see MAV_COMMAND in common.xml MAVLink specs
 */
public static byte mavlink_msg_waypoint_get_command(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Get field current from waypoint message
 *
 * @return false:0, true:1
 */
public static byte mavlink_msg_waypoint_get_current(byte[] msg)
{
    return getByte(msg,  6);
}

/**
 * @brief Get field autocontinue from waypoint message
 *
 * @return autocontinue to next wp
 */
public static byte mavlink_msg_waypoint_get_autocontinue(byte[] msg)
{
    return getByte(msg,  7);
}

/**
 * @brief Get field param1 from waypoint message
 *
 * @return PARAM1 / For NAV command waypoints: Radius in which the waypoint is accepted as reached, in meters
 */
public static Single mavlink_msg_waypoint_get_param1(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field param2 from waypoint message
 *
 * @return PARAM2 / For NAV command waypoints: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds
 */
public static Single mavlink_msg_waypoint_get_param2(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field param3 from waypoint message
 *
 * @return PARAM3 / For LOITER command waypoints: Orbit to circle around the waypoint, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise.
 */
public static Single mavlink_msg_waypoint_get_param3(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field param4 from waypoint message
 *
 * @return PARAM4 / For NAV and LOITER command waypoints: Yaw orientation in degrees, [0..360] 0 = NORTH
 */
public static Single mavlink_msg_waypoint_get_param4(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field x from waypoint message
 *
 * @return PARAM5 / local: x position, global: latitude
 */
public static Single mavlink_msg_waypoint_get_x(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field y from waypoint message
 *
 * @return PARAM6 / y position: global: longitude
 */
public static Single mavlink_msg_waypoint_get_y(byte[] msg)
{
    return BitConverter.ToSingle(msg,  28);
}

/**
 * @brief Get field z from waypoint message
 *
 * @return PARAM7 / z position: global: altitude
 */
public static Single mavlink_msg_waypoint_get_z(byte[] msg)
{
    return BitConverter.ToSingle(msg,  32);
}

/**
 * @brief Decode a waypoint message into a struct
 *
 * @param msg The message to decode
 * @param waypoint C-struct to decode the message contents into
 */
public static void mavlink_msg_waypoint_decode(byte[] msg, ref mavlink_waypoint_t waypoint)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	waypoint.target_system = mavlink_msg_waypoint_get_target_system(msg);
    	waypoint.target_component = mavlink_msg_waypoint_get_target_component(msg);
    	waypoint.seq = mavlink_msg_waypoint_get_seq(msg);
    	waypoint.frame = mavlink_msg_waypoint_get_frame(msg);
    	waypoint.command = mavlink_msg_waypoint_get_command(msg);
    	waypoint.current = mavlink_msg_waypoint_get_current(msg);
    	waypoint.autocontinue = mavlink_msg_waypoint_get_autocontinue(msg);
    	waypoint.param1 = mavlink_msg_waypoint_get_param1(msg);
    	waypoint.param2 = mavlink_msg_waypoint_get_param2(msg);
    	waypoint.param3 = mavlink_msg_waypoint_get_param3(msg);
    	waypoint.param4 = mavlink_msg_waypoint_get_param4(msg);
    	waypoint.x = mavlink_msg_waypoint_get_x(msg);
    	waypoint.y = mavlink_msg_waypoint_get_y(msg);
    	waypoint.z = mavlink_msg_waypoint_get_z(msg);
    
    } else {
        int len = 36; //Marshal.SizeOf(waypoint);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        waypoint = (mavlink_waypoint_t)Marshal.PtrToStructure(i, ((object)waypoint).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
