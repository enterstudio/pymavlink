// MESSAGE HEARTBEAT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_heartbeat_t
    {
         public  UInt32 custom_mode; /// Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
     public  byte type; /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
     public  byte autopilot; /// Autopilot type / class. defined in MAV_CLASS ENUM
     public  byte base_mode; /// System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
     public  byte system_status; /// System status flag, see MAV_STATUS ENUM
     public  byte mavlink_version; /// MAVLink version
    
    };

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_heartbeat_pack(byte system_id, byte component_id, byte[] msg,
                               byte type, byte autopilot, byte base_mode, UInt32 custom_mode, byte system_status)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(custom_mode),0,msg,0,sizeof(UInt32));
	Array.Copy(BitConverter.GetBytes(type),0,msg,4,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(autopilot),0,msg,5,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(base_mode),0,msg,6,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(system_status),0,msg,7,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(2),0,msg,8,sizeof(byte));

} else {
    mavlink_heartbeat_t packet = new mavlink_heartbeat_t();
	packet.custom_mode = custom_mode;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 2;

        
        int len = 9;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    //return mavlink_finalize_message(msg, system_id, component_id, 9, 50);
    return 0;
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 * @param system_status System status flag, see MAV_STATUS ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,byte public autopilot,byte public base_mode,UInt32 public custom_mode,byte public system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[9];
	_mav_put_UInt32(buf, 0, custom_mode);
	_mav_put_byte(buf, 4, type);
	_mav_put_byte(buf, 5, autopilot);
	_mav_put_byte(buf, 6, base_mode);
	_mav_put_byte(buf, 7, system_status);
	_mav_put_byte(buf, 8, 2);

        memcpy(_MAV_PAYLOAD(msg), buf, 9);
#else
    mavlink_heartbeat_t packet;
	packet.custom_mode = custom_mode;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 9);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 9, 50);
}
*/
/**
 * @brief Encode a heartbeat struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot, heartbeat->base_mode, heartbeat->custom_mode, heartbeat->system_status);
}
*/
/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Autopilot type / class. defined in MAV_CLASS ENUM
 * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 * @param custom_mode Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 * @param system_status System status flag, see MAV_STATUS ENUM
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, byte public type, byte public autopilot, byte public base_mode, UInt32 public custom_mode, byte public system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[9];
	_mav_put_UInt32(buf, 0, custom_mode);
	_mav_put_byte(buf, 4, type);
	_mav_put_byte(buf, 5, autopilot);
	_mav_put_byte(buf, 6, base_mode);
	_mav_put_byte(buf, 7, system_status);
	_mav_put_byte(buf, 8, 2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 9, 50);
#else
    mavlink_heartbeat_t packet;
	packet.custom_mode = custom_mode;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 9, 50);
#endif
}

#endif
*/
// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field type from heartbeat message
 *
 * @return Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 */
public static byte mavlink_msg_heartbeat_get_type(byte[] msg)
{
    return getByte(msg,  4);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Autopilot type / class. defined in MAV_CLASS ENUM
 */
public static byte mavlink_msg_heartbeat_get_autopilot(byte[] msg)
{
    return getByte(msg,  5);
}

/**
 * @brief Get field base_mode from heartbeat message
 *
 * @return System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
 */
public static byte mavlink_msg_heartbeat_get_base_mode(byte[] msg)
{
    return getByte(msg,  6);
}

/**
 * @brief Get field custom_mode from heartbeat message
 *
 * @return Navigation mode bitfield, see MAV_AUTOPILOT_CUSTOM_MODE ENUM for some examples. This field is autopilot-specific.
 */
public static UInt32 mavlink_msg_heartbeat_get_custom_mode(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  0);
}

/**
 * @brief Get field system_status from heartbeat message
 *
 * @return System status flag, see MAV_STATUS ENUM
 */
public static byte mavlink_msg_heartbeat_get_system_status(byte[] msg)
{
    return getByte(msg,  7);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
public static byte mavlink_msg_heartbeat_get_mavlink_version(byte[] msg)
{
    return getByte(msg,  8);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
public static void mavlink_msg_heartbeat_decode(byte[] msg, ref mavlink_heartbeat_t heartbeat)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	heartbeat.custom_mode = mavlink_msg_heartbeat_get_custom_mode(msg);
    	heartbeat.type = mavlink_msg_heartbeat_get_type(msg);
    	heartbeat.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
    	heartbeat.base_mode = mavlink_msg_heartbeat_get_base_mode(msg);
    	heartbeat.system_status = mavlink_msg_heartbeat_get_system_status(msg);
    	heartbeat.mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
    
    } else {
        int len = 9; //Marshal.SizeOf(heartbeat);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        heartbeat = (mavlink_heartbeat_t)Marshal.PtrToStructure(i, ((object)heartbeat).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
