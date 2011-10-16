// MESSAGE HEARTBEAT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_HEARTBEAT = 0;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_heartbeat_t
    {
         public  byte type; /// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
     public  byte autopilot; /// Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
     public  byte mavlink_version; /// MAVLink version
    
    };

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_heartbeat_pack(byte system_id, byte component_id, byte[] msg,
                               byte type, byte autopilot)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(type),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(autopilot),0,msg,1,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(2),0,msg,2,sizeof(byte));

} else {
    mavlink_heartbeat_t packet = new mavlink_heartbeat_t();
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = 2;

        
        int len = 3;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    //return mavlink_finalize_message(msg, system_id, component_id, 3);
    return 0;
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public type,byte public autopilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, 2);

        memcpy(_MAV_PAYLOAD(msg), buf, 3);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = 2;

        memcpy(_MAV_PAYLOAD(msg), &packet, 3);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3);
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
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->type, heartbeat->autopilot);
}
*/
/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
 * @param autopilot Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, byte public type, byte public autopilot)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[3];
	_mav_put_byte(buf, 0, type);
	_mav_put_byte(buf, 1, autopilot);
	_mav_put_byte(buf, 2, 2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, 3);
#else
    mavlink_heartbeat_t packet;
	packet.type = type;
	packet.autopilot = autopilot;
	packet.mavlink_version = 2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, 3);
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
    return getByte(msg,  0);
}

/**
 * @brief Get field autopilot from heartbeat message
 *
 * @return Type of the Autopilot: 0: Generic, 1: PIXHAWK, 2: SLUGS, 3: Ardupilot (up to 15 types), defined in MAV_AUTOPILOT_TYPE ENUM
 */
public static byte mavlink_msg_heartbeat_get_autopilot(byte[] msg)
{
    return getByte(msg,  1);
}

/**
 * @brief Get field mavlink_version from heartbeat message
 *
 * @return MAVLink version
 */
public static byte mavlink_msg_heartbeat_get_mavlink_version(byte[] msg)
{
    return getByte(msg,  2);
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
    	heartbeat.type = mavlink_msg_heartbeat_get_type(msg);
    	heartbeat.autopilot = mavlink_msg_heartbeat_get_autopilot(msg);
    	heartbeat.mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(msg);
    
    } else {
        int len = 3; //Marshal.SizeOf(heartbeat);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        heartbeat = (mavlink_heartbeat_t)Marshal.PtrToStructure(i, ((object)heartbeat).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
