// MESSAGE SET_ALTITUDE PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SET_ALTITUDE = 65;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_set_altitude_t
    {
        /// <summary>
        /// The system setting the altitude
        /// </summary>
        public  byte target;
            /// <summary>
        /// The new altitude in meters
        /// </summary>
        public  UInt32 mode;
    
    };

/// <summary>
/// * @brief Pack a set_altitude message
/// * @param system_id ID of this system
/// * @param component_id ID of this component (e.g. 200 for IMU)
/// * @param msg The MAVLink message to compress the data into
/// *
/// * @param target The system setting the altitude
/// * @param mode The new altitude in meters
/// * @return length of the message in bytes (excluding serial stream start sign)
/// </summary>
 
public static UInt16 mavlink_msg_set_altitude_pack(byte system_id, byte component_id, byte[] msg,
                               byte target, UInt32 mode)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(target),0,msg,0,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(mode),0,msg,1,sizeof(UInt32));

} else {
    mavlink_set_altitude_t packet = new mavlink_set_altitude_t();
	packet.target = target;
	packet.mode = mode;

        
        int len = 5;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_SET_ALTITUDE;
    //return mavlink_finalize_message(msg, system_id, component_id, 5);
    return 0;
}

/**
 * @brief Pack a set_altitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target The system setting the altitude
 * @param mode The new altitude in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_set_altitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   byte public target,UInt32 public mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[5];
	_mav_put_byte(buf, 0, target);
	_mav_put_UInt32(buf, 1, mode);

        memcpy(_MAV_PAYLOAD(msg), buf, 5);
#else
    mavlink_set_altitude_t packet;
	packet.target = target;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD(msg), &packet, 5);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_ALTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 5);
}
*/
/**
 * @brief Encode a set_altitude struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_altitude C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_set_altitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_altitude_t* set_altitude)
{
    return mavlink_msg_set_altitude_pack(system_id, component_id, msg, set_altitude->target, set_altitude->mode);
}
*/
/**
 * @brief Send a set_altitude message
 * @param chan MAVLink channel to send the message
 *
 * @param target The system setting the altitude
 * @param mode The new altitude in meters
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_altitude_send(mavlink_channel_t chan, byte public target, UInt32 public mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[5];
	_mav_put_byte(buf, 0, target);
	_mav_put_UInt32(buf, 1, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALTITUDE, buf, 5);
#else
    mavlink_set_altitude_t packet;
	packet.target = target;
	packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_ALTITUDE, (const char *)&packet, 5);
#endif
}

#endif
*/
// MESSAGE SET_ALTITUDE UNPACKING


/**
 * @brief Get field target from set_altitude message
 *
 * @return The system setting the altitude
 */
public static byte mavlink_msg_set_altitude_get_target(byte[] msg)
{
    return getByte(msg,  0);
}

/**
 * @brief Get field mode from set_altitude message
 *
 * @return The new altitude in meters
 */
public static UInt32 mavlink_msg_set_altitude_get_mode(byte[] msg)
{
    return BitConverter.ToUInt32(msg,  1);
}

/**
 * @brief Decode a set_altitude message into a struct
 *
 * @param msg The message to decode
 * @param set_altitude C-struct to decode the message contents into
 */
public static void mavlink_msg_set_altitude_decode(byte[] msg, ref mavlink_set_altitude_t set_altitude)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	set_altitude.target = mavlink_msg_set_altitude_get_target(msg);
    	set_altitude.mode = mavlink_msg_set_altitude_get_mode(msg);
    
    } else {
        int len = 5; //Marshal.SizeOf(set_altitude);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        set_altitude = (mavlink_set_altitude_t)Marshal.PtrToStructure(i, ((object)set_altitude).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
