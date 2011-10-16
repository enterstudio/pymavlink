// MESSAGE VFR_HUD PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_VFR_HUD = 74;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_vfr_hud_t
    {
         public  Single airspeed; /// Current airspeed in m/s
     public  Single groundspeed; /// Current ground speed in m/s
     public  Single alt; /// Current altitude (MSL), in meters
     public  Single climb; /// Current climb rate in meters/second
     public  Int16 heading; /// Current heading in degrees, in compass units (0..360, 0=north)
     public  UInt16 throttle; /// Current throttle setting in integer percent, 0 to 100
    
    };

/**
 * @brief Pack a vfr_hud message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_vfr_hud_pack(byte system_id, byte component_id, ref byte[] msg,
                               Single public airspeed, Single public groundspeed, Int16 public heading, UInt16 public throttle, Single public alt, Single public climb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[20];
	_mav_put_Single(buf, 0, airspeed);
	_mav_put_Single(buf, 4, groundspeed);
	_mav_put_Single(buf, 8, alt);
	_mav_put_Single(buf, 12, climb);
	_mav_put_Int16(buf, 16, heading);
	_mav_put_UInt16(buf, 18, throttle);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_vfr_hud_t packet;
	packet.airspeed = airspeed;
	packet.groundspeed = groundspeed;
	packet.alt = alt;
	packet.climb = climb;
	packet.heading = heading;
	packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_VFR_HUD;
    return mavlink_finalize_message(msg, system_id, component_id, 20, 20);
}
*/
/**
 * @brief Pack a vfr_hud message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_vfr_hud_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public airspeed,Single public groundspeed,Int16 public heading,UInt16 public throttle,Single public alt,Single public climb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Single(buf, 0, airspeed);
	_mav_put_Single(buf, 4, groundspeed);
	_mav_put_Single(buf, 8, alt);
	_mav_put_Single(buf, 12, climb);
	_mav_put_Int16(buf, 16, heading);
	_mav_put_UInt16(buf, 18, throttle);

        memcpy(_MAV_PAYLOAD(msg), buf, 20);
#else
    mavlink_vfr_hud_t packet;
	packet.airspeed = airspeed;
	packet.groundspeed = groundspeed;
	packet.alt = alt;
	packet.climb = climb;
	packet.heading = heading;
	packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD(msg), &packet, 20);
#endif

    msg->msgid = MAVLINK_MSG_ID_VFR_HUD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 20, 20);
}
*/
/**
 * @brief Encode a vfr_hud struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vfr_hud C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_vfr_hud_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vfr_hud_t* vfr_hud)
{
    return mavlink_msg_vfr_hud_pack(system_id, component_id, msg, vfr_hud->airspeed, vfr_hud->groundspeed, vfr_hud->heading, vfr_hud->throttle, vfr_hud->alt, vfr_hud->climb);
}
*/
/**
 * @brief Send a vfr_hud message
 * @param chan MAVLink channel to send the message
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vfr_hud_send(mavlink_channel_t chan, Single public airspeed, Single public groundspeed, Int16 public heading, UInt16 public throttle, Single public alt, Single public climb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[20];
	_mav_put_Single(buf, 0, airspeed);
	_mav_put_Single(buf, 4, groundspeed);
	_mav_put_Single(buf, 8, alt);
	_mav_put_Single(buf, 12, climb);
	_mav_put_Int16(buf, 16, heading);
	_mav_put_UInt16(buf, 18, throttle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, buf, 20, 20);
#else
    mavlink_vfr_hud_t packet;
	packet.airspeed = airspeed;
	packet.groundspeed = groundspeed;
	packet.alt = alt;
	packet.climb = climb;
	packet.heading = heading;
	packet.throttle = throttle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VFR_HUD, (const char *)&packet, 20, 20);
#endif
}

#endif
*/
// MESSAGE VFR_HUD UNPACKING


/**
 * @brief Get field airspeed from vfr_hud message
 *
 * @return Current airspeed in m/s
 */
public static Single mavlink_msg_vfr_hud_get_airspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field groundspeed from vfr_hud message
 *
 * @return Current ground speed in m/s
 */
public static Single mavlink_msg_vfr_hud_get_groundspeed(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field heading from vfr_hud message
 *
 * @return Current heading in degrees, in compass units (0..360, 0=north)
 */
public static Int16 mavlink_msg_vfr_hud_get_heading(byte[] msg)
{
    return BitConverter.ToInt16(msg,  16);
}

/**
 * @brief Get field throttle from vfr_hud message
 *
 * @return Current throttle setting in integer percent, 0 to 100
 */
public static UInt16 mavlink_msg_vfr_hud_get_throttle(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  18);
}

/**
 * @brief Get field alt from vfr_hud message
 *
 * @return Current altitude (MSL), in meters
 */
public static Single mavlink_msg_vfr_hud_get_alt(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field climb from vfr_hud message
 *
 * @return Current climb rate in meters/second
 */
public static Single mavlink_msg_vfr_hud_get_climb(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Decode a vfr_hud message into a struct
 *
 * @param msg The message to decode
 * @param vfr_hud C-struct to decode the message contents into
 */
public static void mavlink_msg_vfr_hud_decode(byte[] msg, ref mavlink_vfr_hud_t vfr_hud)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	vfr_hud.airspeed = mavlink_msg_vfr_hud_get_airspeed(msg);
	vfr_hud.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(msg);
	vfr_hud.alt = mavlink_msg_vfr_hud_get_alt(msg);
	vfr_hud.climb = mavlink_msg_vfr_hud_get_climb(msg);
	vfr_hud.heading = mavlink_msg_vfr_hud_get_heading(msg);
	vfr_hud.throttle = mavlink_msg_vfr_hud_get_throttle(msg);
} else {
    int len = 20; //Marshal.SizeOf(vfr_hud);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    vfr_hud = (mavlink_vfr_hud_t)Marshal.PtrToStructure(i, ((object)vfr_hud).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
