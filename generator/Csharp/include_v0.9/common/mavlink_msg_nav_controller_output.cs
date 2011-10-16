// MESSAGE NAV_CONTROLLER_OUTPUT PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT = 62;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_nav_controller_output_t
    {
         public  Single nav_roll; /// Current desired roll in degrees
     public  Single nav_pitch; /// Current desired pitch in degrees
     public  Int16 nav_bearing; /// Current desired heading in degrees
     public  Int16 target_bearing; /// Bearing to current waypoint/target in degrees
     public  UInt16 wp_dist; /// Distance to active waypoint in meters
     public  Single alt_error; /// Current altitude error in meters
     public  Single aspd_error; /// Current airspeed error in meters/second
     public  Single xtrack_error; /// Current crosstrack error on x-y plane in meters
    
    };

/**
 * @brief Pack a nav_controller_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current waypoint/target in degrees
 * @param wp_dist Distance to active waypoint in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_nav_controller_output_pack(byte system_id, byte component_id, ref byte[] msg,
                               Single public nav_roll, Single public nav_pitch, Int16 public nav_bearing, Int16 public target_bearing, UInt16 public wp_dist, Single public alt_error, Single public aspd_error, Single public xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[26];
	_mav_put_Single(buf, 0, nav_roll);
	_mav_put_Single(buf, 4, nav_pitch);
	_mav_put_Int16(buf, 8, nav_bearing);
	_mav_put_Int16(buf, 10, target_bearing);
	_mav_put_UInt16(buf, 12, wp_dist);
	_mav_put_Single(buf, 14, alt_error);
	_mav_put_Single(buf, 18, aspd_error);
	_mav_put_Single(buf, 22, xtrack_error);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    return mavlink_finalize_message(msg, system_id, component_id, 26);
}
*/
/**
 * @brief Pack a nav_controller_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current waypoint/target in degrees
 * @param wp_dist Distance to active waypoint in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_nav_controller_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public nav_roll,Single public nav_pitch,Int16 public nav_bearing,Int16 public target_bearing,UInt16 public wp_dist,Single public alt_error,Single public aspd_error,Single public xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_Single(buf, 0, nav_roll);
	_mav_put_Single(buf, 4, nav_pitch);
	_mav_put_Int16(buf, 8, nav_bearing);
	_mav_put_Int16(buf, 10, target_bearing);
	_mav_put_UInt16(buf, 12, wp_dist);
	_mav_put_Single(buf, 14, alt_error);
	_mav_put_Single(buf, 18, aspd_error);
	_mav_put_Single(buf, 22, xtrack_error);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26);
}
*/
/**
 * @brief Encode a nav_controller_output struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nav_controller_output C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_nav_controller_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nav_controller_output_t* nav_controller_output)
{
    return mavlink_msg_nav_controller_output_pack(system_id, component_id, msg, nav_controller_output->nav_roll, nav_controller_output->nav_pitch, nav_controller_output->nav_bearing, nav_controller_output->target_bearing, nav_controller_output->wp_dist, nav_controller_output->alt_error, nav_controller_output->aspd_error, nav_controller_output->xtrack_error);
}
*/
/**
 * @brief Send a nav_controller_output message
 * @param chan MAVLink channel to send the message
 *
 * @param nav_roll Current desired roll in degrees
 * @param nav_pitch Current desired pitch in degrees
 * @param nav_bearing Current desired heading in degrees
 * @param target_bearing Bearing to current waypoint/target in degrees
 * @param wp_dist Distance to active waypoint in meters
 * @param alt_error Current altitude error in meters
 * @param aspd_error Current airspeed error in meters/second
 * @param xtrack_error Current crosstrack error on x-y plane in meters
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nav_controller_output_send(mavlink_channel_t chan, Single public nav_roll, Single public nav_pitch, Int16 public nav_bearing, Int16 public target_bearing, UInt16 public wp_dist, Single public alt_error, Single public aspd_error, Single public xtrack_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_Single(buf, 0, nav_roll);
	_mav_put_Single(buf, 4, nav_pitch);
	_mav_put_Int16(buf, 8, nav_bearing);
	_mav_put_Int16(buf, 10, target_bearing);
	_mav_put_UInt16(buf, 12, wp_dist);
	_mav_put_Single(buf, 14, alt_error);
	_mav_put_Single(buf, 18, aspd_error);
	_mav_put_Single(buf, 22, xtrack_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, buf, 26);
#else
    mavlink_nav_controller_output_t packet;
	packet.nav_roll = nav_roll;
	packet.nav_pitch = nav_pitch;
	packet.nav_bearing = nav_bearing;
	packet.target_bearing = target_bearing;
	packet.wp_dist = wp_dist;
	packet.alt_error = alt_error;
	packet.aspd_error = aspd_error;
	packet.xtrack_error = xtrack_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, (const char *)&packet, 26);
#endif
}

#endif
*/
// MESSAGE NAV_CONTROLLER_OUTPUT UNPACKING


/**
 * @brief Get field nav_roll from nav_controller_output message
 *
 * @return Current desired roll in degrees
 */
public static Single mavlink_msg_nav_controller_output_get_nav_roll(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field nav_pitch from nav_controller_output message
 *
 * @return Current desired pitch in degrees
 */
public static Single mavlink_msg_nav_controller_output_get_nav_pitch(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field nav_bearing from nav_controller_output message
 *
 * @return Current desired heading in degrees
 */
public static Int16 mavlink_msg_nav_controller_output_get_nav_bearing(byte[] msg)
{
    return BitConverter.ToInt16(msg,  8);
}

/**
 * @brief Get field target_bearing from nav_controller_output message
 *
 * @return Bearing to current waypoint/target in degrees
 */
public static Int16 mavlink_msg_nav_controller_output_get_target_bearing(byte[] msg)
{
    return BitConverter.ToInt16(msg,  10);
}

/**
 * @brief Get field wp_dist from nav_controller_output message
 *
 * @return Distance to active waypoint in meters
 */
public static UInt16 mavlink_msg_nav_controller_output_get_wp_dist(byte[] msg)
{
    return BitConverter.ToUInt16(msg,  12);
}

/**
 * @brief Get field alt_error from nav_controller_output message
 *
 * @return Current altitude error in meters
 */
public static Single mavlink_msg_nav_controller_output_get_alt_error(byte[] msg)
{
    return BitConverter.ToSingle(msg,  14);
}

/**
 * @brief Get field aspd_error from nav_controller_output message
 *
 * @return Current airspeed error in meters/second
 */
public static Single mavlink_msg_nav_controller_output_get_aspd_error(byte[] msg)
{
    return BitConverter.ToSingle(msg,  18);
}

/**
 * @brief Get field xtrack_error from nav_controller_output message
 *
 * @return Current crosstrack error on x-y plane in meters
 */
public static Single mavlink_msg_nav_controller_output_get_xtrack_error(byte[] msg)
{
    return BitConverter.ToSingle(msg,  22);
}

/**
 * @brief Decode a nav_controller_output message into a struct
 *
 * @param msg The message to decode
 * @param nav_controller_output C-struct to decode the message contents into
 */
public static void mavlink_msg_nav_controller_output_decode(byte[] msg, ref mavlink_nav_controller_output_t nav_controller_output)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	nav_controller_output.nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(msg);
	nav_controller_output.nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(msg);
	nav_controller_output.nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(msg);
	nav_controller_output.target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(msg);
	nav_controller_output.wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(msg);
	nav_controller_output.alt_error = mavlink_msg_nav_controller_output_get_alt_error(msg);
	nav_controller_output.aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(msg);
	nav_controller_output.xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(msg);
} else {
    int len = 26; //Marshal.SizeOf(nav_controller_output);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    nav_controller_output = (mavlink_nav_controller_output_t)Marshal.PtrToStructure(i, ((object)nav_controller_output).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
