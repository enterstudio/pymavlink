// MESSAGE SLUGS_NAVIGATION PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_SLUGS_NAVIGATION = 176;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_slugs_navigation_t
    {
         public  Single u_m; /// Measured Airspeed prior to the Nav Filter
     public  Single phi_c; /// Commanded Roll
     public  Single theta_c; /// Commanded Pitch
     public  Single psiDot_c; /// Commanded Turn rate
     public  Single ay_body; /// Y component of the body acceleration
     public  Single totalDist; /// Total Distance to Run on this leg of Navigation
     public  Single dist2Go; /// Remaining distance to Run on this leg of Navigation
     public  byte fromWP; /// Origin WP
     public  byte toWP; /// Destination WP
    
    };

/**
 * @brief Pack a slugs_navigation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param u_m Measured Airspeed prior to the Nav Filter
 * @param phi_c Commanded Roll
 * @param theta_c Commanded Pitch
 * @param psiDot_c Commanded Turn rate
 * @param ay_body Y component of the body acceleration
 * @param totalDist Total Distance to Run on this leg of Navigation
 * @param dist2Go Remaining distance to Run on this leg of Navigation
 * @param fromWP Origin WP
 * @param toWP Destination WP
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static uint16 mavlink_msg_slugs_navigation_pack(byte system_id, byte component_id, ref byte[] msg,
                               Single public u_m, Single public phi_c, Single public theta_c, Single public psiDot_c, Single public ay_body, Single public totalDist, Single public dist2Go, byte public fromWP, byte public toWP)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    byte buf[30];
	_mav_put_Single(buf, 0, u_m);
	_mav_put_Single(buf, 4, phi_c);
	_mav_put_Single(buf, 8, theta_c);
	_mav_put_Single(buf, 12, psiDot_c);
	_mav_put_Single(buf, 16, ay_body);
	_mav_put_Single(buf, 20, totalDist);
	_mav_put_Single(buf, 24, dist2Go);
	_mav_put_byte(buf, 28, fromWP);
	_mav_put_byte(buf, 29, toWP);

        memcpy(_MAV_PAYLOAD(msg), buf, 30);
#else
    mavlink_slugs_navigation_t packet;
	packet.u_m = u_m;
	packet.phi_c = phi_c;
	packet.theta_c = theta_c;
	packet.psiDot_c = psiDot_c;
	packet.ay_body = ay_body;
	packet.totalDist = totalDist;
	packet.dist2Go = dist2Go;
	packet.fromWP = fromWP;
	packet.toWP = toWP;

        memcpy(_MAV_PAYLOAD(msg), &packet, 30);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_NAVIGATION;
    return mavlink_finalize_message(msg, system_id, component_id, 30);
}
*/
/**
 * @brief Pack a slugs_navigation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param u_m Measured Airspeed prior to the Nav Filter
 * @param phi_c Commanded Roll
 * @param theta_c Commanded Pitch
 * @param psiDot_c Commanded Turn rate
 * @param ay_body Y component of the body acceleration
 * @param totalDist Total Distance to Run on this leg of Navigation
 * @param dist2Go Remaining distance to Run on this leg of Navigation
 * @param fromWP Origin WP
 * @param toWP Destination WP
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_slugs_navigation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   Single public u_m,Single public phi_c,Single public theta_c,Single public psiDot_c,Single public ay_body,Single public totalDist,Single public dist2Go,byte public fromWP,byte public toWP)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[30];
	_mav_put_Single(buf, 0, u_m);
	_mav_put_Single(buf, 4, phi_c);
	_mav_put_Single(buf, 8, theta_c);
	_mav_put_Single(buf, 12, psiDot_c);
	_mav_put_Single(buf, 16, ay_body);
	_mav_put_Single(buf, 20, totalDist);
	_mav_put_Single(buf, 24, dist2Go);
	_mav_put_byte(buf, 28, fromWP);
	_mav_put_byte(buf, 29, toWP);

        memcpy(_MAV_PAYLOAD(msg), buf, 30);
#else
    mavlink_slugs_navigation_t packet;
	packet.u_m = u_m;
	packet.phi_c = phi_c;
	packet.theta_c = theta_c;
	packet.psiDot_c = psiDot_c;
	packet.ay_body = ay_body;
	packet.totalDist = totalDist;
	packet.dist2Go = dist2Go;
	packet.fromWP = fromWP;
	packet.toWP = toWP;

        memcpy(_MAV_PAYLOAD(msg), &packet, 30);
#endif

    msg->msgid = MAVLINK_MSG_ID_SLUGS_NAVIGATION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 30);
}
*/
/**
 * @brief Encode a slugs_navigation struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param slugs_navigation C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_slugs_navigation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_slugs_navigation_t* slugs_navigation)
{
    return mavlink_msg_slugs_navigation_pack(system_id, component_id, msg, slugs_navigation->u_m, slugs_navigation->phi_c, slugs_navigation->theta_c, slugs_navigation->psiDot_c, slugs_navigation->ay_body, slugs_navigation->totalDist, slugs_navigation->dist2Go, slugs_navigation->fromWP, slugs_navigation->toWP);
}
*/
/**
 * @brief Send a slugs_navigation message
 * @param chan MAVLink channel to send the message
 *
 * @param u_m Measured Airspeed prior to the Nav Filter
 * @param phi_c Commanded Roll
 * @param theta_c Commanded Pitch
 * @param psiDot_c Commanded Turn rate
 * @param ay_body Y component of the body acceleration
 * @param totalDist Total Distance to Run on this leg of Navigation
 * @param dist2Go Remaining distance to Run on this leg of Navigation
 * @param fromWP Origin WP
 * @param toWP Destination WP
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_slugs_navigation_send(mavlink_channel_t chan, Single public u_m, Single public phi_c, Single public theta_c, Single public psiDot_c, Single public ay_body, Single public totalDist, Single public dist2Go, byte public fromWP, byte public toWP)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[30];
	_mav_put_Single(buf, 0, u_m);
	_mav_put_Single(buf, 4, phi_c);
	_mav_put_Single(buf, 8, theta_c);
	_mav_put_Single(buf, 12, psiDot_c);
	_mav_put_Single(buf, 16, ay_body);
	_mav_put_Single(buf, 20, totalDist);
	_mav_put_Single(buf, 24, dist2Go);
	_mav_put_byte(buf, 28, fromWP);
	_mav_put_byte(buf, 29, toWP);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_NAVIGATION, buf, 30);
#else
    mavlink_slugs_navigation_t packet;
	packet.u_m = u_m;
	packet.phi_c = phi_c;
	packet.theta_c = theta_c;
	packet.psiDot_c = psiDot_c;
	packet.ay_body = ay_body;
	packet.totalDist = totalDist;
	packet.dist2Go = dist2Go;
	packet.fromWP = fromWP;
	packet.toWP = toWP;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SLUGS_NAVIGATION, (const char *)&packet, 30);
#endif
}

#endif
*/
// MESSAGE SLUGS_NAVIGATION UNPACKING


/**
 * @brief Get field u_m from slugs_navigation message
 *
 * @return Measured Airspeed prior to the Nav Filter
 */
public static Single mavlink_msg_slugs_navigation_get_u_m(byte[] msg)
{
    return BitConverter.ToSingle(msg,  0);
}

/**
 * @brief Get field phi_c from slugs_navigation message
 *
 * @return Commanded Roll
 */
public static Single mavlink_msg_slugs_navigation_get_phi_c(byte[] msg)
{
    return BitConverter.ToSingle(msg,  4);
}

/**
 * @brief Get field theta_c from slugs_navigation message
 *
 * @return Commanded Pitch
 */
public static Single mavlink_msg_slugs_navigation_get_theta_c(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field psiDot_c from slugs_navigation message
 *
 * @return Commanded Turn rate
 */
public static Single mavlink_msg_slugs_navigation_get_psiDot_c(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field ay_body from slugs_navigation message
 *
 * @return Y component of the body acceleration
 */
public static Single mavlink_msg_slugs_navigation_get_ay_body(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field totalDist from slugs_navigation message
 *
 * @return Total Distance to Run on this leg of Navigation
 */
public static Single mavlink_msg_slugs_navigation_get_totalDist(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field dist2Go from slugs_navigation message
 *
 * @return Remaining distance to Run on this leg of Navigation
 */
public static Single mavlink_msg_slugs_navigation_get_dist2Go(byte[] msg)
{
    return BitConverter.ToSingle(msg,  24);
}

/**
 * @brief Get field fromWP from slugs_navigation message
 *
 * @return Origin WP
 */
public static byte mavlink_msg_slugs_navigation_get_fromWP(byte[] msg)
{
    return getByte(msg,  28);
}

/**
 * @brief Get field toWP from slugs_navigation message
 *
 * @return Destination WP
 */
public static byte mavlink_msg_slugs_navigation_get_toWP(byte[] msg)
{
    return getByte(msg,  29);
}

/**
 * @brief Decode a slugs_navigation message into a struct
 *
 * @param msg The message to decode
 * @param slugs_navigation C-struct to decode the message contents into
 */
public static void mavlink_msg_slugs_navigation_decode(byte[] msg, ref mavlink_slugs_navigation_t slugs_navigation)
{
if (MAVLINK_NEED_BYTE_SWAP) {
	slugs_navigation.u_m = mavlink_msg_slugs_navigation_get_u_m(msg);
	slugs_navigation.phi_c = mavlink_msg_slugs_navigation_get_phi_c(msg);
	slugs_navigation.theta_c = mavlink_msg_slugs_navigation_get_theta_c(msg);
	slugs_navigation.psiDot_c = mavlink_msg_slugs_navigation_get_psiDot_c(msg);
	slugs_navigation.ay_body = mavlink_msg_slugs_navigation_get_ay_body(msg);
	slugs_navigation.totalDist = mavlink_msg_slugs_navigation_get_totalDist(msg);
	slugs_navigation.dist2Go = mavlink_msg_slugs_navigation_get_dist2Go(msg);
	slugs_navigation.fromWP = mavlink_msg_slugs_navigation_get_fromWP(msg);
	slugs_navigation.toWP = mavlink_msg_slugs_navigation_get_toWP(msg);
} else {
    int len = 30; //Marshal.SizeOf(slugs_navigation);
    IntPtr i = Marshal.AllocHGlobal(len);
    Marshal.Copy(msg, 0, i, len);
    slugs_navigation = (mavlink_slugs_navigation_t)Marshal.PtrToStructure(i, ((object)slugs_navigation).GetType());
    Marshal.FreeHGlobal(i);
}
}

}
