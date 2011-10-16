// MESSAGE HIL_CONTROLS PACKING
using System;
using System.Runtime.InteropServices;

public partial class Mavlink
{

    public const byte MAVLINK_MSG_ID_HIL_CONTROLS = 68;

    [StructLayout(LayoutKind.Sequential,Pack=1)]
    public struct mavlink_hil_controls_t
    {
         public  UInt64 time_us; /// Timestamp (microseconds since UNIX epoch or microseconds since system boot)
     public  Single roll_ailerons; /// Control output -3 .. 1
     public  Single pitch_elevator; /// Control output -1 .. 1
     public  Single yaw_rudder; /// Control output -1 .. 1
     public  Single throttle; /// Throttle 0 .. 1
     public  byte mode; /// System mode (MAV_MODE)
     public  byte nav_mode; /// Navigation mode (MAV_NAV_MODE)
    
    };

/**
 * @brief Pack a hil_controls message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll_ailerons Control output -3 .. 1
 * @param pitch_elevator Control output -1 .. 1
 * @param yaw_rudder Control output -1 .. 1
 * @param throttle Throttle 0 .. 1
 * @param mode System mode (MAV_MODE)
 * @param nav_mode Navigation mode (MAV_NAV_MODE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 
public static UInt16 mavlink_msg_hil_controls_pack(byte system_id, byte component_id, byte[] msg,
                               UInt64 time_us, Single roll_ailerons, Single pitch_elevator, Single yaw_rudder, Single throttle, byte mode, byte nav_mode)
{
if (MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS) {
	Array.Copy(BitConverter.GetBytes(time_us),0,msg,0,sizeof(UInt64));
	Array.Copy(BitConverter.GetBytes(roll_ailerons),0,msg,8,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(pitch_elevator),0,msg,12,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(yaw_rudder),0,msg,16,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(throttle),0,msg,20,sizeof(Single));
	Array.Copy(BitConverter.GetBytes(mode),0,msg,24,sizeof(byte));
	Array.Copy(BitConverter.GetBytes(nav_mode),0,msg,25,sizeof(byte));

} else {
    mavlink_hil_controls_t packet = new mavlink_hil_controls_t();
	packet.time_us = time_us;
	packet.roll_ailerons = roll_ailerons;
	packet.pitch_elevator = pitch_elevator;
	packet.yaw_rudder = yaw_rudder;
	packet.throttle = throttle;
	packet.mode = mode;
	packet.nav_mode = nav_mode;

        
        int len = 26;
        msg = new byte[len];
        IntPtr ptr = Marshal.AllocHGlobal(len);
        Marshal.StructureToPtr(packet, ptr, true);
        Marshal.Copy(ptr, msg, 0, len);
        Marshal.FreeHGlobal(ptr);
}

    //msg.msgid = MAVLINK_MSG_ID_HIL_CONTROLS;
    //return mavlink_finalize_message(msg, system_id, component_id, 26);
    return 0;
}

/**
 * @brief Pack a hil_controls message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll_ailerons Control output -3 .. 1
 * @param pitch_elevator Control output -1 .. 1
 * @param yaw_rudder Control output -1 .. 1
 * @param throttle Throttle 0 .. 1
 * @param mode System mode (MAV_MODE)
 * @param nav_mode Navigation mode (MAV_NAV_MODE)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
 /*
static inline uint16_t mavlink_msg_hil_controls_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   UInt64 public time_us,Single public roll_ailerons,Single public pitch_elevator,Single public yaw_rudder,Single public throttle,byte public mode,byte public nav_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_UInt64(buf, 0, time_us);
	_mav_put_Single(buf, 8, roll_ailerons);
	_mav_put_Single(buf, 12, pitch_elevator);
	_mav_put_Single(buf, 16, yaw_rudder);
	_mav_put_Single(buf, 20, throttle);
	_mav_put_byte(buf, 24, mode);
	_mav_put_byte(buf, 25, nav_mode);

        memcpy(_MAV_PAYLOAD(msg), buf, 26);
#else
    mavlink_hil_controls_t packet;
	packet.time_us = time_us;
	packet.roll_ailerons = roll_ailerons;
	packet.pitch_elevator = pitch_elevator;
	packet.yaw_rudder = yaw_rudder;
	packet.throttle = throttle;
	packet.mode = mode;
	packet.nav_mode = nav_mode;

        memcpy(_MAV_PAYLOAD(msg), &packet, 26);
#endif

    msg->msgid = MAVLINK_MSG_ID_HIL_CONTROLS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 26);
}
*/
/**
 * @brief Encode a hil_controls struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hil_controls C-struct to read the message contents from
 *//*
static inline uint16_t mavlink_msg_hil_controls_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hil_controls_t* hil_controls)
{
    return mavlink_msg_hil_controls_pack(system_id, component_id, msg, hil_controls->time_us, hil_controls->roll_ailerons, hil_controls->pitch_elevator, hil_controls->yaw_rudder, hil_controls->throttle, hil_controls->mode, hil_controls->nav_mode);
}
*/
/**
 * @brief Send a hil_controls message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param roll_ailerons Control output -3 .. 1
 * @param pitch_elevator Control output -1 .. 1
 * @param yaw_rudder Control output -1 .. 1
 * @param throttle Throttle 0 .. 1
 * @param mode System mode (MAV_MODE)
 * @param nav_mode Navigation mode (MAV_NAV_MODE)
 *//*
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hil_controls_send(mavlink_channel_t chan, UInt64 public time_us, Single public roll_ailerons, Single public pitch_elevator, Single public yaw_rudder, Single public throttle, byte public mode, byte public nav_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[26];
	_mav_put_UInt64(buf, 0, time_us);
	_mav_put_Single(buf, 8, roll_ailerons);
	_mav_put_Single(buf, 12, pitch_elevator);
	_mav_put_Single(buf, 16, yaw_rudder);
	_mav_put_Single(buf, 20, throttle);
	_mav_put_byte(buf, 24, mode);
	_mav_put_byte(buf, 25, nav_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_CONTROLS, buf, 26);
#else
    mavlink_hil_controls_t packet;
	packet.time_us = time_us;
	packet.roll_ailerons = roll_ailerons;
	packet.pitch_elevator = pitch_elevator;
	packet.yaw_rudder = yaw_rudder;
	packet.throttle = throttle;
	packet.mode = mode;
	packet.nav_mode = nav_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HIL_CONTROLS, (const char *)&packet, 26);
#endif
}

#endif
*/
// MESSAGE HIL_CONTROLS UNPACKING


/**
 * @brief Get field time_us from hil_controls message
 *
 * @return Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 */
public static UInt64 mavlink_msg_hil_controls_get_time_us(byte[] msg)
{
    return BitConverter.ToUInt64(msg,  0);
}

/**
 * @brief Get field roll_ailerons from hil_controls message
 *
 * @return Control output -3 .. 1
 */
public static Single mavlink_msg_hil_controls_get_roll_ailerons(byte[] msg)
{
    return BitConverter.ToSingle(msg,  8);
}

/**
 * @brief Get field pitch_elevator from hil_controls message
 *
 * @return Control output -1 .. 1
 */
public static Single mavlink_msg_hil_controls_get_pitch_elevator(byte[] msg)
{
    return BitConverter.ToSingle(msg,  12);
}

/**
 * @brief Get field yaw_rudder from hil_controls message
 *
 * @return Control output -1 .. 1
 */
public static Single mavlink_msg_hil_controls_get_yaw_rudder(byte[] msg)
{
    return BitConverter.ToSingle(msg,  16);
}

/**
 * @brief Get field throttle from hil_controls message
 *
 * @return Throttle 0 .. 1
 */
public static Single mavlink_msg_hil_controls_get_throttle(byte[] msg)
{
    return BitConverter.ToSingle(msg,  20);
}

/**
 * @brief Get field mode from hil_controls message
 *
 * @return System mode (MAV_MODE)
 */
public static byte mavlink_msg_hil_controls_get_mode(byte[] msg)
{
    return getByte(msg,  24);
}

/**
 * @brief Get field nav_mode from hil_controls message
 *
 * @return Navigation mode (MAV_NAV_MODE)
 */
public static byte mavlink_msg_hil_controls_get_nav_mode(byte[] msg)
{
    return getByte(msg,  25);
}

/**
 * @brief Decode a hil_controls message into a struct
 *
 * @param msg The message to decode
 * @param hil_controls C-struct to decode the message contents into
 */
public static void mavlink_msg_hil_controls_decode(byte[] msg, ref mavlink_hil_controls_t hil_controls)
{
    if (MAVLINK_NEED_BYTE_SWAP) {
    	hil_controls.time_us = mavlink_msg_hil_controls_get_time_us(msg);
    	hil_controls.roll_ailerons = mavlink_msg_hil_controls_get_roll_ailerons(msg);
    	hil_controls.pitch_elevator = mavlink_msg_hil_controls_get_pitch_elevator(msg);
    	hil_controls.yaw_rudder = mavlink_msg_hil_controls_get_yaw_rudder(msg);
    	hil_controls.throttle = mavlink_msg_hil_controls_get_throttle(msg);
    	hil_controls.mode = mavlink_msg_hil_controls_get_mode(msg);
    	hil_controls.nav_mode = mavlink_msg_hil_controls_get_nav_mode(msg);
    
    } else {
        int len = 26; //Marshal.SizeOf(hil_controls);
        IntPtr i = Marshal.AllocHGlobal(len);
        Marshal.Copy(msg, 0, i, len);
        hil_controls = (mavlink_hil_controls_t)Marshal.PtrToStructure(i, ((object)hil_controls).GetType());
        Marshal.FreeHGlobal(i);
    }
}

}
